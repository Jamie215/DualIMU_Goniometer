/*
 * KNEE ANGLE - MASTER (thigh)  [UART, quaternion orientation]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270 + BMM150)
 *
 * Each segment runs a Mahony filter and reports its orientation as a unit
 * quaternion (w,x,y,z). The knee angle is derived on the PC from the RELATIVE
 * rotation between the two segments, so arbitrary sensor placement is tolerated
 * (a constant sensor-to-segment misalignment cancels in the relative delta).
 * See knee_collector_uart.py.
 *
 * USE_MAG selects the fusion:
 *   1 -> 9-DOF (accel + gyro + magnetometer): absolute heading, no yaw drift,
 *        BUT the magnetometer MUST be calibrated (see mag_calibrate.ino) and is
 *        sensitive to nearby metal (braces, treadmills, rebar floors).
 *   0 -> 6-DOF (accel + gyro): robust and calibration-free, but yaw drifts.
 * Flip this and reflash to A/B test which is better in your environment.
 *
 * Slave reply packet (18 bytes):
 *   [0] 0xAA header, [1..16] float q0..q3 (LE, w,x,y,z), [17] XOR checksum[1..16]
 *
 * PC line (text):
 *   D,<t_thigh_us>,<tw>,<tx>,<ty>,<tz>,<t_shank_mid_us>,<sw>,<sx>,<sy>,<sz>,<rtt_us>
 * On a bad/missing reply, the shank quaternion and midpoint are 0.
 *
 * Wiring: Master TX(D1)->Slave RX(D0), Master RX(D0)<-Slave TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

#define USE_MAG 1   // 1 = 9-DOF (needs mag calibration below); 0 = 6-DOF

// ---- Mahony filter state --------------------------------------------------
// Kp trades convergence speed for noise rejection; Ki slowly corrects gyro bias.
const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long lastMicros = 0;

#if USE_MAG
// ---- magnetometer calibration (THIS board) --------------------------------
// Fill from mag_calibrate.ino. Defaults are pass-through == uncalibrated ==
// meaningless heading, so calibrate before trusting 9-DOF.
const float MAG_BIAS[3]  = {0.0f, 0.0f, 0.0f};   // hard-iron offset, uT
const float MAG_SCALE[3] = {1.0f, 1.0f, 1.0f};   // soft-iron scale

// The BMM150 and BMI270 do not necessarily share axes. Rotate the raw mag sample
// into the accel/gyro frame here; verify with mag_calibrate.ino. Identity default.
inline void magToImuFrame(float &mx, float &my, float &mz) {
  // e.g. if X/Y are swapped: float t = mx; mx = my; my = t;
}

inline void calibrateMag(float &mx, float &my, float &mz) {
  magToImuFrame(mx, my, mz);
  mx = (mx - MAG_BIAS[0]) * MAG_SCALE[0];
  my = (my - MAG_BIAS[1]) * MAG_SCALE[1];
  mz = (mz - MAG_BIAS[2]) * MAG_SCALE[2];
}

float magX = 0.0f, magY = 0.0f, magZ = 0.0f;   // latest calibrated mag
#else
float magX = 0.0f, magY = 0.0f, magZ = 0.0f;   // always 0 -> IMU-only
#endif

// The pure serial round-trip is ~165 us at 115200 baud ('R' + 18-byte reply).
// 8 ms leaves slack for slave-side scheduling and still fits inside one ~9.6 ms
// IMU sample period, so a late reply can't push the master off its 104 Hz cadence.
const unsigned long SLAVE_TIMEOUT_US = 8000;

// Seed the quaternion from gravity (roll/pitch from accel, yaw = 0). With the
// magnetometer on, the filter pulls yaw to magnetic heading within a second.
void seedFromAccel(float ax, float ay, float az) {
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float cr = cos(roll * 0.5f),  sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f), sp = sin(pitch * 0.5f);
  q0 = cr * cp;
  q1 = sr * cp;
  q2 = cr * sp;
  q3 = -sr * sp;
}

// Mahony AHRS update. Gyro in rad/s. If mag is (0,0,0) it degrades to IMU-only,
// so the same call works whether or not USE_MAG / a mag sample is available.
void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz, float dt) {
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  bool useMag = !(mx == 0.0f && my == 0.0f && mz == 0.0f);

  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    // Estimated direction of gravity (half-vector).
    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;

    if (useMag) {
      float recipMag = 1.0f / sqrt(mx * mx + my * my + mz * mz);
      mx *= recipMag; my *= recipMag; mz *= recipMag;

      // Reference direction of Earth's magnetic field.
      float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
      float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
      float bx = sqrt(hx * hx + hy * hy);
      float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

      // Estimated direction of the magnetic field (half-vector).
      float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
      float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
      float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

      halfex = (my * halfwz - mz * halfwy);
      halfey = (mz * halfwx - mx * halfwz);
      halfez = (mx * halfwy - my * halfwx);
    }

    // Add the gravity error term (cross of measured and estimated gravity).
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);

    if (TWO_KI > 0.0f) {
      integralFBx += TWO_KI * halfex * dt;
      integralFBy += TWO_KI * halfey * dt;
      integralFBz += TWO_KI * halfez * dt;
      gx += integralFBx; gy += integralFBy; gz += integralFBz;
    }
    gx += TWO_KP * halfex;
    gy += TWO_KP * halfey;
    gz += TWO_KP * halfez;
  }

  // Integrate the quaternion rate.
  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  float recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial) { ; }

  if (!IMU.begin()) {
    Serial.println("ERR,IMU init failed");
    while (1) { ; }
  }
  Serial.println("# MASTER cols: D,t_thigh_us,tw,tx,ty,tz,t_shank_mid_us,sw,sx,sy,sz,rtt_us");

  lastMicros = micros();
  unsigned long t0 = millis();
  while (!IMU.accelerationAvailable() && millis() - t0 < 2000) { ; }
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    seedFromAccel(ax, ay, az);
  }
}

// Returns midpoint timestamp on success (fills q[4]/rtt), or 0 on failure.
unsigned long pollSlave(float q[4], unsigned long &rtt) {
  while (Serial1.available()) Serial1.read();   // flush stale

  unsigned long tReq = micros();
  Serial1.write('R');

  uint8_t pkt[18];
  int idx = 0;
  while (idx < 18 && (micros() - tReq) < SLAVE_TIMEOUT_US) {
    if (Serial1.available()) pkt[idx++] = Serial1.read();
  }
  unsigned long tResp = micros();
  if (idx < 18) return 0;             // timeout / incomplete
  if (pkt[0] != 0xAA) return 0;       // bad header

  uint8_t cs = 0;
  for (int i = 1; i <= 16; i++) cs ^= pkt[i];
  if (cs != pkt[17]) return 0;        // checksum fail

  memcpy(q, &pkt[1], 16);
  rtt = tResp - tReq;
  return (tReq + tResp) / 2;
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);      // deg/s

#if USE_MAG
    if (IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz); // uT
      calibrateMag(mx, my, mz);
      magX = mx; magY = my; magZ = mz;   // cache; used every filter tick
    }
#endif

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                 ax, ay, az, magX, magY, magZ, dt);

    float sq[4] = {0, 0, 0, 0};
    unsigned long rtt = 0;
    unsigned long sMid = pollSlave(sq, rtt);

    Serial.print("D,");
    Serial.print(now);      Serial.print(',');
    Serial.print(q0, 4);    Serial.print(',');
    Serial.print(q1, 4);    Serial.print(',');
    Serial.print(q2, 4);    Serial.print(',');
    Serial.print(q3, 4);    Serial.print(',');
    Serial.print(sMid);     Serial.print(',');
    Serial.print(sq[0], 4); Serial.print(',');
    Serial.print(sq[1], 4); Serial.print(',');
    Serial.print(sq[2], 4); Serial.print(',');
    Serial.print(sq[3], 4); Serial.print(',');
    Serial.println(rtt);
  }
}
