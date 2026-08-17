/*
 * KNEE ANGLE - SLAVE (shank)  [UART, quaternion orientation]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270 + BMM150)
 *
 * Runs a Mahony filter and, on request, replies with its orientation quaternion.
 * The knee angle is computed on the PC from the RELATIVE rotation of the two
 * segments (see master_imu.ino), so how the board is strapped to the shank does
 * not matter as long as it's rigid.
 *
 * USE_MAG selects the fusion (keep it the SAME as the master):
 *   1 -> 9-DOF (accel + gyro + magnetometer): needs mag calibration below and is
 *        sensitive to nearby metal.
 *   0 -> 6-DOF (accel + gyro): robust, calibration-free, but yaw drifts.
 *
 * Reply packet (18 bytes):
 *   [0] 0xAA header, [1..16] float q0..q3 (LE, w,x,y,z), [17] XOR checksum[1..16]
 * A fixed binary packet keeps the round-trip well under 1 ms, comfortably inside
 * the master's 8 ms poll budget.
 *
 * Wiring: Slave TX(D1)->Master RX(D0), Slave RX(D0)<-Master TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

#define USE_MAG 1   // 1 = 9-DOF (needs mag calibration below); 0 = 6-DOF

// ---- Mahony filter state (see master_imu.ino for the derivation) ----------
const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // cached latest orientation
unsigned long lastMicros = 0;

#if USE_MAG
// ---- magnetometer calibration (THIS board) --------------------------------
// Fill from mag_calibrate.ino run on the SHANK board. Defaults == uncalibrated.
const float MAG_BIAS[3]  = {0.0f, 0.0f, 0.0f};   // hard-iron offset, uT
const float MAG_SCALE[3] = {1.0f, 1.0f, 1.0f};   // soft-iron scale

// The BMM150 and BMI270 do NOT share axes on this board. mag_calibrate.ino
// measures the correct mapping and prints this function body; paste it in.
inline void magToImuFrame(float &mx, float &my, float &mz) {
  float in0 = mx, in1 = my, in2 = mz;
  mx = in0;   // <- replace with the mapping printed by mag_calibrate.ino
  my = in1;
  mz = in2;
}

inline void calibrateMag(float &mx, float &my, float &mz) {
  // Hard-/soft-iron correction in the RAW sensor frame, THEN remap to IMU frame.
  mx = (mx - MAG_BIAS[0]) * MAG_SCALE[0];
  my = (my - MAG_BIAS[1]) * MAG_SCALE[1];
  mz = (mz - MAG_BIAS[2]) * MAG_SCALE[2];
  magToImuFrame(mx, my, mz);
}

float magX = 0.0f, magY = 0.0f, magZ = 0.0f;
#else
float magX = 0.0f, magY = 0.0f, magZ = 0.0f;   // always 0 -> IMU-only
#endif

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

// Mahony AHRS update. Gyro in rad/s. Degrades to IMU-only when mag is (0,0,0).
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

    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;

    if (useMag) {
      float recipMag = 1.0f / sqrt(mx * mx + my * my + mz * mz);
      mx *= recipMag; my *= recipMag; mz *= recipMag;

      float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
      float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
      float bx = sqrt(hx * hx + hy * hy);
      float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

      float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
      float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
      float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

      halfex = (my * halfwz - mz * halfwy);
      halfey = (mz * halfwx - mx * halfwz);
      halfez = (mx * halfwy - my * halfwx);
    }

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
  Serial1.begin(115200);

  if (!IMU.begin()) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) { digitalWrite(LED_BUILTIN, HIGH); delay(150);
                digitalWrite(LED_BUILTIN, LOW);  delay(150); }
  }

  lastMicros = micros();

  unsigned long t0 = millis();
  while (!IMU.accelerationAvailable() && millis() - t0 < 2000) { ; }
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    seedFromAccel(ax, ay, az);
  }
}

inline void sendPacket() {
  uint8_t pkt[18];
  pkt[0] = 0xAA;
  memcpy(&pkt[1],  &q0, 4);
  memcpy(&pkt[5],  &q1, 4);
  memcpy(&pkt[9],  &q2, 4);
  memcpy(&pkt[13], &q3, 4);
  uint8_t cs = 0;
  for (int i = 1; i <= 16; i++) cs ^= pkt[i];
  pkt[17] = cs;
  Serial1.write(pkt, 18);
}

inline void serviceRequest() {
  if (Serial1.available()) {
    char c = Serial1.read();
    if (c == 'R') sendPacket();
  }
}

void loop() {
  serviceRequest();  // answer fast, top priority

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    serviceRequest();
    IMU.readGyroscope(gx, gy, gz);      // deg/s
    serviceRequest();

#if USE_MAG
    if (IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz); // uT
      calibrateMag(mx, my, mz);
      magX = mx; magY = my; magZ = mz;
      serviceRequest();
    }
#endif

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                 ax, ay, az, magX, magY, magZ, dt);
  }

  serviceRequest();
}
