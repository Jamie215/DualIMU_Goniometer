/*
 * KNEE ANGLE - SLAVE (shank)  [UART, quaternion orientation, runtime config]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270 + BMM150)
 *
 * Runs a Mahony filter and answers the master over UART. The knee angle is
 * computed on the PC from the RELATIVE rotation of the two segments (see
 * master_imu.ino), so how the board is strapped to the shank does not matter.
 *
 * Magnetometer (9-DOF) is CONFIGURED AT RUNTIME via a config packet relayed by
 * the master -- no reflashing to calibrate or to switch 6-DOF/9-DOF. Until a
 * config with use_mag=1 arrives, the board runs 6-DOF (accel+gyro).
 *
 * UART master<->slave (binary), one request byte:
 *   'R' -> 18-byte quaternion packet [0xAA, q0..q3 (LE), xor of 1..16]
 *   'r' -> 26-byte raw packet        [0xBB, ax,ay,az,mx,my,mz (LE), xor of 1..24]
 *   'W' + 32-byte config payload:
 *        [0]=use_mag, [1..12]=bias(3 floats), [13..24]=scale(3 floats),
 *        [25..27]=perm(0..2), [28..30]=sign(0:+,1:-), [31]=xor of [0..30]
 *        -> replies 0x06 (ACK) on good checksum, 0x15 (NAK) otherwise.
 *
 * Wiring: Slave TX(D1)->Master RX(D0), Slave RX(D0)<-Master TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

// ---- Mahony filter state --------------------------------------------------
const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // cached latest orientation
unsigned long lastMicros = 0;

// ---- runtime magnetometer config (set by relayed 'W'; 6-DOF until then) ---
bool  useMagCfg   = false;
float MAG_BIAS[3]  = {0.0f, 0.0f, 0.0f};
float MAG_SCALE[3] = {1.0f, 1.0f, 1.0f};
int   MAG_PERM[3]  = {0, 1, 2};
int   MAG_SIGN[3]  = {1, 1, 1};
float magX = 0.0f, magY = 0.0f, magZ = 0.0f;

// Latest RAW accel+mag captured together at magnetometer rate, for the PC's
// calibration ('r' poll). Paired at the same instant so accel.mag stays a valid
// invariant for the axis solver. rawReady stays false until the first mag read.
float rawAx = 0, rawAy = 0, rawAz = 0, rawMx = 0, rawMy = 0, rawMz = 0;
bool  rawReady = false;

inline void magToImuFrame(float &mx, float &my, float &mz) {
  float in[3] = {mx, my, mz};
  mx = MAG_SIGN[0] * in[MAG_PERM[0]];
  my = MAG_SIGN[1] * in[MAG_PERM[1]];
  mz = MAG_SIGN[2] * in[MAG_PERM[2]];
}

inline void calibrateMag(float &mx, float &my, float &mz) {
  mx = (mx - MAG_BIAS[0]) * MAG_SCALE[0];
  my = (my - MAG_BIAS[1]) * MAG_SCALE[1];
  mz = (mz - MAG_BIAS[2]) * MAG_SCALE[2];
  magToImuFrame(mx, my, mz);
}

void seedFromAccel(float ax, float ay, float az) {
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float cr = cos(roll * 0.5f),  sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f), sp = sin(pitch * 0.5f);
  q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
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
    gx += TWO_KP * halfex; gy += TWO_KP * halfey; gz += TWO_KP * halfez;
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

inline void sendQuat() {
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

inline void sendRaw() {
  if (!rawReady) return;   // no paired sample yet; master skips this poll
  uint8_t pkt[26];
  pkt[0] = 0xBB;
  memcpy(&pkt[1],  &rawAx, 4); memcpy(&pkt[5],  &rawAy, 4); memcpy(&pkt[9],  &rawAz, 4);
  memcpy(&pkt[13], &rawMx, 4); memcpy(&pkt[17], &rawMy, 4); memcpy(&pkt[21], &rawMz, 4);
  uint8_t cs = 0;
  for (int i = 1; i <= 24; i++) cs ^= pkt[i];
  pkt[25] = cs;
  Serial1.write(pkt, 26);
}

inline void recvConfig() {
  uint8_t pl[32];
  int idx = 0;
  unsigned long t0 = micros();
  while (idx < 32 && (micros() - t0) < 50000) {
    if (Serial1.available()) pl[idx++] = Serial1.read();
  }
  if (idx < 32) return;                    // incomplete -> master times out (ERR,S)
  uint8_t cs = 0;
  for (int i = 0; i < 31; i++) cs ^= pl[i];
  if (cs != pl[31]) { Serial1.write((uint8_t)0x15); return; }

  useMagCfg = (pl[0] != 0);
  memcpy(MAG_BIAS,  &pl[1],  12);
  memcpy(MAG_SCALE, &pl[13], 12);
  MAG_PERM[0] = pl[25]; MAG_PERM[1] = pl[26]; MAG_PERM[2] = pl[27];
  MAG_SIGN[0] = pl[28] ? -1 : 1;
  MAG_SIGN[1] = pl[29] ? -1 : 1;
  MAG_SIGN[2] = pl[30] ? -1 : 1;
  if (!useMagCfg) { magX = magY = magZ = 0.0f; }
  Serial1.write((uint8_t)0x06);            // ACK
}

inline void serviceRequest() {
  if (!Serial1.available()) return;
  char c = Serial1.read();
  if      (c == 'R') sendQuat();
  else if (c == 'r') sendRaw();
  else if (c == 'W') recvConfig();
}

void loop() {
  serviceRequest();  // answer fast, top priority

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    serviceRequest();
    IMU.readGyroscope(gx, gy, gz);        // deg/s
    serviceRequest();

    // When a fresh mag sample is ready, cache it paired with THIS accel (for the
    // PC's 'r' calibration poll), and, if configured, feed it into the filter.
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(rawMx, rawMy, rawMz);  // uT, raw
      rawAx = ax; rawAy = ay; rawAz = az;
      rawReady = true;
      if (useMagCfg) {
        float mx = rawMx, my = rawMy, mz = rawMz;
        calibrateMag(mx, my, mz);
        magX = mx; magY = my; magZ = mz;
      }
      serviceRequest();
    }

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                 ax, ay, az, magX, magY, magZ, dt);
  }

  serviceRequest();
}
