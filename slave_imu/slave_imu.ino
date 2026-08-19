/*
 * KNEE ANGLE - SLAVE (shank)  [UART, 6-DOF quaternion + raw gravity]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270; magnetometer unused)
 *
 * On request ('R') replies with its orientation quaternion AND its raw
 * accelerometer vector (see master_imu.ino for the packet + the handedness note).
 *
 * Reply packet (30 bytes):
 *   [0] 0xAA, [1..16] float q0..q3, [17..28] float ax,ay,az, [29] XOR of [1..28]
 *
 * Wiring: Slave TX(D1)->Master RX(D0), Slave RX(D0)<-Master TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.02f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // cached latest orientation
float accX = 0.0f, accY = 0.0f, accZ = 1.0f;        // cached latest raw accel (g)
unsigned long lastMicros = 0;

float gyroBias[3] = {0.0f, 0.0f, 0.0f};
const float BIAS_SANITY_DPS = 3.0f;

// Accelerometer-as-gravity trust vs how far |accel| is from 1 g. SOFT gate
// (matches master): full trust near static, ramping to zero as linear
// acceleration grows, so the filter is always partly drift-corrected while the
// gyro carries the fast part -- no open-loop drift/snap-back after a fast move.
const float ACC_TRUST_FULL_G = 0.10f;
const float ACC_TRUST_ZERO_G = 0.60f;

// Arduino_BMI270_BMM150 returns a REFLECTED (left-handed) frame; negate x of
// accel AND gyro to restore a right-handed frame for the quaternion filter.
inline void readAccel(float &ax, float &ay, float &az) {
  IMU.readAcceleration(ax, ay, az); ax = -ax;
}
inline void readGyro(float &gx, float &gy, float &gz) {
  IMU.readGyroscope(gx, gy, gz); gx = -gx;
}

void calibrateGyroBias() {
  const int N = 300;
  float sx = 0, sy = 0, sz = 0;
  int got = 0;
  unsigned long t0 = millis();
  while (got < N && millis() - t0 < 4000) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      readGyro(gx, gy, gz);
      sx += gx; sy += gy; sz += gz; got++;
    }
  }
  if (got > 0) {
    float bx = sx / got, by = sy / got, bz = sz / got;
    float m = max(fabs(bx), max(fabs(by), fabs(bz)));
    if (m <= BIAS_SANITY_DPS) {
      gyroBias[0] = bx; gyroBias[1] = by; gyroBias[2] = bz;
    }
  }
}

void seedFromAccel(float ax, float ay, float az) {
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float cr = cos(roll * 0.5f),  sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f), sp = sin(pitch * 0.5f);
  q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
}

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  float amag = sqrt(ax * ax + ay * ay + az * az);
  if (amag > 1e-6f) {
    float err = fabs(amag - 1.0f);
    float trust;
    if (err <= ACC_TRUST_FULL_G)      trust = 1.0f;
    else if (err >= ACC_TRUST_ZERO_G) trust = 0.0f;
    else trust = (ACC_TRUST_ZERO_G - err) / (ACC_TRUST_ZERO_G - ACC_TRUST_FULL_G);

    if (trust > 0.0f) {
      float recipNorm = 1.0f / amag;
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

      float halfvx = q1 * q3 - q0 * q2;
      float halfvy = q0 * q1 + q2 * q3;
      float halfvz = q0 * q0 - 0.5f + q3 * q3;

      float halfex = (ay * halfvz - az * halfvy) * trust;
      float halfey = (az * halfvx - ax * halfvz) * trust;
      float halfez = (ax * halfvy - ay * halfvx) * trust;

      if (TWO_KI > 0.0f) {
        integralFBx += TWO_KI * halfex * dt;
        integralFBy += TWO_KI * halfey * dt;
        integralFBz += TWO_KI * halfez * dt;
        gx += integralFBx; gy += integralFBy; gz += integralFBz;
      }
      gx += TWO_KP * halfex; gy += TWO_KP * halfey; gz += TWO_KP * halfez;
    }
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
  Serial1.begin(460800);        // fast board-to-board link (must match master)

  if (!IMU.begin()) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) { digitalWrite(LED_BUILTIN, HIGH); delay(150);
                digitalWrite(LED_BUILTIN, LOW);  delay(150); }
  }

  calibrateGyroBias();                 // keep the board STILL at startup

  lastMicros = micros();
  unsigned long t0 = millis();
  while (!IMU.accelerationAvailable() && millis() - t0 < 2000) { ; }
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    readAccel(ax, ay, az);
    accX = ax; accY = ay; accZ = az;
    seedFromAccel(ax, ay, az);
  }
}

inline void sendPacket() {
  uint8_t pkt[30];
  pkt[0] = 0xAA;
  memcpy(&pkt[1],  &q0, 4);
  memcpy(&pkt[5],  &q1, 4);
  memcpy(&pkt[9],  &q2, 4);
  memcpy(&pkt[13], &q3, 4);
  memcpy(&pkt[17], &accX, 4);
  memcpy(&pkt[21], &accY, 4);
  memcpy(&pkt[25], &accZ, 4);
  uint8_t cs = 0;
  for (int i = 1; i <= 28; i++) cs ^= pkt[i];
  pkt[29] = cs;
  Serial1.write(pkt, 30);
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
    readAccel(ax, ay, az);
    serviceRequest();
    readGyro(gx, gy, gz);             // deg/s, handedness-corrected
    gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];
    serviceRequest();

    accX = ax; accY = ay; accZ = az;  // cache raw gravity for the reply

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);
  }

  serviceRequest();
}
