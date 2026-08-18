/*
 * KNEE ANGLE - SLAVE (shank)  [UART, 6-DOF quaternion]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270; magnetometer unused)
 *
 * 6-DOF Mahony filter (accel + gyro). On request ('R') it replies with its
 * orientation quaternion. The knee angle is computed on the PC from the RELATIVE
 * rotation of the two segments (see master_imu.ino), so how the board is strapped
 * to the shank does not matter as long as it's rigid.
 *
 * Reply packet (18 bytes):
 *   [0] 0xAA header, [1..16] float q0..q3 (LE, w,x,y,z), [17] XOR checksum[1..16]
 *
 * Wiring: Slave TX(D1)->Master RX(D0), Slave RX(D0)<-Master TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.02f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // cached latest orientation
unsigned long lastMicros = 0;

void seedFromAccel(float ax, float ay, float az) {
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float cr = cos(roll * 0.5f),  sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f), sp = sin(pitch * 0.5f);
  q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
}

// Mahony AHRS update, IMU-only (accel + gyro). Gyro in rad/s.
void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    float halfex = (ay * halfvz - az * halfvy);
    float halfey = (az * halfvx - ax * halfvz);
    float halfez = (ax * halfvy - ay * halfvx);

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

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);
  }

  serviceRequest();
}
