/*
 * KNEE ANGLE - MASTER (thigh)  [UART, 6-DOF quaternion]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270; magnetometer unused)
 *
 * 6-DOF Mahony filter (accel + gyro). Each segment reports its orientation as a
 * unit quaternion; the knee angle is derived on the PC from the RELATIVE rotation
 * of the two segments, so sensor placement is tolerated. See knee_collector_uart.py.
 *
 * The master reads a FIXED 18-byte binary packet from the slave:
 *   [0] 0xAA header, [1..16] float q0..q3 (LE, w,x,y,z), [17] XOR checksum[1..16]
 * The slave sample is bracketed at (T_req + T_resp) / 2.
 *
 * PC line (12 fields):
 *   D,<t_thigh_us>,<tw>,<tx>,<ty>,<tz>,<t_shank_mid_us>,<sw>,<sx>,<sy>,<sz>,<rtt_us>
 * On a bad/missing reply, the shank quaternion and midpoint are 0.
 *
 * Wiring: Master TX(D1)->Slave RX(D0), Master RX(D0)<-Slave TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

// Mahony gains. Kp pulls the estimate toward gravity; Ki estimates and cancels
// the gyro bias, which is the main drift source in a 6-DOF (no-mag) filter.
const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.02f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long lastMicros = 0;

const unsigned long SLAVE_TIMEOUT_US = 8000;

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

    // Estimated direction of gravity (half-vector).
    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error = cross(measured gravity, estimated gravity).
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
  while (Serial1.available()) Serial1.read();
  unsigned long tReq = micros();
  Serial1.write('R');

  uint8_t pkt[18];
  int idx = 0;
  while (idx < 18 && (micros() - tReq) < SLAVE_TIMEOUT_US) {
    if (Serial1.available()) pkt[idx++] = Serial1.read();
  }
  unsigned long tResp = micros();
  if (idx < 18 || pkt[0] != 0xAA) return 0;
  uint8_t cs = 0;
  for (int i = 1; i <= 16; i++) cs ^= pkt[i];
  if (cs != pkt[17]) return 0;
  memcpy(q, &pkt[1], 16);
  rtt = tResp - tReq;
  return (tReq + tResp) / 2;
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);      // deg/s

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);

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
