/*
 * KNEE ANGLE - MASTER (thigh)  [UART, 6-DOF quaternion + raw gravity]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270; magnetometer unused)
 *
 * Streams, per segment: an orientation quaternion (6-DOF Mahony) AND the raw
 * accelerometer vector. The collector's default "gravity" angle uses the RAW
 * accelerometer (drift-free, filter-free -- stationary => flat), while the
 * quaternion is available for the optional gyro-fused method.
 *
 * IMPORTANT axis note: Arduino_BMI270_BMM150 returns accel/gyro as
 *   x=-sensor.y, y=-sensor.x, z=sensor.z  (determinant -1, a REFLECTION -> a
 * left-handed frame). A quaternion filter needs a right-handed frame, so we
 * negate x of accel AND gyro to restore det +1. Without this the gyro's rotation
 * sense is mirrored vs the accel and the filter wanders.
 *
 * Slave reply packet (30 bytes):
 *   [0] 0xAA header
 *   [1..16]  float q0..q3        (LE, w,x,y,z)
 *   [17..28] float ax,ay,az      (LE, g, handedness-corrected)
 *   [29] XOR checksum of [1..28]
 *
 * PC line (18 fields):
 *   D,t_thigh_us,tw,tx,ty,tz,tax,tay,taz,t_shank_mid_us,sw,sx,sy,sz,sax,say,saz,rtt_us
 * On a bad/missing reply, shank quaternion+accel and midpoint are 0.
 *
 * Wiring: Master TX(D1)->Slave RX(D0), Master RX(D0)<-Slave TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.02f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long lastMicros = 0;

// Resting gyro bias (deg/s), measured at startup. Guarded: if the board wasn't
// still (measured bias too large), we discard it rather than inject a false rate.
float gyroBias[3] = {0.0f, 0.0f, 0.0f};
const float BIAS_SANITY_DPS = 3.0f;

// Trust the accelerometer as "gravity" only when its magnitude is near 1 g;
// during fast motion linear acceleration corrupts it, so we let the gyro carry.
const float ACC_GATE_G = 0.15f;

const unsigned long SLAVE_TIMEOUT_US = 8000;

// Read sensors with the reflection fixed (negate x -> right-handed frame).
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
    if (m <= BIAS_SANITY_DPS) {          // trust only if the board was still
      gyroBias[0] = bx; gyroBias[1] = by; gyroBias[2] = bz;
    }  // else leave 0 (a moving startup would otherwise destabilize the filter)
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
  if (amag > 1e-6f && fabs(amag - 1.0f) < ACC_GATE_G) {
    float recipNorm = 1.0f / amag;
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
  Serial.begin(115200);
  Serial1.begin(460800);        // fast board-to-board link (must match slave)
  while (!Serial) { ; }

  if (!IMU.begin()) {
    Serial.println("ERR,IMU init failed");
    while (1) { ; }
  }
  Serial.println("# MASTER cols: D,t_thigh_us,tw,tx,ty,tz,tax,tay,taz,"
                 "t_shank_mid_us,sw,sx,sy,sz,sax,say,saz,rtt_us");

  calibrateGyroBias();                 // keep the board STILL at startup
  Serial.print("# master gyro bias dps: ");
  Serial.print(gyroBias[0], 3); Serial.print(',');
  Serial.print(gyroBias[1], 3); Serial.print(',');
  Serial.println(gyroBias[2], 3);

  lastMicros = micros();
  unsigned long t0 = millis();
  while (!IMU.accelerationAvailable() && millis() - t0 < 2000) { ; }
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    readAccel(ax, ay, az);
    seedFromAccel(ax, ay, az);
  }
}

// Returns midpoint timestamp on success (fills q[4]/a[3]/rtt), or 0 on failure.
unsigned long pollSlave(float q[4], float a[3], unsigned long &rtt) {
  while (Serial1.available()) Serial1.read();
  unsigned long tReq = micros();
  Serial1.write('R');

  uint8_t pkt[30];
  int idx = 0;
  while (idx < 30 && (micros() - tReq) < SLAVE_TIMEOUT_US) {
    if (Serial1.available()) pkt[idx++] = Serial1.read();
  }
  unsigned long tResp = micros();
  if (idx < 30 || pkt[0] != 0xAA) return 0;
  uint8_t cs = 0;
  for (int i = 1; i <= 28; i++) cs ^= pkt[i];
  if (cs != pkt[29]) return 0;
  memcpy(q, &pkt[1], 16);
  memcpy(a, &pkt[17], 12);
  rtt = tResp - tReq;
  return (tReq + tResp) / 2;
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);              // deg/s, handedness-corrected
    gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);

    float sq[4] = {0, 0, 0, 0};
    float sa[3] = {0, 0, 0};
    unsigned long rtt = 0;
    unsigned long sMid = pollSlave(sq, sa, rtt);

    Serial.print("D,");
    Serial.print(now);      Serial.print(',');
    Serial.print(q0, 4);    Serial.print(',');
    Serial.print(q1, 4);    Serial.print(',');
    Serial.print(q2, 4);    Serial.print(',');
    Serial.print(q3, 4);    Serial.print(',');
    Serial.print(ax, 4);    Serial.print(',');
    Serial.print(ay, 4);    Serial.print(',');
    Serial.print(az, 4);    Serial.print(',');
    Serial.print(sMid);     Serial.print(',');
    Serial.print(sq[0], 4); Serial.print(',');
    Serial.print(sq[1], 4); Serial.print(',');
    Serial.print(sq[2], 4); Serial.print(',');
    Serial.print(sq[3], 4); Serial.print(',');
    Serial.print(sa[0], 4); Serial.print(',');
    Serial.print(sa[1], 4); Serial.print(',');
    Serial.print(sa[2], 4); Serial.print(',');
    Serial.println(rtt);
  }
}
