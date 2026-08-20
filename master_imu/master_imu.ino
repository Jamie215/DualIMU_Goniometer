/*
 * KNEE ANGLE - MASTER (thigh)  [UART, 6-DOF quaternion + raw gravity]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270; magnetometer unused)
 *
 * Streams, per segment: an orientation quaternion (6-DOF Mahony) AND the raw
 * accelerometer vector. The collector derives the knee angle from the gravity
 * direction in each board's own frame, taken from the QUATERNION (gyro-fused,
 * so it stays smooth through fast motion and drift-free in tilt). The raw accel
 * is still sent for the filter-free cross-check in --monitor.
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

// Accelerometer-as-gravity trust, as a function of how far |accel| is from 1 g.
// A hard on/off gate let the filter run gyro-only through a fast move, so it
// drifted and then snapped back on the way out (the "wrong reading right after
// returning from a big angle" symptom). Instead we SOFT-gate: full accel trust
// when nearly static, ramping linearly to zero as linear acceleration grows, so
// some drift correction is always applied while the gyro carries the fast part.
const float ACC_TRUST_FULL_G = 0.10f;   // within this of 1 g -> trust accel fully
const float ACC_TRUST_ZERO_G = 0.60f;   // beyond this -> gyro only (no correction)

// A 30-byte reply at 115200 baud is ~2.6 ms on the wire, so a healthy round-trip
// is ~3-4 ms. But the slave only answers 'R' at points in its OWN ~104 Hz loop,
// and the mbed RTOS underneath can add sporadic multi-ms stalls, so a reply can
// legitimately arrive several ms late. A tight window drops those as "invalid".
//
// We deliberately trade peak rate for completeness: this window is wide enough to
// catch a reply delayed by nearly a full slave loop. It is almost free -- the
// master loop is gated by its own IMU (~9.6 ms/sample), so a wider timeout only
// extends the occasional LATE poll, not every one; the average rate stays high.
// Knee flexion has negligible content above ~10 Hz, so even the worst-case rate
// this implies is ample. Widen it further (and accept a lower rate) if valid% is
// still low; the failure diagnostics below (rtt on a dropped poll) tell you which
// way to tune -- see pollSlave().
const unsigned long SLAVE_TIMEOUT_US = 12000;

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
  if (amag > 1e-6f) {
    // Soft gate: 1.0 near static, ramping to 0.0 as |accel| leaves 1 g.
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
  Serial.begin(115200);
  Serial1.begin(115200);        // board-to-board link (must match slave). 115200,
                                // not 460800: a 30-byte reply at 104 Hz needs only
                                // ~25 kbaud, and the two boards clock this async link
                                // off independent oscillators that drift apart as they
                                // warm -- the lower rate keeps ample timing margin so
                                // framing errors don't grow over a session.
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
//
// The reply is framed by a 0xAA header, so we SCAN for that header before reading
// the 29-byte body instead of reading 30 bytes blind. Reading blind meant a single
// lost or extra byte on the async link shifted every following packet by one, and
// the misalignment persisted (each poll's "30 bytes" straddled a packet boundary),
// which is what makes rtt climb and validity fall together over a session. With a
// header resync a glitch costs one packet, not a self-sustaining run of them.
//
// rtt is ALWAYS set to the elapsed poll time, even on failure, so a dropped sample
// is self-diagnosing in the log: rtt near SLAVE_TIMEOUT_US means the slave never
// answered in time (too-slow response -> widen the window / lower the rate), while
// a SMALL rtt on a dropped sample means bytes arrived but were corrupt (checksum /
// framing -> a baud, wiring, or GND problem). A moderate rtt with a valid sample is
// the healthy case.
unsigned long pollSlave(float q[4], float a[3], unsigned long &rtt) {
  while (Serial1.available()) Serial1.read();   // drop stale bytes before framing
  unsigned long tReq = micros();
  Serial1.write('R');

  // Resync: read forward until the 0xAA header (or time out). Right after the
  // flush the next thing on the wire is a fresh reply, so the header is normally
  // the first byte; the scan only spends work when stray bytes precede it.
  bool haveHeader = false;
  while ((micros() - tReq) < SLAVE_TIMEOUT_US) {
    if (Serial1.available() && Serial1.read() == 0xAA) { haveHeader = true; break; }
  }
  if (!haveHeader) { rtt = micros() - tReq; return 0; }   // no reply in the window

  uint8_t pkt[30];
  pkt[0] = 0xAA;
  int idx = 1;
  while (idx < 30 && (micros() - tReq) < SLAVE_TIMEOUT_US) {
    if (Serial1.available()) pkt[idx++] = Serial1.read();
  }
  unsigned long tResp = micros();
  rtt = tResp - tReq;
  if (idx < 30) return 0;                        // body didn't finish in the window
  uint8_t cs = 0;
  for (int i = 1; i <= 28; i++) cs ^= pkt[i];    // checksum guards a false 0xAA match
  if (cs != pkt[29]) return 0;                   // bytes arrived but were corrupt
  memcpy(q, &pkt[1], 16);
  memcpy(a, &pkt[17], 12);
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
