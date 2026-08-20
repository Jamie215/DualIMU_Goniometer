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
 * The shank board STREAMS its state continuously (~104 Hz); this board is a passive
 * listener. Each loop it drains its UART buffer, keeping the freshest complete
 * packet, and merges it onto its own clock -- it never blocks waiting on the slave,
 * so a slave stall just ages the last packet instead of dropping a sample.
 *
 * Slave stream packet (30 bytes):
 *   [0] 0xAA header
 *   [1..16]  float q0..q3        (LE, w,x,y,z)
 *   [17..28] float ax,ay,az      (LE, g, handedness-corrected)
 *   [29] XOR checksum of [1..28]
 *
 * PC line (18 fields):
 *   D,t_thigh_us,tw,tx,ty,tz,tax,tay,taz,t_shank_recv_us,sw,sx,sy,sz,sax,say,saz,age_us
 * The last field is the freshest shank packet's AGE in us (0 if none yet). When the
 * newest packet is older than SHANK_STALE_US the shank fields and its timestamp are
 * 0, so the collector marks the sample invalid and forward-fills it.
 *
 * Wiring: Slave TX(D1)->Master RX(D0), GND<->GND. (Master no longer transmits.)
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

// The shank streams a packet every ~9.6 ms (~104 Hz). We treat the newest packet
// as a live shank sample until it is older than this; past it, the slave is
// presumed stalled and the sample is emitted invalid for the collector to fill.
// 30 ms is ~3 packet intervals: it absorbs normal phase jitter and the brief mbed
// RTOS stalls without lying about the data (shank orientation barely moves in
// 30 ms), while still flagging a genuinely dead slave promptly.
const unsigned long SHANK_STALE_US = 30000;

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
                                // not 460800: a 30-byte packet at 104 Hz needs only
                                // ~25 kbaud, and the two boards clock this async link
                                // off independent oscillators that drift apart as they
                                // warm -- the lower rate keeps ample timing margin so
                                // framing errors don't grow over a session.
  while (!Serial) { ; }

  if (!IMU.begin()) {
    Serial.println("ERR,IMU init failed");
    while (1) { ; }
  }
  Serial.println("# MASTER fw: stream-2 (shank streams; age_us fail-safe)");
  Serial.println("# MASTER cols: D,t_thigh_us,tw,tx,ty,tz,tax,tay,taz,"
                 "t_shank_recv_us,sw,sx,sy,sz,sax,say,saz,age_us");

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

// Freshest shank state received from the stream, plus when (master clock) it was
// parsed. shankRecvUs == 0 until the first good packet arrives.
float shankQ[4] = {0, 0, 0, 0};
float shankA[3] = {0, 0, 0};
unsigned long shankRecvUs = 0;

// Persistent frame-assembly state for the free-running stream. We resync on the
// 0xAA header and validate the XOR checksum, so a lost/extra byte costs one frame
// (checksum fail -> drop -> resync to the next header), never a lasting desync.
uint8_t rxBuf[30];
int rxHave = 0;

// Non-blocking: consume every byte currently buffered, updating the cache with the
// LAST complete, checksum-good packet. Called often so the UART buffer never backs
// up; whatever the slave streamed while we were busy is waiting here, not lost.
void pumpShankStream() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (rxHave == 0) {
      if (b == 0xAA) { rxBuf[0] = b; rxHave = 1; }   // wait for a header to start
    } else {
      rxBuf[rxHave++] = b;
      if (rxHave == 30) {
        uint8_t cs = 0;
        for (int i = 1; i <= 28; i++) cs ^= rxBuf[i];
        if (cs == rxBuf[29]) {                        // good frame -> update cache
          memcpy(shankQ, &rxBuf[1], 16);
          memcpy(shankA, &rxBuf[17], 12);
          shankRecvUs = micros();
        }
        rxHave = 0;   // start the next frame (bad checksum just resyncs on 0xAA)
      }
    }
  }
}

void loop() {
  pumpShankStream();   // keep draining even between our own IMU samples

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

    pumpShankStream();                // grab the freshest packet before emitting
    // Age the packet against a timestamp taken AFTER the final pump: that pump can
    // parse a packet whose micros() stamp is later than `now` (captured above), and
    // an unsigned `now - shankRecvUs` would then underflow to ~4.29e9 and look stale.
    // Fail SAFE regardless: if tRef ever reads before the packet stamp (reordering
    // or the ~71 min micros() rollover), clamp age to 0 (just-received) instead of
    // letting an unsigned wrap flag a good sample invalid.
    unsigned long tRef = micros();
    unsigned long age = (shankRecvUs == 0 || tRef < shankRecvUs) ? 0
                                                                 : (tRef - shankRecvUs);
    bool shankFresh = (shankRecvUs != 0) && (age <= SHANK_STALE_US);

    Serial.print("D,");
    Serial.print(now);        Serial.print(',');
    Serial.print(q0, 4);      Serial.print(',');
    Serial.print(q1, 4);      Serial.print(',');
    Serial.print(q2, 4);      Serial.print(',');
    Serial.print(q3, 4);      Serial.print(',');
    Serial.print(ax, 4);      Serial.print(',');
    Serial.print(ay, 4);      Serial.print(',');
    Serial.print(az, 4);      Serial.print(',');
    // Shank block: freshest packet if within SHANK_STALE_US, else the zero sentinel
    // (timestamp + quaternion + accel all 0) so the collector marks it invalid.
    if (shankFresh) {
      Serial.print(shankRecvUs); Serial.print(',');
      Serial.print(shankQ[0], 4); Serial.print(',');
      Serial.print(shankQ[1], 4); Serial.print(',');
      Serial.print(shankQ[2], 4); Serial.print(',');
      Serial.print(shankQ[3], 4); Serial.print(',');
      Serial.print(shankA[0], 4); Serial.print(',');
      Serial.print(shankA[1], 4); Serial.print(',');
      Serial.print(shankA[2], 4); Serial.print(',');
    } else {
      Serial.print(0); Serial.print(',');   // t_shank_recv_us
      for (int i = 0; i < 7; i++) { Serial.print(0); Serial.print(','); }  // q+accel
    }
    Serial.println(age);      // freshest shank packet age (us); 0 if none yet
  }
}
