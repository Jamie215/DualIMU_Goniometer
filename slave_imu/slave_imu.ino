/*
 * KNEE ANGLE - SLAVE (shank)  [UART, 6-DOF quaternion + raw gravity]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270; magnetometer unused)
 *
 * STREAMS its orientation quaternion AND raw accelerometer vector at a fixed 50 Hz
 * while a collection is active -- it does NOT wait to be polled. The master reads
 * whatever complete packets are already in its UART buffer and uses the freshest,
 * never blocking on the slave; a slave stall just ages the newest packet.
 *
 * Collect-on-demand: the slave runs the IMU only while the master's keepalive ('S')
 * is arriving (i.e. while the PC has the master's USB port open). It idles when the
 * keepalive stops or on 'X', so the sensor isn't run while nothing is observing.
 *
 * Packet (30 bytes), framed for resync on a free-running stream:
 *   [0] 0xAA, [1..16] float q0..q3, [17..28] float ax,ay,az, [29] XOR of [1..28]
 * (See master_imu.ino for the handedness note.)
 *
 * Wiring (BOTH directions): Slave TX(D1)->Master RX(D0) for the stream, and
 * Master TX(D1)->Slave RX(D0) for the keepalive, plus GND<->GND.
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

// Sensor-stall recovery. The board streams only while the BMI270 keeps asserting
// new data; if data-ready gets stuck the loop would otherwise spin silently for
// seconds (the multi-second shank outages seen in the field, master healthy the
// whole time). So: if no IMU sample arrives for STALL_WARN_MS, show it on the RGB
// LED (below) and periodically re-init the sensor to recover in ms instead of
// waiting for it to un-stick on its own.
const unsigned long STALL_WARN_MS   = 250;
const unsigned long REINIT_EVERY_MS = 500;
unsigned long lastSampleMs = 0;
unsigned long lastReinitMs = 0;

// Low-rate self-heal (the "I had to reset the sensor every session" fix). A healthy
// BMI270 streams ~104 Hz; on a cold power-up it sometimes comes up misconfigured at
// a low rate (~10 Hz) with bad data, which used to need a MANUAL board reset. The
// master gets a fresh init for free every session (opening its USB port resets it),
// but the slave free-runs from power with nothing to re-init it. This watchdog gives
// the slave the same clean start automatically: if it streams fewer than
// MIN_SAMPLES_PER_WINDOW in a RATE_WINDOW_MS window, force a full sensor re-init --
// at boot AND mid-session, so a bad power-up state heals itself within ~1 s.
const unsigned long RATE_WINDOW_MS         = 1000;
const unsigned long MIN_SAMPLES_PER_WINDOW = 30;   // healthy ~50 stream; below this = degraded
unsigned long sampleCount  = 0;
unsigned long rateWindowMs = 0;

// Fixed stream rate (the 50 Hz baseline). The filter still updates on every IMU
// sample for smoothness, but we transmit at most one packet per STREAM_PERIOD_US.
// Halving the packets on the wire is the point: far more timing margin on the async
// board-to-board link, and 50 Hz is ample for knee ROM.
const unsigned long STREAM_PERIOD_US = 20000;   // 50 Hz
unsigned long lastSendUs = 0;

// Collect-on-demand. The slave runs the IMU only while the master says a collection
// is active. The master forwards a keepalive ('S') while its USB port is open; the
// slave activates on it and idles once the keepalive stops arriving (port closed,
// master reset, or unplugged). 'X' idles immediately. This keeps the sensor from
// running while nothing is observing, and keeps both boards in lockstep.
bool active = false;
unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 1000;   // idle if no keepalive within this

// Health indicator on the built-in LED (pin 13). The onboard RGB LED is dead on
// this unit, so state is encoded as BLINK PATTERN instead of colour:
//   fast HEARTBEAT blink   = active, streaming
//   slow blink             = idle, waiting for a collection to start
//   SOLID on               = sensor stalled (loop running, re-init firing)
//   3 fast flashes at start = boot -- confirms this firmware is flashed
//   OFF / frozen           = loop not running (hard fault) or unpowered
const unsigned long HEARTBEAT_MS = 150;   // active blink half-period (~3 Hz)
const unsigned long IDLE_BLINK_MS = 700;  // idle blink half-period (slow)
bool ledState = false;
unsigned long lastBlinkMs = 0;

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
  Serial1.begin(115200);        // board-to-board link (must match master). Lowered
                                // from 460800 for async-clock timing margin -- see
                                // master_imu.ino for the rationale.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Boot blink: 3 fast flashes on every (re)start. Seen at plug-in it confirms
  // THIS firmware is flashed; seen MID-SESSION it means the board just reset
  // (brown-out / fault -> slave power), vs. a sensor hang which holds it SOLID.
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(60);
    digitalWrite(LED_BUILTIN, LOW);  delay(120);
  }

  if (!IMU.begin()) {
    while (1) { digitalWrite(LED_BUILTIN, HIGH); delay(150);   // fast blink = IMU init failed
                digitalWrite(LED_BUILTIN, LOW);  delay(150); }
  }
  // No calibration or streaming at boot -- the board idles until the master signals
  // a collection has begun (see activate()), so the IMU isn't run while unobserved.
}

// Begin an active collection: calibrate the gyro bias (board should be still) and
// seed orientation from gravity, fresh for this session.
void activate() {
  calibrateGyroBias();
  reinitSeed(2000);
  lastSampleMs  = millis();
  rateWindowMs  = millis();
  lastSendUs    = micros();
  lastCmdMs     = millis();            // set AFTER the ~3 s calibrate so we don't instantly time out
  active = true;
}

// Wait (up to timeout) for the accelerometer to produce data, then seed the
// orientation from gravity. Shared by startup and the self-heal re-init.
void reinitSeed(unsigned long timeout_ms) {
  lastMicros = micros();
  unsigned long t0 = millis();
  while (!IMU.accelerationAvailable() && millis() - t0 < timeout_ms) { ; }
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    readAccel(ax, ay, az);
    accX = ax; accY = ay; accZ = az;
    seedFromAccel(ax, ay, az);
  }
}

// Full sensor re-init: re-configure the BMI270 (restores its ~104 Hz output rate
// if it came up wrong) and re-seed. This is what a manual board reset was doing
// by hand; the watchdogs call it automatically.
void reinitIMU() {
  IMU.begin();
  reinitSeed(300);
  lastSampleMs = millis();
  lastReinitMs = millis();
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
  Serial1.write(pkt, 30);           // ~30 B at 50 Hz = ~13% of the 115200 link
}

void loop() {
  // Follow the master's collect/idle commands. 'S' (start / keepalive) keeps us
  // active; 'X' idles immediately; no keepalive within CMD_TIMEOUT_MS also idles.
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == 'S') { lastCmdMs = millis(); if (!active) activate(); }
    else if (c == 'X') { active = false; }
  }

  // Idle when not in a collection: don't touch the IMU, just show a slow blink.
  if (!active || (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    active = false;
    if (millis() - lastBlinkMs >= IDLE_BLINK_MS) {
      lastBlinkMs = millis();
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
    return;
  }

  // Rate watchdog: once per window, re-init the sensor if too few samples streamed.
  // Heals a bad low-rate state (~10 Hz) automatically, no manual reset needed.
  unsigned long tw = millis();
  if (tw - rateWindowMs >= RATE_WINDOW_MS) {
    if (sampleCount < MIN_SAMPLES_PER_WINDOW) reinitIMU();
    sampleCount = 0;
    rateWindowMs = millis();
  }

  // Active: process + stream at a fixed 50 Hz (at most one packet per STREAM_PERIOD_US).
  if ((micros() - lastSendUs) >= STREAM_PERIOD_US
      && IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    lastSendUs = micros();
    float ax, ay, az, gx, gy, gz;
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);             // deg/s, handedness-corrected
    gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];

    accX = ax; accY = ay; accZ = az;  // cache raw gravity for the packet

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 50.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);
    sendPacket();                     // stream the freshly-updated orientation

    sampleCount++;
    lastSampleMs = millis();
    // active: fast heartbeat blink (non-blocking toggle)
    if (millis() - lastBlinkMs >= HEARTBEAT_MS) {
      lastBlinkMs = millis();
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
    return;
  }

  // Active but no new IMU data for a while -> real stall: make it visible and recover.
  unsigned long nowMs = millis();
  if (nowMs - lastSampleMs > STALL_WARN_MS) {
    digitalWrite(LED_BUILTIN, HIGH);              // SOLID on = sensor stalled
    if (nowMs - lastReinitMs > REINIT_EVERY_MS) reinitIMU();
  }
}
