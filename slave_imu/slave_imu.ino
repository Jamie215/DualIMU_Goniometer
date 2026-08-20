/*
 * KNEE ANGLE - SLAVE (shank)  [UART, 6-DOF quaternion + raw gravity]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270; magnetometer unused)
 *
 * STREAMS its orientation quaternion AND raw accelerometer vector continuously,
 * one packet per IMU update (~104 Hz) -- it does NOT wait to be polled. The old
 * request/response ('R' -> reply) coupled the master to the slave's response
 * latency: the slave only answered between chunks of its own loop, and the mbed
 * RTOS underneath adds sporadic multi-ms stalls, so a reply could arrive after the
 * master's timeout and the sample was lost. Streaming removes that deadline: the
 * master reads whatever complete packets are already in its UART buffer and uses
 * the freshest, never blocking on the slave. A slave stall just makes the newest
 * packet a little older; nothing is dropped waiting on it.
 *
 * Packet (30 bytes), framed for resync on a free-running stream:
 *   [0] 0xAA, [1..16] float q0..q3, [17..28] float ax,ay,az, [29] XOR of [1..28]
 * (See master_imu.ino for the handedness note.)
 *
 * Wiring: Slave TX(D1)->Master RX(D0), GND<->GND. (Master RX is all that's used
 * now; the slave no longer reads its RX line.)
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
const unsigned long MIN_SAMPLES_PER_WINDOW = 60;   // healthy ~104; below this = degraded
unsigned long sampleCount  = 0;
unsigned long rateWindowMs = 0;

// Health indicator on the built-in LED (pin 13). The onboard RGB LED is dead on
// this unit, so state is encoded as BLINK PATTERN instead of colour, which stays
// unambiguous with a single LED:
//   steady HEARTBEAT blink = healthy, streaming
//   SOLID on               = sensor stalled (loop running, re-init firing)
//   3 fast flashes at start = boot -- at plug-in confirms this firmware is flashed;
//                             mid-session means the board reset (brown-out/fault)
//   OFF / frozen           = loop not running (hard fault) or unpowered
const unsigned long HEARTBEAT_MS = 150;   // healthy blink half-period (~3 Hz)
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

  calibrateGyroBias();                 // keep the board STILL at startup
  reinitSeed(2000);                    // wait for data, seed orientation from gravity
  lastSampleMs = millis();
  rateWindowMs = millis();
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
  Serial1.write(pkt, 30);           // ~30 B at 104 Hz = ~27% of the 115200 link
}

void loop() {
  // Rate watchdog: once per window, re-init the sensor if too few samples streamed.
  // Heals a bad low-rate power-up (~10 Hz) automatically, no manual reset needed.
  unsigned long tw = millis();
  if (tw - rateWindowMs >= RATE_WINDOW_MS) {
    if (sampleCount < MIN_SAMPLES_PER_WINDOW) reinitIMU();
    sampleCount = 0;
    rateWindowMs = millis();
  }

  // Free-running: update orientation on each IMU sample and stream one packet.
  // No RX handling -- the master is a passive listener on this link now.
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);             // deg/s, handedness-corrected
    gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];

    accX = ax; accY = ay; accZ = az;  // cache raw gravity for the packet

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);
    sendPacket();                     // stream the freshly-updated orientation

    sampleCount++;
    lastSampleMs = millis();
    // healthy: steady heartbeat blink (non-blocking toggle)
    if (millis() - lastBlinkMs >= HEARTBEAT_MS) {
      lastBlinkMs = millis();
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
    return;
  }

  // No new IMU data. If the sensor has been silent long enough to be a real stall
  // (not just the sub-10 ms wait between samples), make it visible and recover it.
  unsigned long nowMs = millis();
  if (nowMs - lastSampleMs > STALL_WARN_MS) {
    digitalWrite(LED_BUILTIN, HIGH);              // SOLID on = sensor stalled
    if (nowMs - lastReinitMs > REINIT_EVERY_MS) reinitIMU();
  }
}
