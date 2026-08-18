/*
 * KNEE ANGLE - MASTER (thigh)  [UART, quaternion orientation, runtime config]
 * Board: Arduino Nano 33 BLE Rev2  (onboard BMI270 + BMM150)
 *
 * Each segment runs a Mahony filter and reports its orientation as a unit
 * quaternion. The knee angle is derived on the PC from the RELATIVE rotation of
 * the two segments, so arbitrary sensor placement is tolerated. See
 * knee_collector_uart.py.
 *
 * Magnetometer (9-DOF) is CONFIGURED AT RUNTIME -- no reflashing to calibrate or
 * to switch 6-DOF/9-DOF. The PC pushes each board's hard-/soft-iron + axis map
 * over USB at startup; the master applies its own and relays the slave's over
 * UART. Until a config with use_mag=1 arrives, the board runs 6-DOF.
 *
 * PC->master USB commands (ASCII, newline-terminated):
 *   PING                              -> "PONG"
 *   CFG,<M|S>,<um>,bx,by,bz,sx,sy,sz,p0,p1,p2,s0,s1,s2  -> "OK,<M|S>" / "ERR,S"
 *   STREAM,<M|S>,<0|1>                -> stream raw accel+mag (RM.../RS...) on/off
 * Default (no stream): the master emits the D data line every cycle.
 *
 * PC data line (12 fields):
 *   D,<t_thigh_us>,<tw>,<tx>,<ty>,<tz>,<t_shank_mid_us>,<sw>,<sx>,<sy>,<sz>,<rtt_us>
 *
 * UART master<->slave (binary):
 *   'R' -> 18-byte quaternion packet [0xAA, q0..q3, xor]
 *   'r' -> 26-byte raw packet        [0xBB, ax,ay,az,mx,my,mz, xor]
 *   'W' + 32-byte config payload     -> slave applies, replies 0x06 (ACK)/0x15
 *
 * Wiring: Master TX(D1)->Slave RX(D0), Master RX(D0)<-Slave TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

// ---- Mahony filter state --------------------------------------------------
const float TWO_KP = 2.0f * 0.5f;
const float TWO_KI = 2.0f * 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long lastMicros = 0;

// ---- runtime magnetometer config (set by CFG; 6-DOF until then) -----------
bool  useMagCfg   = false;
float MAG_BIAS[3]  = {0.0f, 0.0f, 0.0f};
float MAG_SCALE[3] = {1.0f, 1.0f, 1.0f};
int   MAG_PERM[3]  = {0, 1, 2};
int   MAG_SIGN[3]  = {1, 1, 1};
float magX = 0.0f, magY = 0.0f, magZ = 0.0f;   // latest calibrated mag (0 = none)

// Rotate the raw mag into the IMU frame using the learned axis map.
inline void magToImuFrame(float &mx, float &my, float &mz) {
  float in[3] = {mx, my, mz};
  mx = MAG_SIGN[0] * in[MAG_PERM[0]];
  my = MAG_SIGN[1] * in[MAG_PERM[1]];
  mz = MAG_SIGN[2] * in[MAG_PERM[2]];
}

inline void calibrateMag(float &mx, float &my, float &mz) {
  mx = (mx - MAG_BIAS[0]) * MAG_SCALE[0];   // hard-/soft-iron in raw frame...
  my = (my - MAG_BIAS[1]) * MAG_SCALE[1];
  mz = (mz - MAG_BIAS[2]) * MAG_SCALE[2];
  magToImuFrame(mx, my, mz);                // ...then remap into the IMU frame.
}

// ---- serial modes ---------------------------------------------------------
enum Mode { NORMAL, STREAM_M, STREAM_S };
Mode mode = NORMAL;
char cmdBuf[96];
int  cmdLen = 0;

const unsigned long SLAVE_TIMEOUT_US = 8000;

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

// ---- slave I/O ------------------------------------------------------------
unsigned long pollSlaveQuat(float q[4], unsigned long &rtt) {
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

bool pollSlaveRaw(float a[3], float m[3]) {
  while (Serial1.available()) Serial1.read();
  unsigned long tReq = micros();
  Serial1.write('r');
  uint8_t pkt[26];
  int idx = 0;
  while (idx < 26 && (micros() - tReq) < SLAVE_TIMEOUT_US) {
    if (Serial1.available()) pkt[idx++] = Serial1.read();
  }
  if (idx < 26 || pkt[0] != 0xBB) return false;
  uint8_t cs = 0;
  for (int i = 1; i <= 24; i++) cs ^= pkt[i];
  if (cs != pkt[25]) return false;
  memcpy(a, &pkt[1], 12);
  memcpy(m, &pkt[13], 12);
  return true;
}

bool sendConfigToSlave(int um, float *b, float *sc, int *pm, int *sg) {
  uint8_t pl[32];
  pl[0] = um ? 1 : 0;
  memcpy(&pl[1],  b,  12);
  memcpy(&pl[13], sc, 12);
  pl[25] = (uint8_t)pm[0]; pl[26] = (uint8_t)pm[1]; pl[27] = (uint8_t)pm[2];
  pl[28] = (sg[0] < 0) ? 1 : 0;
  pl[29] = (sg[1] < 0) ? 1 : 0;
  pl[30] = (sg[2] < 0) ? 1 : 0;
  uint8_t cs = 0;
  for (int i = 0; i < 31; i++) cs ^= pl[i];
  pl[31] = cs;

  for (int attempt = 0; attempt < 3; attempt++) {
    // Quiet-flush: drain the link until it's been silent for ~3 ms, so an
    // in-flight reply to a prior 'R' poll can't be mistaken for the ACK.
    unsigned long quiet = micros();
    while (micros() - quiet < 3000) {
      if (Serial1.available()) { Serial1.read(); quiet = micros(); }
    }
    Serial1.write('W');
    Serial1.write(pl, 32);
    // Wait specifically for ACK (0x06) or NAK (0x15); ignore stray bytes.
    unsigned long t0 = micros();
    while (micros() - t0 < 200000) {
      if (Serial1.available()) {
        uint8_t r = Serial1.read();
        if (r == 0x06) return true;
        if (r == 0x15) break;          // NAK -> retry
      }
    }
  }
  return false;
}

// ---- USB command handling -------------------------------------------------
void applyConfigMaster(int um, float *b, float *sc, int *pm, int *sg) {
  useMagCfg = (um != 0);
  for (int i = 0; i < 3; i++) {
    MAG_BIAS[i] = b[i]; MAG_SCALE[i] = sc[i];
    MAG_PERM[i] = pm[i]; MAG_SIGN[i] = sg[i];
  }
  if (!useMagCfg) { magX = magY = magZ = 0.0f; }
}

void processCommand(char *s) {
  if (!strcmp(s, "PING")) { Serial.println("PONG"); return; }

  if (!strncmp(s, "STREAM,", 7)) {
    char who = s[7];
    bool on = (s[9] == '1');
    mode = on ? (who == 'M' ? STREAM_M : STREAM_S) : NORMAL;
    return;
  }

  if (!strncmp(s, "CFG,", 4)) {
    strtok(s, ",");                       // "CFG"
    char *w = strtok(NULL, ",");
    char who = w ? w[0] : '?';
    int um = atoi(strtok(NULL, ","));
    float b[3], sc[3];
    int pm[3], sg[3];
    for (int i = 0; i < 3; i++) b[i]  = atof(strtok(NULL, ","));
    for (int i = 0; i < 3; i++) sc[i] = atof(strtok(NULL, ","));
    for (int i = 0; i < 3; i++) pm[i] = atoi(strtok(NULL, ","));
    for (int i = 0; i < 3; i++) sg[i] = atoi(strtok(NULL, ","));
    if (who == 'M') {
      applyConfigMaster(um, b, sc, pm, sg);
      Serial.println("OK,M");
    } else if (who == 'S') {
      Serial.println(sendConfigToSlave(um, b, sc, pm, sg) ? "OK,S" : "ERR,S");
    }
    return;
  }
}

void handleUsbCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = 0; processCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

void printVec6(const char *tag, float a0, float a1, float a2,
               float m0, float m1, float m2) {
  Serial.print(tag);
  Serial.print(a0, 4); Serial.print(',');
  Serial.print(a1, 4); Serial.print(',');
  Serial.print(a2, 4); Serial.print(',');
  Serial.print(m0, 4); Serial.print(',');
  Serial.print(m1, 4); Serial.print(',');
  Serial.println(m2, 4);
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

void loop() {
  handleUsbCommands();

  if (mode == STREAM_M) {                 // raw self accel+mag for calibration
    // Gate on the SLOW magnetometer and read accel at the same instant, so the
    // pair is simultaneous. Gating on accel instead pairs fresh accel with STALE
    // mag, which breaks the accel.mag invariant the axis solver relies on.
    if (IMU.magneticFieldAvailable()) {
      float ax, ay, az, mx, my, mz;
      IMU.readMagneticField(mx, my, mz);
      IMU.readAcceleration(ax, ay, az);
      printVec6("RM,", ax, ay, az, mx, my, mz);
    }
    return;
  }
  if (mode == STREAM_S) {                 // raw slave accel+mag (relayed)
    float a[3], m[3];
    if (pollSlaveRaw(a, m)) printVec6("RS,", a[0], a[1], a[2], m[0], m[1], m[2]);
    return;
  }

  // NORMAL: fuse + emit the D data line.
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);        // deg/s

    if (useMagCfg && IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz);  // uT
      calibrateMag(mx, my, mz);
      magX = mx; magY = my; magZ = mz;
    }

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                 ax, ay, az, magX, magY, magZ, dt);

    float sq[4] = {0, 0, 0, 0};
    unsigned long rtt = 0;
    unsigned long sMid = pollSlaveQuat(sq, rtt);

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
