/*
 * KNEE ANGLE - MASTER (thigh)  [UART, raw-binary reply]
 * Board: Arduino Nano 33 BLE Rev2
 *
 * Reads a FIXED 10-byte binary packet from the slave (header 0xAA, pitch, roll,
 * checksum). Reading a known-size packet is fast and unambiguous, versus parsing
 * a variable text line. Brackets the slave sample at (T_req+T_resp)/2.
 *
 * PC line (text, for the collector):
 *   D,<t_thigh_us>,<thigh_pitch>,<thigh_roll>,<t_shank_mid_us>,<shank_pitch>,<shank_roll>,<rtt_us>
 * On a bad/missing reply, shank fields and midpoint are 0.
 *
 * Wiring: Master TX(D1)->Slave RX(D0), Master RX(D0)<-Slave TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

const float ALPHA = 0.98f;
float pitch = 0.0f, roll = 0.0f;
unsigned long lastMicros = 0;

const unsigned long SLAVE_TIMEOUT_US = 3000;  // plenty for a 10-byte reply

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial) { ; }

  if (!IMU.begin()) {
    Serial.println("ERR,IMU init failed");
    while (1) { ; }
  }
  Serial.println("# MASTER cols: D,t_thigh_us,thigh_pitch,thigh_roll,t_shank_mid_us,shank_pitch,shank_roll,rtt_us");

  lastMicros = micros();
  unsigned long t0 = millis();
  while (!IMU.accelerationAvailable() && millis() - t0 < 2000) { ; }
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    roll  = atan2(ay, az) * 180.0f / PI;
  }
}

// Returns midpoint timestamp on success (fills sp/sr/rtt), or 0 on failure.
unsigned long pollSlave(float &sp, float &sr, unsigned long &rtt) {
  while (Serial1.available()) Serial1.read();   // flush stale

  unsigned long tReq = micros();
  Serial1.write('R');

  // Read exactly 10 bytes with timeout.
  uint8_t pkt[10];
  int idx = 0;
  while (idx < 10 && (micros() - tReq) < SLAVE_TIMEOUT_US) {
    if (Serial1.available()) pkt[idx++] = Serial1.read();
  }
  unsigned long tResp = micros();
  if (idx < 10) return 0;             // timeout / incomplete
  if (pkt[0] != 0xAA) return 0;       // bad header

  uint8_t cs = 0;
  for (int i = 1; i <= 8; i++) cs ^= pkt[i];
  if (cs != pkt[9]) return 0;         // checksum fail

  memcpy(&sp, &pkt[1], 4);
  memcpy(&sr, &pkt[5], 4);
  rtt = tResp - tReq;
  return (tReq + tResp) / 2;
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f / 104.0f;

    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    float rollAcc  = atan2(ay, az) * 180.0f / PI;
    pitch += gy * dt;
    roll  += gx * dt;
    pitch = ALPHA * pitch + (1.0f - ALPHA) * pitchAcc;
    roll  = ALPHA * roll  + (1.0f - ALPHA) * rollAcc;

    float sp = 0, sr = 0;
    unsigned long rtt = 0;
    unsigned long sMid = pollSlave(sp, sr, rtt);

    Serial.print("D,");
    Serial.print(now);      Serial.print(',');
    Serial.print(pitch, 2); Serial.print(',');
    Serial.print(roll, 2);  Serial.print(',');
    Serial.print(sMid);     Serial.print(',');
    Serial.print(sp, 2);    Serial.print(',');
    Serial.print(sr, 2);    Serial.print(',');
    Serial.println(rtt);
  }
}
