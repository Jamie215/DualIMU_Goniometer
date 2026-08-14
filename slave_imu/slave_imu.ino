/*
 * KNEE ANGLE - SLAVE (shank)  [UART, raw-binary reply for low latency]
 * Board: Arduino Nano 33 BLE Rev2
 *
 * Change vs previous: the reply is now a FIXED 10-BYTE BINARY packet instead of
 * formatted text. Text (Serial1.print of floats) was slow to generate and send,
 * pushing round-trip time near the master's 5 ms timeout and causing dropped
 * cycles. Raw bytes are small and fast -> round-trip should drop well under 1 ms.
 *
 * Reply packet (10 bytes):
 *   [0] 0xAA        header/sync byte
 *   [1..4] float pitch  (little-endian)
 *   [5..8] float roll   (little-endian)
 *   [9] checksum    XOR of bytes [1..8]
 *
 * Wiring: Slave TX(D1)->Master RX(D0), Slave RX(D0)<-Master TX(D1), GND<->GND.
 */

#include "Arduino_BMI270_BMM150.h"

const float ALPHA = 0.98f;
float pitch = 0.0f, roll = 0.0f;   // cached latest orientation
unsigned long lastMicros = 0;

void setup() {
  Serial1.begin(115200);

  if (!IMU.begin()) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) { digitalWrite(LED_BUILTIN, HIGH); delay(150);
                digitalWrite(LED_BUILTIN, LOW);  delay(150); }
  }

  lastMicros = micros();

  // Seed from accelerometer.
  unsigned long t0 = millis();
  while (!IMU.accelerationAvailable() && millis() - t0 < 2000) { ; }
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    roll  = atan2(ay, az) * 180.0f / PI;
  }
}

inline void sendPacket() {
  uint8_t pkt[10];
  pkt[0] = 0xAA;
  memcpy(&pkt[1], &pitch, 4);
  memcpy(&pkt[5], &roll, 4);
  uint8_t cs = 0;
  for (int i = 1; i <= 8; i++) cs ^= pkt[i];
  pkt[9] = cs;
  Serial1.write(pkt, 10);
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
    IMU.readGyroscope(gx, gy, gz);
    serviceRequest();

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
  }

  serviceRequest();
}
