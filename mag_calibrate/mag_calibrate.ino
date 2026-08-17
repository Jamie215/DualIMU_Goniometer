/*
 * MAGNETOMETER CALIBRATION  (Arduino Nano 33 BLE Rev2, onboard BMM150)
 *
 * The raw magnetometer is useless for heading until it's calibrated: nearby
 * ferrous metal and the board itself add a constant HARD-IRON offset, and
 * distortions add a SOFT-IRON scale/skew. This sketch estimates both with the
 * standard min/max (bounding-box) method and prints ready-to-paste constants.
 *
 * Each board has its OWN magnetometer and its own mounting, so calibrate BOTH
 * boards separately and paste each board's numbers into its own sketch
 * (master_imu.ino for the thigh board, slave_imu.ino for the shank board).
 *
 * HOW TO RUN:
 *   1. Flash this to ONE board. Open Serial Monitor @ 115200.
 *   2. When it prints GO, slowly rotate the board through EVERY orientation
 *      (figure-8s, and point each face up/down) for ~30 s. The goal is to trace
 *      out a full sphere of field directions.
 *   3. Copy the printed MAG_BIAS / MAG_SCALE lines into that board's sketch.
 *   4. Repeat for the other board.
 *
 * AXIS-ALIGNMENT CHECK: this sketch also prints raw accel and mag together while
 * you hold still, so you can verify the mag axes match the IMU axes. If, when you
 * tilt the board so gravity is along +X, the magnetic field's dominant change is
 * on a DIFFERENT mag axis, you need an axis remap (see magToImuFrame() in the
 * main sketches).
 */

#include "Arduino_BMI270_BMM150.h"

const unsigned long CAL_MS = 30000;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!IMU.begin()) {
    Serial.println("IMU init failed");
    while (1) { ; }
  }

  // --- brief axis-alignment aid: hold the board still in a few poses ---------
  Serial.println("Hold the board still in a couple of orientations; compare");
  Serial.println("which accel axis carries gravity vs which mag axis moves most.");
  for (int i = 0; i < 15; i++) {
    float ax = 0, ay = 0, az = 0, mx = 0, my = 0, mz = 0;
    if (IMU.accelerationAvailable())  IMU.readAcceleration(ax, ay, az);
    if (IMU.magneticFieldAvailable()) IMU.readMagneticField(mx, my, mz);
    Serial.print("acc["); Serial.print(ax, 2); Serial.print(',');
    Serial.print(ay, 2); Serial.print(','); Serial.print(az, 2);
    Serial.print("]  mag["); Serial.print(mx, 1); Serial.print(',');
    Serial.print(my, 1); Serial.print(','); Serial.print(mz, 1);
    Serial.println("]");
    delay(400);
  }

  Serial.println("\nCalibration: rotate through ALL orientations until done.");
  delay(1500);
  Serial.println("GO");

  float mnx = 1e9, mny = 1e9, mnz = 1e9;
  float mxx = -1e9, mxy = -1e9, mxz = -1e9;
  unsigned long t0 = millis(), lastPrint = 0;
  while (millis() - t0 < CAL_MS) {
    if (IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz);
      if (mx < mnx) mnx = mx;  if (mx > mxx) mxx = mx;
      if (my < mny) mny = my;  if (my > mxy) mxy = my;
      if (mz < mnz) mnz = mz;  if (mz > mxz) mxz = mz;
    }
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.print((CAL_MS - (millis() - t0)) / 1000);
      Serial.println(" s left...");
    }
  }

  float bx = (mxx + mnx) * 0.5f, by = (mxy + mny) * 0.5f, bz = (mxz + mnz) * 0.5f;
  float cx = (mxx - mnx) * 0.5f, cy = (mxy - mny) * 0.5f, cz = (mxz - mnz) * 0.5f;

  if (cx < 1.0f || cy < 1.0f || cz < 1.0f) {
    Serial.println("WARNING: one axis barely moved -> not enough rotation.");
    Serial.println("Re-run and make sure every face points up and down.");
    return;
  }

  float avg = (cx + cy + cz) / 3.0f;
  float sx = avg / cx, sy = avg / cy, sz = avg / cz;

  Serial.println("\n---- paste into THIS board's sketch ----");
  Serial.print("const float MAG_BIAS[3]  = {");
  Serial.print(bx, 3); Serial.print("f, ");
  Serial.print(by, 3); Serial.print("f, ");
  Serial.print(bz, 3); Serial.println("f};");
  Serial.print("const float MAG_SCALE[3] = {");
  Serial.print(sx, 4); Serial.print("f, ");
  Serial.print(sy, 4); Serial.print("f, ");
  Serial.print(sz, 4); Serial.println("f};");
  Serial.println("----------------------------------------");
}

void loop() { }
