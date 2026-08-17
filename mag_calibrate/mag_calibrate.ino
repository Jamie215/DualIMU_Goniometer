/*
 * MAGNETOMETER CALIBRATION + AXIS ALIGNMENT  (Nano 33 BLE Rev2, onboard BMM150)
 *
 * Two problems this sketch solves, from a single ~40 s tumble:
 *
 * 1. HARD-/SOFT-IRON. Raw mag has a constant offset (hard iron) and scale/skew
 *    (soft iron). Estimated with the min/max bounding-box method -> MAG_BIAS,
 *    MAG_SCALE.
 *
 * 2. AXIS ALIGNMENT. On this board the BMM150 (mag) and BMI270 (accel/gyro) do
 *    NOT share axes, so the mag must be rotated into the IMU frame before fusion.
 *    The two frames differ only by a 90-degree axis swap/flip, so there are just
 *    24 candidate rotations. We exploit a physical invariant: the angle between
 *    gravity and the Earth's field is fixed, so accel_hat . mag_hat is CONSTANT
 *    across orientations -- but only when both are in the same frame. We tumble,
 *    then pick the candidate that holds that dot product most constant (lowest
 *    variance). That uniquely recovers the mapping, printed as a ready-to-paste
 *    magToImuFrame() body.
 *
 * Each board has its own mag and mounting -> run this on BOTH boards and paste
 * each board's output into its own sketch (master_imu.ino / slave_imu.ino).
 *
 * HOW TO RUN:
 *   1. Flash to ONE board. Open Serial Monitor @ 115200.
 *   2. On "GO", SLOWLY tumble the board through every orientation for ~40 s
 *      (slow so the accelerometer sees gravity, not hand acceleration; cover all
 *      faces up and down). Paste the printed block into that board's sketch.
 *   3. Repeat on the other board.
 */

#include "Arduino_BMI270_BMM150.h"

const unsigned long CAL_MS = 40000;
const int MAXS = 800;                 // stored (accel,mag) pairs for the solve
const unsigned long STORE_GAP_US = 30000;  // >= 30 ms between stored samples

float Ax[MAXS], Ay[MAXS], Az[MAXS];
float Mx[MAXS], My[MAXS], Mz[MAXS];
int   nS = 0;

// The 6 axis orderings and their permutation parity (sign of the permutation).
const int PERM[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
const int PAR[6]     = {  +1,     -1,     -1,     +1,     +1,     -1  };

static inline float pick(const float v[3], int idx) { return v[idx]; }

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  if (!IMU.begin()) {
    Serial.println("IMU init failed");
    while (1) { ; }
  }

  Serial.println("Mag calibration + axis alignment.");
  Serial.println("On GO, SLOWLY tumble through ALL orientations for ~40 s.");
  delay(2000);
  Serial.println("GO");

  float mnx = 1e9, mny = 1e9, mnz = 1e9;
  float mxx = -1e9, mxy = -1e9, mxz = -1e9;
  unsigned long t0 = millis(), lastPrint = 0, lastStore = micros();

  while (millis() - t0 < CAL_MS) {
    if (IMU.magneticFieldAvailable() && IMU.accelerationAvailable()) {
      float mx, my, mz, ax, ay, az;
      IMU.readMagneticField(mx, my, mz);
      IMU.readAcceleration(ax, ay, az);

      if (mx < mnx) mnx = mx;  if (mx > mxx) mxx = mx;
      if (my < mny) mny = my;  if (my > mxy) mxy = my;
      if (mz < mnz) mnz = mz;  if (mz > mxz) mxz = mz;

      if (nS < MAXS && (micros() - lastStore) >= STORE_GAP_US) {
        Ax[nS] = ax; Ay[nS] = ay; Az[nS] = az;
        Mx[nS] = mx; My[nS] = my; Mz[nS] = mz;
        nS++;
        lastStore = micros();
      }
    }
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.print((CAL_MS - (millis() - t0)) / 1000);
      Serial.print(" s left, samples="); Serial.println(nS);
    }
  }

  // ---- hard-/soft-iron ----
  float bx = (mxx + mnx) * 0.5f, by = (mxy + mny) * 0.5f, bz = (mxz + mnz) * 0.5f;
  float cx = (mxx - mnx) * 0.5f, cy = (mxy - mny) * 0.5f, cz = (mxz - mnz) * 0.5f;
  if (cx < 1.0f || cy < 1.0f || cz < 1.0f || nS < 100) {
    Serial.println("WARNING: not enough rotation/samples. Re-run, cover all faces.");
    return;
  }
  float avg = (cx + cy + cz) / 3.0f;
  float sx = avg / cx, sy = avg / cy, sz = avg / cz;
  const float BIAS[3]  = {bx, by, bz};
  const float SCALE[3] = {sx, sy, sz};

  // ---- axis alignment: search the 24 proper signed permutations ----
  // For each candidate, compute variance of d = a_hat . (P . m_cal_hat) over all
  // samples; the true mapping minimizes it. Calibration is applied in the RAW
  // frame first (matching calibrateMag in the main sketches), then P.
  double bestVar = 1e18, nextVar = 1e18;
  int bestP[3] = {0,1,2}, bestSgn[3] = {1,1,1};

  for (int pi = 0; pi < 6; pi++) {
    for (int smask = 0; smask < 8; smask++) {
      int s0 = (smask & 1) ? -1 : 1;
      int s1 = (smask & 2) ? -1 : 1;
      int s2 = (smask & 4) ? -1 : 1;
      if (PAR[pi] * s0 * s1 * s2 != 1) continue;   // proper rotations only (det +1)

      double sum = 0.0, sumsq = 0.0;
      int cnt = 0;
      for (int i = 0; i < nS; i++) {
        float an = sqrt(Ax[i]*Ax[i] + Ay[i]*Ay[i] + Az[i]*Az[i]);
        if (an < 1e-6f) continue;
        float mc[3] = { (Mx[i]-BIAS[0])*SCALE[0],
                        (My[i]-BIAS[1])*SCALE[1],
                        (Mz[i]-BIAS[2])*SCALE[2] };
        float mn = sqrt(mc[0]*mc[0] + mc[1]*mc[1] + mc[2]*mc[2]);
        if (mn < 1e-6f) continue;
        float pmx = s0 * pick(mc, PERM[pi][0]);
        float pmy = s1 * pick(mc, PERM[pi][1]);
        float pmz = s2 * pick(mc, PERM[pi][2]);
        float d = (Ax[i]*pmx + Ay[i]*pmy + Az[i]*pmz) / (an * mn);
        sum += d; sumsq += (double)d * d; cnt++;
      }
      if (cnt < 50) continue;
      double var = (sumsq - sum*sum/cnt) / cnt;
      if (var < bestVar) {
        nextVar = bestVar; bestVar = var;
        bestP[0]=PERM[pi][0]; bestP[1]=PERM[pi][1]; bestP[2]=PERM[pi][2];
        bestSgn[0]=s0; bestSgn[1]=s1; bestSgn[2]=s2;
      } else if (var < nextVar) {
        nextVar = var;
      }
    }
  }

  // ---- report ----
  Serial.println("\n============ paste into THIS board's sketch ============");
  Serial.print("const float MAG_BIAS[3]  = {");
  Serial.print(bx, 3); Serial.print("f, ");
  Serial.print(by, 3); Serial.print("f, ");
  Serial.print(bz, 3); Serial.println("f};");
  Serial.print("const float MAG_SCALE[3] = {");
  Serial.print(sx, 4); Serial.print("f, ");
  Serial.print(sy, 4); Serial.print("f, ");
  Serial.print(sz, 4); Serial.println("f};");

  Serial.println("inline void magToImuFrame(float &mx, float &my, float &mz) {");
  Serial.println("  float in0 = mx, in1 = my, in2 = mz;");
  printAxis("mx", bestSgn[0], bestP[0]);
  printAxis("my", bestSgn[1], bestP[1]);
  printAxis("mz", bestSgn[2], bestP[2]);
  Serial.println("}");
  Serial.println("========================================================");

  Serial.print("axis-fit std: best="); Serial.print(sqrt(bestVar), 4);
  Serial.print("  next-best="); Serial.print(sqrt(nextVar), 4);
  Serial.print("  (separation ratio "); Serial.print(sqrt(nextVar/bestVar), 2);
  Serial.println("x)");
  if (sqrt(nextVar / bestVar) < 1.3) {
    Serial.println("WARNING: weak separation -> tumble was too fast or incomplete.");
    Serial.println("Re-run slowly, covering every face up AND down.");
  }
}

void printAxis(const char *out, int sgn, int idx) {
  Serial.print("  "); Serial.print(out); Serial.print(" = ");
  Serial.print(sgn < 0 ? "-in" : "in");
  Serial.print(idx); Serial.println(";");
}

void loop() { }
