# Literature Review: IMU-Based Joint Angle / ROM Measurement

A reading list for the dual-IMU knee goniometer. It places the project within
the published work and maps papers onto the four research directions in
[`areas_to_explore.md`](areas_to_explore.md):

| Area | Topic | Section below |
|---|---|---|
| — | Where this project sits (closest prior work) | §1 |
| — | Reviews to read first | §2 |
| 1 | Magnetometer fusion; magnetometer-free methods; absolute vs. relative | §3 |
| 2 | Absolute (anatomical) angle via functional / sensor-to-segment calibration | §4 |
| 3 | Bluetooth (BLE) links | §5 |
| 4 | Multiple nodes / joints, time synchronization | §6 |
| — | Validation methodology (what "accurate" means clinically) | §7 |

Links go to the DOI, the publisher page, or PubMed. Each citation (title,
authors, venue) was checked against an index listing in Sept 2026. Numbers
quoted are the headline results the authors report. Read the paper before
you rely on one.

---

## 1. Where this project sits: closest prior work

This project runs 6-DOF (accel + gyro) fusion on each segment. It measures
each segment's tilt against gravity in the sagittal plane, zeroes it on a
static hold, and takes the knee angle as the difference in segment tilt. That
design has a long history:

- **Williamson R, Andrews BJ (2001).** *Detecting absolute human knee angle and
  angular velocity using accelerometers and rate gyroscopes.* Med Biol Eng
  Comput 39:294–302. [doi:10.1007/BF02345283](https://doi.org/10.1007/BF02345283)
  This is the closest ancestor. One gyro + accelerometer module sits on the
  thigh and one on the shank. Each segment's tilt comes from fusing the gyro
  and accelerometer, and the knee angle is the **difference of segment
  tilts**, which is the same idea as `sagittal_inclination()`. The paper also
  auto-nulls gyro offset. Mean difference from a reference goniometer was
  about 2.1–2.4° during sit-to-stand, including in an FES-assisted paraplegic
  user.
- **Dejnabadi H, Jolles BM, Aminian K (2005).** *A new approach to accurate
  measurement of uniaxial joint angles based on a combination of accelerometers
  and gyroscopes.* IEEE Trans Biomed Eng 52(8):1478–84.
  [PubMed 16119244](https://pubmed.ncbi.nlm.nih.gov/16119244/)
  Uses one module per segment and "virtual sensors" placed at the joint
  centre. It needs no integration, so it does not drift. Knee flexion in gait
  had an RMS error of about 1.3° against optical motion capture. The model is
  personalized per subject, which foreshadows Area 2.
- **Cooper G, Sheret I, McMillan L, et al. (2009).** *Inertial sensor-based
  knee flexion/extension angle estimation.* J Biomech 42(16):2678–85.
  [PubMed 19782986](https://pubmed.ncbi.nlm.nih.gov/19782986/)
  Combines Kalman filters with anatomical constraints and deliberately
  **leaves out the magnetometer** because of indoor field distortion (the
  same reasoning as Area 1). Error was 0.7° for slow walking and 3.4° for
  running. Fast motion is where this project's README expects lag or
  overshoot.
- **Seel T, Raisch J, Schauer T (2014).** *IMU-Based Joint Angle Measurement
  for Gait Analysis.* Sensors 14(4):6891–6909.
  [doi:10.3390/s140406891](https://doi.org/10.3390/s140406891)
  The most-cited modern paper on hinge-joint (knee) angles. It (a)
  **identifies the joint axis automatically** from arbitrary motion using
  gyro/accel kinematic constraints, and (b) fuses a gyro-integrated angle
  with an accelerometer-based angle. This is the reference implementation
  for Area 2.
- **Song SY, Pei Y, Hsiao-Wecksler ET (2022).** *Estimating Relative Angles
  Using Two Inertial Measurement Units Without Magnetometers.* IEEE Sensors J
  22(20):19688–99. [IEEE Xplore](https://ieeexplore.ieee.org/document/9888780/) ·
  [open code](https://github.com/ssong47/compute_relative_angle_between_two_IMUs)
  A low-cost two-IMU system using 6-axis MPU6050s. It benchmarks seven
  filters, including **Mahony** and Madgwick, for the relative angle, and the
  code is open. This is the most directly comparable hobby-grade study; use
  it to justify the Mahony choice or to tune `TWO_KP`.
- **Buranapuntalug S, Chaitrakul N, Liamtragoolpanich P, et al. (2026).**
  *Development and measurement of elbow and knee joints using an
  electro-goniometer in healthy subjects: A preliminary study.* SICOT-J 12:23.
  [doi:10.1051/sicotj/2026016](https://doi.org/10.1051/sicotj/2026016) ·
  [PubMed 42085583](https://pubmed.ncbi.nlm.nih.gov/42085583/)
  Uses **the same hardware family: two Arduino Nano 33 BLE boards** across
  the joint. Roll and pitch go over BLE to a phone app, and the joint angle is
  the orientation difference after an initial calibration. This is a very
  close comparator for Area 3 and for a validation-study design.

Filter background:

- **Mahony R, Hamel T, Pflimlin J-M (2008).** *Nonlinear complementary filters
  on the special orthogonal group.* IEEE Trans Autom Control 53(5):1203–17.
  [doi:10.1109/TAC.2008.923738](https://doi.org/10.1109/TAC.2008.923738)
  The filter that runs on both boards.
- **Madgwick SOH, Harrison AJL, Vaidyanathan R (2011).** *Estimation of IMU and
  MARG orientation using a gradient descent algorithm.* IEEE ICORR.
  [doi:10.1109/ICORR.2011.5975346](https://doi.org/10.1109/ICORR.2011.5975346)
- **Kok M, Hol JD, Schön TB (2017).** *Using Inertial Sensors for Position and
  Orientation Estimation.* Found Trends Signal Process 11(1–2):1–153.
  [arXiv:1704.06053](https://arxiv.org/abs/1704.06053)
  A free tutorial of about 150 pages. It is the best single reference for the
  maths behind complementary and Kalman filters, gyro bias, and drift.

---

## 2. Reviews to read first

- **Weygers I, Kok M, Konings M, et al. (2020).** *Inertial Sensor-Based Lower
  Limb Joint Kinematics: A Methodological Systematic Review.* Sensors
  20(3):673. [doi:10.3390/s20030673](https://doi.org/10.3390/s20030673)
  Its conclusion is that methods are "inherently application dependent" and
  that sensor limitations get compensated with biomechanical assumptions. It
  is the best map of the design space this project is navigating.
- **Picerno P (2017).** *25 years of lower limb joint kinematics by using
  inertial and magnetic sensors: A review of methodological approaches.* Gait
  Posture 51:239–246. [PubMed 27833057](https://pubmed.ncbi.nlm.nih.gov/27833057/)
- **Poitras I, Dupuis F, Bielmann M, et al. (2019).** *Validity and Reliability
  of Wearable Sensors for Joint Angle Estimation: A Systematic Review.*
  Sensors 19(7):1555. [doi:10.3390/s19071555](https://doi.org/10.3390/s19071555)
  Reports validity and reliability per joint and per task complexity. Use it
  to set accuracy targets for the knee.
- **Kobsar D, et al. (2020).** *Validity and reliability of wearable inertial
  sensors in healthy adult walking: a systematic review and meta-analysis.*
  J NeuroEng Rehabil. [PMC7216606](https://pmc.ncbi.nlm.nih.gov/articles/PMC7216606/)
- **Vitali RV, Perkins NC (2020).** *Determining anatomical frames via inertial
  motion capture: A survey of methods.* J Biomech 106:109832.
  [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0021929020302554)
  **Essential for Area 2.** It surveys 112 studies and sorts sensor-to-bone
  alignment into four families: *assumed alignment, functional alignment,
  model-based, augmented data*. The current zero-hold plus sweep approach is
  a light form of functional alignment. The authors call for a standard way
  to define anatomical axes.
- **Pacher L, Chatellier C, Vauzelle R, Fradet L (2020).** *Sensor-to-Segment
  Calibration Methodologies for Lower-Body Kinematic Analysis with Inertial
  Sensors: A Systematic Review.* Sensors 20(11):3322.
  [doi:10.3390/s20113322](https://doi.org/10.3390/s20113322)
- *Inertial Human Motion Capture: From Biomechanics to Recent Sensor Fusion
  Methods and Back* (2026, arXiv preprint).
  [arXiv:2607.16000](https://arxiv.org/abs/2607.16000)
  A very recent survey that spans biomechanics and sensor fusion. Only the
  title and venue were checked, so skim the abstract first.

---

## 3. Area 1: Magnetometer fusion, and absolute vs. relative angle

The literature supports the conclusion in Area 1. Indoor magnetic fields are
distorted enough to hurt accuracy. The research community has moved toward
**magnetometer-free** joint tracking that recovers relative heading from
*kinematic constraints* instead of from the magnetometer.

**How bad indoor fields are:**

- **de Vries WHK, Veeger HEJ, Baten CTM, van der Helm FCT (2009).** *Magnetic
  distortion in motion labs, implications for validating inertial magnetic
  sensors.* Gait Posture 29(4):535–41.
  [PubMed 19150239](https://pubmed.ncbi.nlm.nih.gov/19150239/)
  Heading SD was **about 29° at 5 cm above the floor**, falling to about 3°
  above 100 cm, in an ordinary motion lab. This matters directly for a
  **shank** sensor, which sits low, so it supports the "do not add" decision.
  Their method (mapping the field over the lab volume) is also a template for
  experiment 1.8(1), the field-quality gate.
- **Fan B, Li Q, Liu T (2018).** *How Magnetic Disturbance Influences the
  Attitude and Heading in Magnetic and Inertial Sensor-Based Orientation
  Estimation.* Sensors 18(1):76.
  [doi:10.3390/s18010076](https://doi.org/10.3390/s18010076)
  Explains how a disturbed field affects the heading estimate, and whether it
  also contaminates the attitude (tilt) estimate. That second question is the
  one that matters for a gravity-referenced sagittal angle.

**Magnetometer-free joint angles, using kinematic constraints in place of the magnetometer:**

- **Laidig D, Schauer T, Seel T (2017).** *Exploiting kinematic constraints to
  compensate magnetic disturbances when calculating joint angles of
  approximate hinge joints from orientation estimates of inertial sensors.*
  IEEE ICORR, 971–976. [PubMed 28813947](https://pubmed.ncbi.nlm.nih.gov/28813947/)
  For hinge joints like the knee, the hinge constraint corrects the relative
  heading error. In large disturbances, joint-angle RMSE fell from
  **25.8° to 2.6°**. This is the principled way to get out-of-plane
  information without trusting the field.
- **Lee JK, Jeon TH (2019).** *Magnetic Condition-Independent 3D Joint Angle
  Estimation Using Inertial Sensors and Kinematic Constraints.* Sensors
  19(24):5522. [doi:10.3390/s19245522](https://doi.org/10.3390/s19245522)
  An attitude Kalman filter is followed by a heading Kalman filter whose
  correction comes from a joint constraint, not the magnetometer. RMSE was
  1.58°, compared with 5.38° for a conventional filter with disturbance
  compensation.
- **Teufl W, Miezal M, Taetz B, Fröhlich M, Bleser G (2018).** *Validity,
  Test-Retest Reliability and Long-Term Stability of Magnetometer Free
  Inertial Sensor Based 3D Joint Kinematics.* Sensors 18(7):1980.
  [doi:10.3390/s18071980](https://doi.org/10.3390/s18071980)
  Shows that full 3-D lower-limb kinematics are clinically usable **without a
  magnetometer**. This is the strongest evidence that, even if a 3-D
  requirement arrives, the magnetometer still is not necessary.
- **Laidig D, Weygers I, Seel T (2022).** *Self-Calibrating Magnetometer-Free
  Inertial Motion Tracking of 2-DoF Joints.* Sensors 22(24):9850.
  [doi:10.3390/s22249850](https://doi.org/10.3390/s22249850)
  Identifies the joint axes and the heading offset together from the
  constraints.

**If you revive magnetometer fusion, use a modern filter and benchmark it:**

- **Laidig D, Seel T (2023).** *VQF: Highly accurate IMU orientation
  estimation with bias estimation and magnetic disturbance rejection.* Inf
  Fusion 91:187–204. [doi:10.1016/j.inffus.2022.10.014](https://doi.org/10.1016/j.inffus.2022.10.014) ·
  [arXiv](https://arxiv.org/abs/2203.17024) · [code (C++/Python/Matlab, `pip install vqf`)](https://github.com/dlaidig/vqf)
  Runs 6D and 9D fusion at the same time, estimates gyro bias, and detects and
  rejects magnetic disturbances. Average RMSE was 2.9°, against 5.3–16.7° for
  other methods. It also works 6D-only, so it is a drop-in candidate to A/B
  against the on-board Mahony filter even without the magnetometer.
- **Caruso M, Sabatini AM, Laidig D, et al. (2021).** *Analysis of the
  Accuracy of Ten Algorithms for Orientation Estimation Using Inertial and
  Magnetic Sensing under Optimal Conditions: One Size Does Not Fit All.*
  Sensors 21(7):2543. [doi:10.3390/s21072543](https://doi.org/10.3390/s21072543) ·
  [code](https://github.com/marcocaruso/sensor_fusion_algorithm_codes)
  Shows that filter gains (such as `TWO_KP`) matter as much as the choice of
  algorithm, and that no single tuning wins everywhere.

**How this bears on Area 1's conclusion.** The literature agrees with §1.6.
Magnetometers, and even perfect orientation, recover the *relative sensor
orientation*, not the *sensor-to-bone* mounting. Every paper above that
reports anatomical angles adds a separate calibration step, which is Area 2.

---

## 4. Area 2: Absolute (anatomical) angle via functional calibration

This is the most mature part of the literature, and it offers several ready
recipes. Order of reading: Vitali & Perkins (§2) for the taxonomy, Seel 2014
(§1) for the canonical hinge-axis method, then the papers below.

**Functional (motion-based) calibration:**

- **Favre J, Jolles BM, Aissaoui R, Aminian K (2008).** *Ambulatory
  measurement of 3D knee joint angle.* J Biomech 41(5):1029–35.
  [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0021929007005350)
- **Favre J, Aissaoui R, Jolles BM, de Guise JA, Aminian K (2009).**
  *Functional calibration procedure for 3D knee joint angle description using
  inertial sensors.* J Biomech 42(14):2330–5.
  [PubMed 19665712](https://pubmed.ncbi.nlm.nih.gov/19665712/)
  The classic knee **functional calibration**. Prescribed movements define
  the flexion axis in each sensor frame. This is essentially the "prescribed
  pure knee-flexion reps" idea in §2.2.
- **McGrath T, Fineman R, Stirling L (2018).** *An Auto-Calibrating Knee
  Flexion-Extension Axis Estimator Using Principal Component Analysis with
  Inertial Sensors.* Sensors 18(6):1882.
  [doi:10.3390/s18061882](https://doi.org/10.3390/s18061882)
  (A [correction](https://pmc.ncbi.nlm.nih.gov/articles/PMC6479793/) was
  published.) **Recommended first prototype for Area 2.** It runs PCA on the
  gyro signals during gait to find the dominant rotation axis, which is the
  flexion axis, with no alignment and no discrete calibration step. It is
  simple, cheap to compute, and uses gyro data you already stream.
- **Olsson F, Kok M, Seel T, Halvorsen K (2020).** *Robust Plug-and-Play Joint
  Axis Estimation Using Inertial Sensors.* Sensors 20(12):3534.
  [doi:10.3390/s20123534](https://doi.org/10.3390/s20123534)
  Makes the Seel-style axis estimator robust to arbitrary, non-ideal motion,
  so it does not need a clean calibration window.
- **Nowka D, Kok M, Seel T (2019).** *On motions that allow for identification
  of hinge joint axes from kinematic constraints and 6D IMU data.* ECC,
  4325–31. [doi:10.23919/ECC.2019.8795846](https://doi.org/10.23919/ECC.2019.8795846)
  **Important for protocol design.** It proves which motions make the axis
  identifiable from 6D (magnetometer-free) data. Planar motion is sufficient
  **unless the joint axis stays perfectly horizontal**. A seated or standing
  knee-flexion rep keeps the knee axis roughly horizontal, so the calibration
  protocol needs some variation in axis orientation, for example moving the
  thigh as well. Check this against the paper before you finalize the
  protocol.
- **Müller P, Bégin M-A, Schauer T, Seel T (2017).** *Alignment-Free,
  Self-Calibrating Elbow Angles Measurement Using Inertial Sensors.* IEEE JBHI
  21(2):312–319. [PubMed 28113331](https://pubmed.ncbi.nlm.nih.gov/28113331/)
  Uses arbitrary motion plus one zero-reference pose. That is architecturally
  the closest thing to "keep the current zero-hold, add axis identification".
- **Laidig D, Müller P, Seel T (2017).** *Automatic anatomical calibration for
  IMU-based elbow angle measurement in disturbed magnetic fields.* Curr Dir
  Biomed Eng 3(2):167–170.
  [doi:10.1515/cdbme-2017-0035](https://doi.org/10.1515/cdbme-2017-0035)
  Links Area 1 and Area 2: automatic sensor-to-segment calibration that is
  robust to magnetic disturbance.

**Anatomical (landmark or known-pose) calibration:**

- **Picerno P, Cereatti A, Cappozzo A (2008).** *Joint kinematics estimate
  using wearable inertial and magnetic sensing modules.* Gait Posture
  28(4):588–95. [PubMed 18502130](https://pubmed.ncbi.nlm.nih.gov/18502130/)
  Anatomical axes are measured directly with a device aligned to palpable
  landmarks. This is the "known-pose / external reference" option in §2.2.
- **Cutti AG, Ferrari A, Garofalo P, et al. (2010).** *"Outwalk": a protocol
  for clinical gait analysis based on inertial and magnetic sensors.* Med Biol
  Eng Comput 48:17–25.
  [doi:10.1007/s11517-009-0545-x](https://doi.org/10.1007/s11517-009-0545-x)
  A complete clinical protocol that combines a static posture with functional
  flexion. It is a useful model for writing an REB-ready calibration
  procedure.

**Comparing calibration methods head to head:**

- **Lebleu J, Gosseye T, Detrembleur C, et al. (2020).** *Lower Limb
  Kinematics Using Inertial Sensors during Locomotion: Accuracy and
  Reproducibility of Joint Angle Calculations with Different Sensor-to-Segment
  Calibrations.* Sensors 20(3):715.
  [doi:10.3390/s20030715](https://doi.org/10.3390/s20030715)
  Four functional calibrations were compared against optical motion capture.
  RMSE was below 3.6° for most methods, the **squat calibration was least
  accurate**, and sagittal reproducibility was excellent (ICC > 0.91). This is
  a directly usable guide to *which* calibration motion to prescribe.

**Takeaway for Area 2.** The field has settled on **functional axis
identification** (Seel 2014, McGrath 2018, Olsson 2020), sometimes combined
with **one static reference pose** (Müller 2017, Outwalk). That combination
matches the plan in §2.2. Two cautions from the literature: excitation
matters (Nowka 2019), and the choice of calibration motion changes accuracy
(Lebleu 2020). Even anatomical calibration does not resolve the contracture
case in §1.6 unless a static pose is referenced to an external truth such as
a goniometer or imaging.

---

## 5. Area 3: Bluetooth (BLE) links

- **Veijalainen P, Charalambous T, Wichman R (2022).** *Feasibility of
  Bluetooth Low Energy for motion capturing with Inertial Measurement Units.*
  J Netw Comput Appl.
  [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S1084804522002077)
  Builds a BLE 5 IMU motion-capture system from commercial parts. Static
  accuracy and delay matched or beat commercial suits, and throughput was
  enough for real-time use. It also identifies the bottlenecks. Read this
  before prototyping Link B.
- **Tipparaju VV, Mallires KR, Wang D, Tsow F, Xian X (2021).** *Mitigation of
  Data Packet Loss in Bluetooth Low Energy-Based Wearable Healthcare
  Ecosystem.* Biosensors 11(10):350.
  [MDPI](https://www.mdpi.com/2079-6374/11/10/350)
  **Directly actionable.** Lowering the notification rate and **bundling
  several samples per notification** cut packet loss below 1%, and a
  queue/re-request protocol removed the rest. This matches the "batch several
  samples per notification" mitigation in §3.3.
- **Buranapuntalug et al. (2026)**, see §1. Nano 33 BLE boards streaming to a
  phone over BLE. It is proof that the same hardware works wirelessly for this
  task.
- **Krull N, Schulthess L, Magno M, Benini L, Leitner C (2025).** *Wireless
  Low-Latency Synchronization for Body-Worn Multi-Node Systems in Sports.*
  [arXiv:2509.06541](https://arxiv.org/abs/2509.06541) (also IEEE conference
  proceedings). Covers body-worn BLE nodes that need low latency and tight
  synchronization.

**What this means for Area 3.** The recommendation in §3.3 to **timestamp at
the source** is standard practice. Sample batching is the proven fix for loss
at 50 Hz. BLE delay and throughput are feasible for real-time joint angles.

---

## 6. Area 4: Multiple nodes / joints and time synchronization

**Multi-segment kinematics (what to build toward):**

- **Al Borno M, O'Day J, Ibarra V, et al. (2022).** *OpenSense: An open-source
  toolbox for inertial-measurement-unit-based measurement of lower extremity
  kinematics over long durations.* J NeuroEng Rehabil 19:22.
  [doi:10.1186/s12984-022-01001-x](https://doi.org/10.1186/s12984-022-01001-x)
  Uses sensor fusion plus OpenSim inverse kinematics on a constrained model
  (hip, knee, ankle) and assesses drift over 10-minute trials. It is the
  natural PC-side back end if the project grows to N segments: export
  per-segment quaternions and let OpenSense handle the kinematic chain.
- **Teufl et al. (2018)**, see §3. Multi-segment 3-D kinematics without a
  magnetometer.

**Synchronization (the biggest correctness risk named in §4.3):**

- **Coviello G, Avitabile G (2020).** *Multiple Synchronized Inertial
  Measurement Unit Sensor Boards Platform for Activity Monitoring.* IEEE
  Sensors J. [IEEE Xplore](https://ieeexplore.ieee.org/iel7/7361/9132591/09044713.pdf) ·
  related open-access paper: *A Synchronized Multi-Unit Wireless Platform for
  Long-Term Activity Monitoring*, Electronics 9(7):1118,
  [MDPI](https://www.mdpi.com/2079-9292/9/7/1118)
  Low-cost wireless IMU nodes that log locally with a lightweight
  synchronization algorithm. They report tens of microseconds of mismatch over
  a 24-hour test, and they have a companion study on IMU sampling-rate
  mismatch between nodes.
- **Samarasekera T, et al. (2026).** *A Synchronized Multi-IMU Wearable System
  for Tracking of Joint-Angles in Sports Motion Analysis With Reference-Based
  Validation and Dynamic Task Characterization.*
  [arXiv:2607.26027](https://arxiv.org/abs/2607.26027)
  A modular multi-IMU platform with **RTC-disciplined microsecond
  timestamps** at each node and relative-rotation joint angles. It is
  validated on **seated knee flexion-extension** against a markerless video
  reference, and shows near-zero drift over a 2 h 12 min hold. The dataset is
  public (SynIMU-Sport on IEEE DataPort). This is architecturally very close to
  a scaled-up version of this project.
- **Krull et al. (2025)**, see §5. BLE multi-node synchronization.
- *Synchronisation of multiple unconnected inertial measurement units using
  software correction* (2025). J Biomech.
  [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S0021929025001435)
  Aligns streams **after the fact**, which fits topology B (independent pairs
  aggregated at the PC).

**What this means for Area 4.** The literature points to source timestamps
plus a periodic clock-sync exchange, and possibly an event-based
post-alignment (for example a shared tap or impulse) as a backstop. Topology
A (a BLE star) is where most published multi-node BLE work sits.

---

## 7. Validation methodology: the clinical benchmark

To claim the device "works", compare it with the clinical instrument it
would replace, and use accepted reliability statistics.

- **Gogia PP, Braatz JH, Rose SJ, Norton BJ (1987).** *Reliability and
  validity of goniometric measurements at the knee.* Phys Ther
  67(2):192–195. [Oxford Academic](https://academic.oup.com/ptj/article-abstract/67/2/192/2728138)
  The classic validation of the manual goniometer against radiographs. It
  sets the bar the IMU must meet: agreement within the known error of manual
  goniometry.
- **Poitras 2019** and **Kobsar 2020** (§2) give pooled validity and
  reliability numbers per joint and task to benchmark against.
- **Lebleu 2020** (§4) models how to report per-plane RMSE, amplitude
  difference, and ICC.

A suggested protocol, drawn from the papers above: (1) a bench test on a rigid
hinge or protractor (Song 2022, Lee & Jeon 2019); (2) static poses against a
manual goniometer (Gogia 1987); (3) dynamic sit-to-stand and gait against
optical or markerless video (Williamson 2001, Samarasekera 2026). Report RMSE,
Bland–Altman limits of agreement, and ICC.

---

## Suggested reading order

1. Weygers 2020 and Vitali & Perkins 2020, for the overall map.
2. Williamson & Andrews 2001 and Seel 2014, for this design and its canonical
   upgrade.
3. Area 2 prototype: McGrath 2018, then Nowka 2019 (protocol constraints),
   then Lebleu 2020 (choice of calibration motion).
4. Area 1: de Vries 2009 (field gate), then Laidig 2017 ICORR / Teufl 2018
   (magnetometer-free 3-D), then VQF (a drop-in filter to A/B).
5. Areas 3 and 4: Tipparaju 2021 (batching), Veijalainen 2022, Samarasekera
   2026 (sync architecture), and OpenSense (multi-joint back end).
