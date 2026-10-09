---
title: "Dual-IMU Knee Goniometer — SOP: Build and Use"
subtitle: "Standard operating procedure for assembling, setting up and running the prototype"
---

# 1. Purpose and scope

This SOP describes how to build the dual-IMU knee goniometer prototype, set up
its software, and run a data-collection session. It covers the device as it
currently exists: a **bench / proof-of-concept prototype** with two cased
Arduino boards joined by soldered wires, each powered over USB, with the
central board connected to a PC.

Items marked **[TO FILL]** are details of the physical build that are not
recorded in the repository. Fill them in when the device is handed over.

For the reasoning behind any step, see `docs/DESIGN_DECISIONS.md`. For the
math and protocol, see `README.md`.

> **Safety and use limits.** This is a research prototype, not a medical
> device. It is tethered to a computer and to USB power, and a wire runs across
> the knee. Use it on the bench or for supervised, low-risk movement only; keep
> cables clear of feet and furniture. On-body use with participants requires
> the appropriate ethics approval.

# 2. Bill of materials

| Qty | Item | Notes |
|---|---|---|
| 2 | Arduino Nano 33 BLE **Rev2** | Onboard BMI270 IMU. Label one **CENTRAL (thigh)** and one **PERIPHERAL (shank)**. |
| 2 | Board case | **[TO FILL]** model / source / 3-D print file |
| 3 | Inter-board wires | Two signal wires + one ground, soldered. Gauge **[TO FILL]**, length **[TO FILL]** |
| — | Strain relief / heat-shrink | **[TO FILL]** |
| 2 | USB cable (micro-USB to PC/charger) | One for each board. The central's cable carries data. |
| 1 | USB power for the peripheral | Second PC USB port, USB charger, or power bank |
| 1 | PC | Windows, macOS or Linux; Python 3.8+ |
| — | Mounting to the leg | **[TO FILL]** (strap / tape / sleeve) — not yet defined |

> **Known faulty board.** One of the two original boards has a faulty BMI270
> sensor that fails after 20–40 min of use (Stationary Test Report). It is
> **[TO FILL: identify / label which board]**. Replace it before further testing.

# 3. Building the device

## 3.1 Wiring

Solder three wires between the boards. Both UART directions are required.

| From | To | Purpose |
|---|---|---|
| Peripheral **TX (D1)** | Central **RX (D0)** | Shank data stream |
| Central **TX (D1)** | Peripheral **RX (D0)** | Keepalive / start–stop |
| Peripheral **GND** | Central **GND** | Common ground |

Rules:

1. TX always goes to RX on the other board (crossed), never TX–TX.
2. Do **not** connect the two boards' 3.3 V or VIN pins together; each board is
   powered from its own USB.
3. Add strain relief where the wires leave each case; a broken ground or
   keepalive wire shows up as a dead peripheral (see §8).
4. Check continuity of all three wires with a multimeter, and check there is no
   short between adjacent pins.

## 3.2 Enclosure and labelling

1. Fit each board in its case with the USB port accessible.
2. Mark on each case: its role (CENTRAL / PERIPHERAL), and an arrow along the
   board's long axis pointing toward the USB port. (Placement convention, §6.)
3. Make sure the built-in LED is visible (it is the main status indicator).

# 4. Software setup (once per PC)

## 4.1 Firmware tools

1. Install the **Arduino IDE** (2.x).
2. Boards Manager → install **Arduino Mbed OS Nano Boards**.
3. Library Manager → install **Arduino_BMI270_BMM150**.

## 4.2 Flashing the boards

Flash both boards **from the same commit** (the diagnostic frame layout must
match on both).

1. Connect the **central** board by USB. Open `central_imu/central_imu.ino`.
   Select board **Arduino Nano 33 BLE** and the board's port. Upload. Wait for
   *Done uploading*.
2. Connect the **peripheral** board. Open `peripheral_imu/peripheral_imu.ino`.
   Upload. On restart its LED gives **3 quick flashes** then a **slow blink**.
3. Optional check: open the Serial Monitor on the central at 115200 baud. It
   prints `# CENTRAL fw: gated-50hz-onewrite-diag ...`. Close the Serial
   Monitor afterwards — only one program can hold the port.

Both sketches ship with `#define DIAG 1` (once-a-second diagnostic lines).
Leave it on unless there is a reason not to; if you change it, change both.

## 4.3 Python environment

```
python -m venv .venv
# Windows: .venv\Scripts\activate      macOS/Linux: source .venv/bin/activate
pip install -r requirements.txt
python knee_collector_uart.py --selftest     # expect: Self-test PASSED.
python knee_gui.py --selftest                # expect: knee_gui self-test PASSED.
python knee_gui.py --simulate                # optional: GUI with synthetic data, no hardware
```

# 5. Pre-session checklist

1. Both boards powered: central by USB **to the PC**, peripheral by its own USB.
2. Peripheral LED shows a **slow blink** (idle, waiting).
3. Inter-board wires intact; no strain on solder joints.
4. Correct (non-faulty) boards fitted.
5. Close the Arduino Serial Monitor and any other program using the central's port.

# 6. Placing the sensors on the leg

1. **Central (thigh)** on the front/side of the thigh, above the knee;
   **peripheral (shank)** on the shank, below the knee.
2. Align each board's long axis roughly along the limb, USB port toward the hip.
   Exact rotation and position do **not** need to match — the calibration
   removes the mounting orientation.
3. Fix both boards **firmly** so they cannot slide or rotate during the session
   (method **[TO FILL]**). Movement of a board relative to the limb after
   calibration appears as angle error. Route the wires so they do not pull on
   either board.

For bench tests, tape boards to rigid surfaces (e.g. a desk and a ruler or a
hinged rig).

# 7. Running a session (GUI — recommended)

1. Start the GUI: `python knee_gui.py`. It scans the serial ports and picks
   out the central (use **Rescan** if it is not listed).
2. Click **Connect**. Opening the port starts the boards: the central's LED turns **on**, the
   peripheral's LED switches to a **fast blink**. **Keep both boards still for
   ~4 s** — each board measures its gyro bias now.
3. Get into the zero pose: **leg straight and still** (or the bench reference pose).
4. Click **Calibrate & collect**. Follow the banner:
   - **ZEROING** (amber, 2 s): hold completely still. This pose becomes 0°.
   - **SWEEP** (amber, 6 s): do a few slow movements that bend **both the knee
     and the hip** (e.g. marching or sit-to-stand), so **each** board tilts by
     more than 5°. A board that does not tilt is treated as fixed and does not
     contribute to the angle.
   - **RUNNING** (green): perform the task. The angle is positive in flexion.
5. Click **Stop collecting** when done. The plots switch to the whole session
   for review (pan / zoom / save toolbar).
6. For another trial, click **Calibrate & collect** again — it recalibrates and
   writes a new file. (There is no pause/resume.)
7. Close the GUI (or disconnect). The boards return to idle.

**Display mode:** *Fill (line)* forward-fills short gaps; *Gap (points)* shows
dropouts as gaps. It only changes the display; the CSV records the real status
of every sample either way.

## 7.1 Command-line alternative

```
python knee_collector_uart.py --port COM5 --monitor   # live link / tilt check
python knee_collector_uart.py --port COM5 --out trial1.csv
```

Use the port of the central (`COMx` on Windows, `/dev/ttyACM0` on Linux,
`/dev/cu.usbmodem…` on macOS). The CLI prompts the same zero (2 s) and sweep
(6 s) phases (`--cal-seconds`, `--sweep-seconds` to change them).

# 8. Data handling

| File | Contents |
|---|---|
| `knee_YYYYMMDD_HHMMSS.csv` | One row per sample (50 Hz): time, both quaternions, knee angle, both segment inclinations, status, phase, packet age, raw accelerometers |
| `knee_diag_YYYYMMDD_HHMMSS.log` | Firmware banners and once-a-second `# CDIAG` / `# PDIAG` health lines |

1. Files are written to the folder the GUI was started from; **Save copy…**
   copies the current session elsewhere. Sessions auto-save; a crash does not
   lose data.
2. Rename or record each file against the trial in a run log (date, subject /
   rig, placement, notes).
3. **Archive raw data immediately** to the project's data store
   **[TO FILL: location]**. CSV and log files are excluded from git, and the raw
   data from the stationary tests was not preserved.
4. For analysis, skip the calibration phases (`phase` column ≠ `running`).
   Stationary analysis also excluded the first 20 s of each session.
5. `python analysis/diagnostics/parse_diag.py knee_diag_<timestamp>.log` turns a
   diagnostic log into per-second CSVs and a per-minute summary.

# 9. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| GUI finds no port / "No data on port" | Central not connected, not powered, or port held by another program | Reconnect USB; close Serial Monitor; **Rescan** |
| "Isn't an 18-field 'D' line" | Old or wrong firmware | Reflash **both** boards from the same commit (§4.2) |
| "Every shank quaternion is the zero sentinel" | No data from the peripheral | Check peripheral power and LED; check both UART wires and GND |
| Peripheral LED slow blink while collecting | Peripheral not receiving the keepalive | Check Central TX (D1) → Peripheral RX (D0) wire and GND |
| Peripheral LED solid on | Sensor stalled; firmware re-initialising | Wait a few seconds; if it persists or the board keeps rebooting (3 flashes repeatedly), suspect a faulty IMU — swap the board |
| No LED on peripheral | Unpowered or hard fault | Check its USB power; reflash |
| Angle stays at 0 while moving | Board not swept during calibration (< 5° tilt) | Recalibrate with a larger sweep that tilts both boards |
| Angle wrong sign | Sweep direction | Positive = flexion relative to the sweep direction; sweep in flexion |
| Angle creeps slowly at rest | A board or its mount is moving | Fix boards more firmly; check cable pull |
| Shank data lost after 20–40 min | Faulty BMI270 (known on one board) | Replace the board |
| Low "valid %" | Loose wire or noise on the link | Check solder joints; shorten / twist wires |
| GUI stalls or rows look duplicated | Very old GUI version | Update to the current `knee_gui.py` (one row per sample) |

# 10. After the session

1. Close the GUI so the boards stop (central LED off, peripheral slow blink).
2. Unplug both USB cables.
3. Archive the CSV and log files (§8).
4. Note any faults (LED states, error banners, rebooting) in the run log.
