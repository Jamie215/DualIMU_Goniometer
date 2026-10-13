---
title: "Dual-IMU Knee Goniometer — Handover Summary"
---

# Handover summary

State of the project at transition (October 2026), what is in the repository,
what is missing, and what to do next.

## Current state

- **Working bench prototype.** Two Arduino Nano 33 BLE Rev2 boards (thigh =
  central, shank = peripheral) in basic cases, joined by soldered UART wires,
  each powered over USB; the central streams to a PC at a true 50 Hz.
- **Software complete for PoC testing.** Live GUI with guided calibration and
  auto-saved sessions; CLI collector; self-tests pass. The GUI is released as a
  Windows program, `KneeGoniometerGUI.exe` (v1.0.0, Releases page).
- **SOP finalised** (`docs/SOP_Build_and_Use.docx`, version 1.0): build, setup,
  sessions and troubleshooting for operators; Python setup and releasing a new
  GUI version are in its appendices.
- **Preliminary stationary testing done.** ≈0.01–0.02° noise, ≈ −0.001 °/min
  drift on healthy hardware; 45°/90° rigs read 45.0°/89.0° (preliminary).
- **Not yet done:** dynamic / on-body testing, both-node calibration, a
  controlled accuracy test.

## Documents

| Document | Use it for |
|---|---|
| `docs/SOP_Build_and_Use.docx` (master; `.md` mirror) | Building the device and running a session |
| `docs/DESIGN_DECISIONS.md` (+ `.docx`) | Why the system is the way it is |
| `analysis/static_test/Static_Stability_Test_Report.docx` | Preliminary Stationary Test Report |
| `analysis/static_test/FINDINGS.md` | Detailed working record behind the report |
| `README.md` | Technical reference: math, wire protocol, commands |
| `docs/areas_to_explore.md` | Analyses of magnetometer, absolute angle, BLE, multi-node |

## Repository map

```
central_imu/central_imu.ino          thigh board firmware
peripheral_imu/peripheral_imu.ino    shank board firmware
knee_collector_uart.py               angle math, calibration, CLI collector, --selftest
knee_gui.py                          live GUI (uses knee_collector_uart.py)
requirements.txt                     Python dependencies
analysis/static_test/                stationary test scripts, figures, report
analysis/diagnostics/parse_diag.py   diagnostic log -> tables
docs/                                SOP, design decisions, handover, research areas
docs/build_docs.sh                   regenerates the design-decisions and handover .docx from Markdown
.github/workflows/release.yml        builds KneeGoniometerGUI.exe (Windows GUI) on a v* tag
```

## Open issues

1. **Faulty board.** One board's BMI270 fails after 20–40 min. It must be
   identified, labelled and replaced. It should be the board without the red
   wire soldered to the 3.3 V pin; confirm before replacing it.
2. **Raw test data not preserved.** The stationary-test CSVs and diagnostic
   logs were not archived (`*.csv` / `*.log` are git-ignored), so the figures in
   the report cannot be regenerated. Plan data storage once testing moves
   beyond the preliminary stage.
3. **Case design file not in the repository.** The SOP uses a custom 3-D printed
   case (`DualIMU_Goniometer.STL`) and elastic straps; the STL file should be
   added to the repository so the case can be reprinted.
4. **Analysis scripts are working scripts.** `analysis/static_test/stats.py`,
   `plot.py` and `calibration.py` were written ad hoc for this dataset (they
   read every CSV in the current folder). `plot_report.py` and
   `plot_angle_rig.py` produced the report figures and expect specific file
   names.
5. **Peripheral-only fault guard.** The sensor-fault guard runs on the
   peripheral only; the central has no equivalent.
6. **Angle log resolution.** Angles are written to 0.01°, which limits how low
   a noise floor can be measured.

## Recommended next steps (in order)

1. Replace the faulty board; repeat a one-hour stationary run on two healthy
   boards for a clean baseline.
2. Sweep **both** nodes during calibration so the angle uses both sensors; fix
   the boards and cables firmly.
3. Try the strap placement in the SOP on a person and refine it if the boards
   shift during movement.
4. Controlled accuracy test: repeated 0°/45°/90° placements on a precise rig or
   against a goniometer.
5. Dynamic range-of-motion testing (tune `TWO_KP` and the soft accel gate if
   fast reps lag or overshoot).
6. Consider the longer-term directions in `docs/areas_to_explore.md` (battery +
   BLE for an untethered device; absolute angle via functional calibration).

## Housekeeping

- No licence file — decide whether one is needed before sharing the code.
- **GUI release:** `v1.0.0` is published with `KneeGoniometerGUI.exe` on the
  Releases page (tag on `main` at the handover). To publish a changed GUI, follow
  "Releasing an updated GUI" in the README (also SOP Appendix B): merge to `main`, push
  the next version tag, and the release workflow builds and attaches the exe.
- Regenerate the design-decisions and handover Word copies after editing their
  Markdown: `bash docs/build_docs.sh`. The SOP is edited in Word directly.
