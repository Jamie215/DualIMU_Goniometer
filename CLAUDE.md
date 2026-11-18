# CLAUDE.md

Knee goniometer made of two Arduino Nano 33 BLE Rev2 boards. The thigh board
(central) and the shank board (peripheral) each run an orientation filter. They are
linked by wired UART (`Serial1`, 115200 baud). The central streams both segments
to a PC, where Python computes and logs the knee angle. See README.md for the full
picture.

## Layout

- `central_imu/central_imu.ino`: thigh board; reads the peripheral stream, streams to the PC
- `peripheral_imu/peripheral_imu.ino`: shank board; streams packets over UART
- `knee_collector_uart.py`: PC collector: calibration, angle math, CSV, `--selftest`
- `knee_gui.py`: live GUI; imports the math from `knee_collector_uart.py`
- `analysis/static_test/`, `analysis/diagnostics/`: test analysis scripts and the stationary test report
- `.github/workflows/release.yml`: builds `KneeGoniometerGUI.exe` on a `v*` tag

## Commands

Use **Python 3.12** (same as CI).

```bash
pip install -r requirements.txt
python knee_collector_uart.py --selftest   # math tests, no hardware
python knee_gui.py --selftest              # GUI tests, no hardware
python knee_gui.py --simulate              # GUI with fake data (needs a display)
```

Both selftests must pass before a change is done. The release build runs
`knee_gui.py --selftest` and stops if it fails.

## Rules

- **Wire protocol** is shared by both sketches and the collector. If you change
  any part, change every side in the same change:
  - peripheral → central: 30-byte packet, `0xAA` header, XOR checksum
  - diagnostics: `0xAB` frame carrying the `PDiag` struct (must be identical in both `.ino` files)
  - central → PC: 18-field `D,...` text line
- **Firmware changes** mean reflashing **both** boards. Say so in the release notes.
- **CSV columns** are read by analysis code and by users. Changing them is a
  breaking change (major version bump).
- The design goal is a **placement-independent** angle. Don't add assumptions
  about exact board orientation.
- **Releases:** push a `v*` tag; `release.yml` builds the exe and publishes it.
  Never reuse a tag. Don't tag or push tags unless asked.
- **Docs** are Markdown only. Don't add `.docx` copies. The one exception is
  `analysis/static_test/Static_Stability_Test_Report.docx`, a finished report.
