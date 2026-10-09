# Dual-IMU Knee Goniometer — SOP: Build and Use

> **The Word file `docs/SOP_Build_and_Use.docx` is the master copy of this SOP.** This
> Markdown is a read-only mirror for viewing on GitHub; edit the Word file, then refresh
> this mirror with:
> `pandoc docs/SOP_Build_and_Use.docx -t gfm --extract-media=docs/images/sop -o docs/SOP_Build_and_Use.md`
> (and change `docs/images/` to `images/` in the image links).

# 1. Purpose and scope

This SOP describes how to build the dual-IMU knee goniometer prototype,
set up its software, and run a data-collection session. It covers the
device as it currently exists: a bench / proof-of-concept prototype with
two cased Arduino boards joined by soldered wires, each powered over
USB, with the central board connected to a PC.

> **Safety and use limits.** This is a research prototype. It is
> connected by cable to a computer and to USB power, and a wire runs
> across the knee. Use it on the bench or for supervised, low-risk
> movement only, and keep cables clear of feet and furniture. For a real
> test deployment, consider a wireless version.

<img src="images/sop/media/image1.jpeg"
style="width:3.44306in;height:2.71528in" />

Figure 1. MSK-IF version research prototype

# 2. Materials

| **Qty** | **Item**                            | **Notes**                                                                                             |
|---------|-------------------------------------|-------------------------------------------------------------------------------------------------------|
| 2       | Arduino Nano 33 BLE Rev2            | Each has a built-in motion sensor (BMI270). Label one “CENTRAL (thigh)” and one “PERIPHERAL (shank)”. |
| 2       | Board case                          | Custom 3D-printed (DualIMU_Goniometer.STL)                                                            |
| 3       | Inter-board wires                   | Two signal wires + one shared ground, soldered.                                                       |
| 2       | USB cable (micro-USB to PC/charger) | One for each board. The thigh (central) board’s cable also carries the data to the PC.                |
| 1       | USB power for the peripheral        | Second PC USB port, USB charger, or power bank                                                        |
| 1       | PC                                  | Windows, macOS or Linux; Python 3.8+                                                                  |
| 2       | Mounting to the leg                 | Elastic straps                                                                                        |

> Note: If using the research prototype developed by MSK-IF: the motion
> sensor on one of the two original boards is faulty and stops working
> after 20–40 minutes of use (see the Preliminary Stationary Test
> Report). Replace that board before further testing.

# 3. Building the device

## 3.1 Wiring

Solder three wires between the two boards, as in the table below. One
wire carries the shank board’s measurements to the thigh board, one lets
the thigh board tell the shank board when to start and stop, and one is
a shared ground. Each signal wire goes from a TX (send) pin on one board
to the RX (receive) pin on the other — never TX to TX.

| **From**               | **To**                 | **Purpose**                                             |
|------------------------|------------------------|---------------------------------------------------------|
| Peripheral **TX (D1)** | Central **RX (D0)**    | Sends the shank board’s measurements to the thigh board |
| Central **TX (D1)**    | Peripheral **RX (D0)** | Tells the shank board to start and stop                 |
| Peripheral **GND**     | Central **GND**        | Shared ground (the other two wires need it to work)     |

<img src="images/sop/media/image2.png"
style="width:5.08333in;height:3.56691in" />

Figure 2. Arduino Nano 33 BLE Rev 2 pinouts with relevant pins
highlighted

Note:

1.  Do **not** connect the two boards’ 3.3 V or VIN pins together; each
    board gets its own USB power.
2.  If needed, add strain relief (e.g. heat-shrink or a little hot glue)
    where the wires leave each case. A broken wire usually shows up as
    “no data from the peripheral” in the GUI (see §9).
3.  Before closing the cases, use a multimeter in continuity (beep) mode
    to check that each wire joins the right two pins, and that no two
    neighbouring pins are accidentally joined.

## 3.2 Enclosure and labelling

1.  Print two cases from `DualIMU_Goniometer.STL`, one per board (each
    case is a base and a lid).
2.  Fit each board in its case with the USB port accessible.
3.  Mark on each case its role (CENTRAL / PERIPHERAL) and an arrow along
    the board’s long side pointing toward the USB port. The arrow is
    used when placing the sensors (§6).

# 4. Software setup (once per PC)

## 4.1 Firmware tools

The Arduino IDE is a free program used to load the programs (“firmware”)
onto the boards.

1.  Install the **Arduino IDE** (version 2.x) from arduino.cc.
2.  In the Arduino IDE, open **Tools → Board → Boards Manager**, search
    for **Arduino Mbed OS Nano Boards** and click Install.
3.  Open **Tools → Manage Libraries**, search for
    **Arduino_BMI270_BMM150** and click Install.

## 4.2 Flashing the boards

Download the project files to the PC from the project repository ([Link
to Repository](https://github.com/Jamie215/DualIMU_Goniometer)). Load
(“flash”) both boards from the same version of the files.

1.  Connect the thigh (central) board to the PC by USB. In the Arduino
    IDE, open `central_imu/central_imu.ino` from the project folder.
    Select the board **Arduino Nano 33 BLE** and its port (**Tools →
    Port**), then click **Upload** (the → arrow). Wait for the message
    *Done uploading*.
2.  Connect the shank (peripheral) board and repeat with
    `peripheral_imu/peripheral_imu.ino`. When it restarts, its light
    flashes 3 times quickly and then blinks slowly.
3.  Optional check: with the thigh board connected, open **Tools →
    Serial Monitor** and set it to 115200 baud. A line starting with
    “`# CENTRAL fw:`” means the program is loaded. Close the Serial
    Monitor afterwards — only one program can use the board’s connection
    at a time.

Both programs also send a short health report once a second, which the
GUI saves for troubleshooting. This is switched on by the line
“`#define DIAG 1`” near the top of each program; leave it as it is.

## 4.3 Python environment

The data-collection program (the GUI) runs on the PC and is written in
Python. This step installs Python and the add-ons the program needs. It
only needs to be done once per PC.

1\. Install Python 3.8 or newer from python.org. On Windows, tick “Add
Python to PATH” during installation. (On Linux, also install the
`python3-tk` package.)

2\. Open a terminal (on Windows, Command Prompt) in the project folder.

3\. Type the commands below one line at a time. Lines starting with \#
are notes, not commands. On macOS/Linux, type `python3` instead of
`python` in the first line.

    python -m venv .venv
    # Windows: .venv\Scripts\activate      macOS/Linux: source .venv/bin/activate
    pip install -r requirements.txt
    python knee_collector_uart.py --selftest     # expect: Self-test PASSED.
    python knee_gui.py --selftest                # expect: knee_gui self-test PASSED.
    python knee_gui.py --simulate                # optional: GUI with synthetic data

What the commands do: the first two lines create and switch on a
separate Python workspace for this project (a “virtual environment”), so
the add-ons do not affect anything else on the PC. Switch it on again
(the *activate* line) each time you open a new terminal. `pip install`
installs the add-ons. The two *selftest* lines check the installation
without any hardware; each should end with **PASSED**. The last line
opens the GUI with made-up data, so you can practise before using the
device.

# 5. Pre-session checklist

1.  Both boards powered: the thigh (central) board by USB to the PC, the
    shank (peripheral) board by its own USB power.
2.  The shank board’s light blinks slowly (standby, waiting to start).
3.  The wires between the boards are intact, with no strain on the
    solder joints.
4.  The correct (non-faulty) boards are fitted.
5.  The Arduino Serial Monitor, and any other program using the thigh
    board’s connection, is closed.

# 6. Placing the sensors on the leg

1.  Put the **thigh (central)** board on the thigh, above the knee, and
    the **shank (peripheral)** board on the shank, below the knee.
2.  Choose a spot where the skin and muscle move as little as possible
    when the knee bends and straightens: the flat middle part of each
    segment, away from the knee joint and the kneecap, and not over a
    muscle that bulges when it tightens. Good spots are the outer side
    of the lower thigh, about a hand-width above the kneecap, and the
    flat inner surface of the shin bone (or the outer side of the
    shank), about a hand-width below the knee. Do not place a board over
    the knee crease or the kneecap.
3.  Line up each board’s long side roughly with the leg, with the arrow
    (USB port) pointing toward the hip. The boards do not need to be
    lined up exactly — the calibration at the start of each session
    takes care of how they sit.
4.  Fasten each board **firmly** with an elastic strap — snug but
    comfortable — so it cannot slide or turn during the session. If a
    board moves on the leg after calibration, the angle will be wrong.
    Route the wire so it does not pull on either board.

# 7. Running a session (GUI — recommended)

1.  Start the GUI: in the terminal from §4.3 (with the virtual
    environment switched on), type `python knee_gui.py`. The program
    looks for the thigh board and selects it. If it is not listed, click
    **Rescan**.
2.  Click **Connect**. This switches both boards on: the thigh board’s
    light turns on and the shank board’s light starts blinking quickly.
    Keep both boards still for about 4 seconds while they measure their
    resting state (Figure 3).
3.  Get into the starting pose: **leg straight and still** (or the
    starting position of the bench setup). This pose will read as 0°.
4.  Click **Calibrate & collect** and follow the coloured banner at the
    top:
    - Amber “**CALIBRATING - HOLD STILL**” (2 s): hold completely still
      (Figure 4).
    - Amber “**CALIBRATING - SWEEP**” (6 s): slowly bend and straighten
      the leg a few times, moving **both the knee and the hip** (e.g.
      marching on the spot or a sit-to-stand), so that each board tilts
      by more than 5°. A board that does not tilt enough is left out of
      the angle (Figure 5).
    - Green “**RUNNING**”: carry out the task. The angle goes up as the
      knee bends (Figure 6).
5.  Click **Stop collecting** when done. The graphs then show the whole
    session so you can review it (Figure 7).
6.  For another trial, click **Calibrate & collect** again. It
    calibrates again and saves a new file.
7.  Close the GUI (or click **Disconnect**). The boards go back to
    standby.

The screenshots below show each step. They were taken in the GUI’s
practice mode (made-up data), so the numbers will differ from a real
session. The top graph is the knee angle, the middle graph shows the
tilt of each board, and the bottom graph shows how fresh the shank data
is (low, steady values are good). Bottom right, “valid %” is the share
of measurements received correctly.

<img src="images/sop/media/image3.png"
style="width:4.8in;height:3.84in" />

Figure 3. GUI after Connect, ready to calibrate (simulated data)

<img src="images/sop/media/image4.png"
style="width:4.8in;height:3.84in" />

Figure 4. Calibration — hold still (simulated data)

<img src="images/sop/media/image5.png"
style="width:4.8in;height:3.84in" />

Figure 5. Calibration — sweep, with a countdown and the live shank tilt
(simulated data)

<img src="images/sop/media/image6.png"
style="width:4.8in;height:3.84in" />

Figure 6. Running — the knee angle is recorded and plotted live
(simulated data)

<img src="images/sop/media/image7.png"
style="width:4.8in;height:3.84in" />

Figure 7. After Stop collecting — the whole session is shown for review
(simulated data)

**Dropouts setting** (bottom of the window): *Fill (line)* draws a
continuous line over short gaps in the data; *Gap (points)* shows each
gap. This only changes the display — the saved file is the same either
way.

## 7.1 Command-line alternative (optional, for technical users)

    python knee_collector_uart.py --port COM5 --monitor   # live link / tilt check
    python knee_collector_uart.py --port COM5 --out trial1.csv

Replace COM5 with the thigh board’s port (`COMx` on Windows,
`/dev/ttyACM0` on Linux, `/dev/cu.usbmodem…` on macOS). The command-line
program walks through the same hold-still (2 s) and sweep (6 s) steps;
`--cal-seconds` and `--sweep-seconds` change their length.

# 8. Data handling

| **File**                        | **Contents**                                                                                                                                                                     |
|---------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `knee_YYYYMMDD_HHMMSS.csv`      | One row per measurement (50 per second): time, knee angle, the tilt of each board, whether each measurement was received correctly, and raw sensor values for technical analysis |
| `knee_diag_YYYYMMDD_HHMMSS.log` | Messages from the boards, including the once-a-second health report; used for troubleshooting                                                                                    |

1.  Files are saved in the folder the GUI was started from. Each session
    is saved automatically while it runs, so nothing is lost if the
    program closes unexpectedly. **Save copy…** saves a copy somewhere
    else.
2.  Write down which file belongs to which trial (date, participant or
    setup, sensor placement, notes).
3.  For technical users: when analysing, leave out the calibration rows
    (the `phase` column is not `running`). The stationary analysis also
    left out the first 20 s of each session.
4.  For technical users:
    `python analysis/diagnostics/parse_diag.py knee_diag_<timestamp>.log`
    turns a health log into tables and a per-minute summary.

# 9. Troubleshooting

| **Symptom**                                      | **Likely cause**                                                         | **Action**                                                                                                                                    |
|--------------------------------------------------|--------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| GUI finds no port, or “No data on …”             | Thigh board not connected or not powered, or another program is using it | Reconnect the USB cable; close the Arduino Serial Monitor; click Rescan                                                                       |
| “… isn't an 18-field 'D' line …”                 | The boards have an old or wrong program                                  | Load both boards from the same version of the files (§4.2)                                                                                    |
| “… no data from the peripheral …”                | The computer is receiving nothing from the shank board                   | Check the shank board has power and its light is blinking; check all three wires (§3.1)                                                       |
| Shank board light blinks slowly during a session | The shank board is not getting the start signal                          | Check the wire from thigh TX (D1) to shank RX (D0), and the ground wire                                                                       |
| Shank board light stays on                       | Its motion sensor stopped responding; the board is trying to restart it  | Wait a few seconds. If it continues, or the light keeps flashing 3 times (the board restarting), the sensor may be faulty — replace the board |
| Shank board light off                            | No power, or the program has stopped                                     | Check its USB power; load the program again (§4.2)                                                                                            |
| Angle stays at 0 while moving                    | The boards did not tilt enough during the sweep                          | Calibrate again with bigger, slower movements that tilt both boards                                                                           |
| Angle goes negative when the knee bends          | The sweep did not bend the knee                                          | Calibrate again, bending the knee during the sweep                                                                                            |
| Angle slowly drifts while the leg is still       | A board or strap is moving                                               | Tighten the straps; make sure the wire is not pulling on a board                                                                              |
| Shank data stops after 20–40 min                 | Faulty motion sensor (known on one of the original boards)               | Replace the board                                                                                                                             |
| Low “valid %” (bottom right of the GUI)          | Loose wire or poor connection between the boards                         | Check the solder joints; keep the wires short and twisted together                                                                            |
| GUI freezes or saved rows repeat                 | Old version of the GUI                                                   | Update to the latest knee_gui.py from the repository                                                                                          |

# 10. After the session

1.  Close the GUI so the boards stop (thigh board light off, shank board
    blinking slowly).
2.  Unplug both USB cables.
3.  Keep the saved files (§8) together with your trial notes.
4.  Note any problems (lights, error messages, restarts) in your trial
    notes.
