#!/usr/bin/env python3
"""
KNEE GONIOMETER -- live collection + visualization GUI  [Tkinter + Matplotlib]

A proof-of-concept front end around knee_collector_uart.py. It finds the master
board's serial port, runs the same two-phase calibration and gravity-referenced
angle math as the CLI, and adds what a testing session wants:

  * PORT SCAN         -- probe each serial port for valid 'D' lines and pick the
                         master automatically (only the master is on USB; the
                         slave is diagnosed *through* it via shank-valid %).
  * CONSISTENT 100 Hz -- the jittery ~104 Hz device stream is resampled onto a
                         fixed 10 ms grid, so both the CSV and the plot are a
                         clean 100 Hz record regardless of source jitter or the
                         up-to-8 ms slave-poll stalls.
  * OBVIOUS CALIBRATION -- a big colour-coded banner drives the phases
                         (ZEROING -> SWEEP -> RUNNING) with a live countdown.
  * VISUALIZATION     -- knee angle (primary) plus the two segment inclinations
                         it is built from, and the link RTT.
  * DROPOUT MODE (switchable live):
        FILL  -- short gaps forward-filled, drawn as a continuous line.
        GAP   -- no fill; real samples only, drawn as discrete points so
                 dropouts show as visible gaps. The CSV records which samples
                 were real either way.
  * PAUSE + ERRORS    -- Pause freezes logging (display stays live); a red
                         banner names the exact fault (no data / wrong firmware /
                         dead slave link) using the shared diagnose_stream().
  * AUTO-SAVE         -- a timestamped CSV is opened at connect; "Save copy..."
                         relocates it. A crash never loses a session.

Run:
  python knee_gui.py                 # launch the GUI (scan for ports)
  python knee_gui.py --simulate      # launch with a synthetic source (no board)
  python knee_gui.py --selftest      # headless: exercise the sample-gate logic
"""

import argparse
import csv
import math
import random
import threading
import time
from collections import deque
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None
    list_ports = None

from knee_collector_uart import (
    parse_line, is_valid, gravity_in_board, average_gravity, estimate_forward,
    sagittal_inclination, gravity_knee_angle, _incl_from_zero,
    q_from_axis_angle, q_mul, diagnose_stream,
    CAL_SECONDS, SWEEP_SECONDS, MAX_FILL,
)

# --------------------------------------------------------------------------- #
# Tuning
# --------------------------------------------------------------------------- #
SAMPLE_HZ = 100
GRID_DT = 1.0 / SAMPLE_HZ          # fixed resample period (s)
PLOT_WINDOW_SEC = 12               # rolling x-window shown in the plots
PLOT_N = int(PLOT_WINDOW_SEC * SAMPLE_HZ) + 50
STALE_SEC = 0.08                   # a source sample older than this = no fresh data
LINK_ERROR_SEC = 1.5               # no valid sample for this long -> red banner
SIM_PORT = '[simulate]'            # sentinel port name for the synthetic source


# --------------------------------------------------------------------------- #
# Sample gate: the fill-vs-gap decision, isolated so it is unit-testable.
# One tick in, one (angle_or_None, status) out. In FILL mode short dropouts are
# forward-filled (continuous line); in GAP mode they are dropped (discrete
# points). Counters feed the valid-% readout in either mode.
# --------------------------------------------------------------------------- #
class SampleGate:
    def __init__(self, max_fill=MAX_FILL, fill_mode=True):
        self.max_fill = max_fill
        self.fill_mode = fill_mode
        self.last_good = None
        self.consecutive_bad = 0
        self.n_valid = 0
        self.n_filled = 0
        self.n_missing = 0

    def process(self, ok, angle):
        """ok: this tick has a fresh, valid sample. Returns (angle_or_None, status)."""
        if ok:
            self.last_good = angle
            self.consecutive_bad = 0
            self.n_valid += 1
            return angle, 'valid'
        self.consecutive_bad += 1
        if (self.fill_mode and self.last_good is not None
                and self.consecutive_bad <= self.max_fill):
            self.n_filled += 1
            return self.last_good, 'filled'
        self.n_missing += 1
        return None, 'missing'

    def valid_pct(self):
        total = self.n_valid + self.n_filled + self.n_missing
        if total == 0:
            return None
        return 100.0 * self.n_valid / total


# --------------------------------------------------------------------------- #
# Synthetic serial source for demoing / testing without hardware. Emits the same
# 18-field 'D' lines the master firmware does: a slowly flexing knee, with a
# periodic ~0.4 s shank dropout so the fill/gap modes and the error banner have
# something to react to.
# --------------------------------------------------------------------------- #
class FakeSerial:
    _MOUNT_T = q_from_axis_angle((0.4, -0.6, 0.7), math.radians(50))
    _MOUNT_S = q_from_axis_angle((-0.3, 0.8, 0.2), math.radians(80))

    def __init__(self, *_args, **_kwargs):
        self._t0 = time.monotonic()
        self._closed = False

    def _pitch_quat(self, pitch_deg, mount):
        return q_mul(q_from_axis_angle((0.0, 1.0, 0.0), math.radians(pitch_deg)), mount)

    def _line(self):
        t = time.monotonic() - self._t0
        thigh_pitch = 10.0 + 10.0 * math.sin(2 * math.pi * t / 6.0 + 0.5)
        shank_pitch = 45.0 + 45.0 * math.sin(2 * math.pi * t / 6.0)
        qt = self._pitch_quat(thigh_pitch, self._MOUNT_T)
        qs = self._pitch_quat(shank_pitch, self._MOUNT_S)
        gt = gravity_in_board(qt)
        gs = gravity_in_board(qs)
        t_us = int(t * 1e6)
        dropout = (t % 8.0) > 7.6          # periodic link glitch (~0.4 s / 8 s)
        rtt = 300 + int(40 * random.random())
        if dropout:
            sq = (0.0, 0.0, 0.0, 0.0); sa = (0.0, 0.0, 0.0); s_t = 0
        else:
            sq = qs; sa = gs; s_t = t_us
        f = lambda v: f"{v:.4f}"
        fields = ['D', str(t_us),
                  f(qt[0]), f(qt[1]), f(qt[2]), f(qt[3]), f(gt[0]), f(gt[1]), f(gt[2]),
                  str(s_t),
                  f(sq[0]), f(sq[1]), f(sq[2]), f(sq[3]), f(sa[0]), f(sa[1]), f(sa[2]),
                  str(rtt)]
        return (','.join(fields) + '\n').encode('ascii')

    def readline(self):
        if self._closed:
            return b''
        time.sleep(1.0 / 104.0)            # mimic the device sample rate
        return self._line()

    def reset_input_buffer(self):
        pass

    def close(self):
        self._closed = True


# --------------------------------------------------------------------------- #
# Port scanning
# --------------------------------------------------------------------------- #
def probe_port(port, baud=115200, seconds=1.2):
    """Open a port briefly and count valid 'D' lines. Returns (valid, parsed, seen)."""
    ser = serial.Serial(port, baud, timeout=0.3)
    seen = parsed = valid = 0
    try:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            line = ser.readline().decode('ascii', 'ignore').strip()
            if not line:
                continue
            seen += 1
            rec = parse_line(line)
            if rec is not None:
                parsed += 1
                if is_valid(rec):
                    valid += 1
    finally:
        ser.close()
    return valid, parsed, seen


def scan_ports(seconds=1.2):
    """Enumerate serial ports and probe each. Returns a list of
    (device, valid_count, label) sorted best-first, plus the simulate sentinel."""
    results = []
    if list_ports is not None:
        for p in list_ports.comports():
            try:
                valid, parsed, seen = probe_port(p.device, seconds=seconds)
            except Exception as exc:
                results.append((p.device, -1, f"{p.device}  (busy/err: {exc})"))
                continue
            if valid > 0:
                tag = f"master OK ({valid} valid)"
            elif parsed > 0:
                tag = "streaming, shank link down"
            elif seen > 0:
                tag = "data, not 'D' lines (firmware?)"
            else:
                tag = "no data"
            results.append((p.device, valid, f"{p.device}  [{tag}]"))
    results.sort(key=lambda r: r[1], reverse=True)
    results.append((SIM_PORT, 0, f"{SIM_PORT}  [synthetic demo source]"))
    return results


# --------------------------------------------------------------------------- #
# Acquisition thread: owns the serial link, runs the calibration state machine,
# and publishes the latest computed sample + phase for the UI and the sampler.
# --------------------------------------------------------------------------- #
class Collector(threading.Thread):
    def __init__(self, port, simulate=False, baud=115200,
                 cal_seconds=CAL_SECONDS, sweep_seconds=SWEEP_SECONDS):
        super().__init__(daemon=True)
        self.port = port
        self.simulate = simulate
        self.baud = baud
        self.cal_seconds = cal_seconds
        self.sweep_seconds = sweep_seconds

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._req_cal = threading.Event()
        self._req_stop_collecting = threading.Event()

        # calibration products
        self._d_thigh = self._d_shank = None
        self._f_thigh = self._f_shank = None

        self._state = {
            'phase': 'connecting',     # connecting|waiting|idle|zeroing|sweep|running
            'link_error': False,
            'link_msg': '',
            'phase_end': None,         # monotonic deadline for the countdown
            'cal_note': '',
            'sweep_preview': None,     # live shank tilt shown during the sweep
            'valid': False,
            'angle': None, 'incl_t': None, 'incl_s': None,
            'thigh_q': None, 'shank_q': None, 't_us': 0, 'rtt': 0,
            'recv_mono': 0.0,
        }

    # -- commands (thread-safe) --
    def request_calibration(self):
        self._req_cal.set()

    def stop_collecting(self):
        """End the active session: drop the calibration and return to idle."""
        self._req_stop_collecting.set()

    def stop(self):
        self._stop.set()

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)

    def _open(self):
        if self.simulate:
            return FakeSerial()
        if serial is None:
            raise RuntimeError("pyserial not installed. pip install pyserial")
        return serial.Serial(self.port, self.baud, timeout=1)

    def run(self):
        try:
            ser = self._open()
        except Exception as exc:
            self._set(phase='waiting', link_error=True,
                      link_msg=f"Cannot open {self.port}: {exc}")
            return
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        # rolling "last time we saw X" stamps drive the link diagnosis
        last_seen = last_parsed = last_valid = time.monotonic()
        last_raw = ''
        got_valid = 0

        gz_t = []; gz_s = []      # gravity-in-board during zeroing
        gs_t = []; gs_s = []      # gravity-in-board during sweep

        self._set(phase='waiting', link_msg=f"Waiting for data on {self.port} ...")

        while not self._stop.is_set():
            try:
                raw = ser.readline().decode('ascii', 'ignore').strip()
            except Exception as exc:
                self._set(link_error=True, link_msg=f"Serial read failed: {exc}")
                break

            now = time.monotonic()
            phase = self._state['phase']

            if raw:
                last_seen = now
                last_raw = raw
            rec = parse_line(raw) if raw else None
            if rec is not None:
                last_parsed = now
            valid = bool(rec) and is_valid(rec)
            if valid:
                last_valid = now
                got_valid += 1

            # leave the waiting phase once the stream is proven healthy
            if phase in ('connecting', 'waiting'):
                if got_valid >= 5:
                    phase = 'idle'
                    self._set(phase='idle', link_error=False, link_msg='')

            # stop-collecting request: reset to idle and discard calibration, so
            # the next collection starts a fresh, recalibrated session.
            if self._req_stop_collecting.is_set():
                self._req_stop_collecting.clear()
                if phase in ('zeroing', 'sweep', 'running'):
                    gz_t.clear(); gz_s.clear(); gs_t.clear(); gs_s.clear()
                    self._d_thigh = self._d_shank = None
                    self._f_thigh = self._f_shank = None
                    phase = 'idle'
                    self._set(phase='idle', cal_note='', sweep_preview=None,
                              phase_end=None, angle=None, incl_t=None, incl_s=None)

            # calibration (re)starts a session from idle. Each press is a reset:
            # a fresh zero + sweep, and (in the sampler) a new CSV and cleared plot.
            if self._req_cal.is_set() and phase == 'idle':
                self._req_cal.clear()
                gz_t.clear(); gz_s.clear(); gs_t.clear(); gs_s.clear()
                phase = 'zeroing'
                self._set(phase='zeroing', cal_note='', sweep_preview=None,
                          phase_end=now + self.cal_seconds)

            gt = gs = None
            if valid:
                gt = gravity_in_board(rec['thigh_q'])
                gs = gravity_in_board(rec['shank_q'])

            angle = incl_t = incl_s = None
            sweep_preview = None

            if phase == 'zeroing':
                if valid:
                    gz_t.append(gt); gz_s.append(gs)
                if now >= self._state['phase_end']:
                    self._d_thigh = average_gravity(gz_t)
                    self._d_shank = average_gravity(gz_s)
                    phase = 'sweep'
                    note = '' if gz_s else 'no valid samples during hold'
                    self._set(phase='sweep', cal_note=note,
                              phase_end=now + self.sweep_seconds)

            elif phase == 'sweep':
                if valid:
                    gs_t.append(gt); gs_s.append(gs)
                    if self._d_shank is not None:
                        sweep_preview = _incl_from_zero(gs, self._d_shank)
                if now >= self._state['phase_end']:
                    self._f_thigh = estimate_forward(gs_t, self._d_thigh)
                    self._f_shank = estimate_forward(gs_s, self._d_shank)
                    if self._f_shank is None:
                        note = "shank barely moved during calibration -- redo with a fuller range"
                    elif self._f_thigh is None:
                        note = "thigh didn't tilt; treating it as fixed (knee = shank tilt)"
                    else:
                        note = "calibrated (gravity-referenced, drift-free)"
                    phase = 'running'
                    self._set(phase='running', cal_note=note, phase_end=None)

            elif phase == 'running':
                if valid:
                    angle = gravity_knee_angle(rec['thigh_q'], rec['shank_q'],
                                               self._d_thigh, self._f_thigh,
                                               self._d_shank, self._f_shank)
                    incl_t = sagittal_inclination(gt, self._d_thigh, self._f_thigh)
                    incl_s = sagittal_inclination(gs, self._d_shank, self._f_shank)

            # link diagnosis: choose the message by which stage of the chain
            # went quiet, so a mid-session dropout names the real fault.
            link_error = False
            link_msg = ''
            if phase != 'connecting':
                if now - last_seen > 1.0:
                    link_error = True
                    link_msg = diagnose_stream(0, 0, 0, last_raw, self.port)
                elif now - last_parsed > 1.0:
                    link_error = True
                    link_msg = diagnose_stream(1, 0, 0, last_raw, self.port)
                elif now - last_valid > LINK_ERROR_SEC:
                    link_error = True
                    link_msg = diagnose_stream(1, 1, 0, last_raw, self.port)

            self._set(
                phase=phase, link_error=link_error, link_msg=link_msg,
                sweep_preview=sweep_preview, valid=valid,
                angle=angle, incl_t=incl_t, incl_s=incl_s,
                thigh_q=rec['thigh_q'] if rec else None,
                shank_q=rec['shank_q'] if (rec and valid) else None,
                t_us=rec['t_thigh'] if rec else 0,
                rtt=rec['rtt'] if rec else 0,
                recv_mono=now if valid else self._state['recv_mono'],
            )

        try:
            ser.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Sampler thread: snapshots the collector onto the fixed 100 Hz grid, applies the
# fill/gap gate, appends to the plot ring buffer, and writes the CSV.
#
# Collection is SESSION-based and driven by the collector's phase: a session runs
# for as long as the phase is zeroing/sweep/running. On the transition into a
# session it opens a fresh timestamped CSV and clears the plot; on the transition
# out (a Stop, which resets to idle) it closes the file. Outside a session it
# does nothing -- so when stopped the display FREEZES on the last data instead of
# rolling, and re-collecting is a clean reset with its own file.
# --------------------------------------------------------------------------- #
class Sampler(threading.Thread):
    HEADER = ['t_wall_iso', 't_session_s', 't_thigh_us',
              'thigh_qw', 'thigh_qx', 'thigh_qy', 'thigh_qz',
              'shank_qw', 'shank_qx', 'shank_qy', 'shank_qz',
              'knee_angle_deg', 'incl_thigh_deg', 'incl_shank_deg',
              'status', 'phase', 'rtt_us', 'fill_mode']
    ACTIVE = ('zeroing', 'sweep', 'running')

    def __init__(self, collector, path_prefix='knee'):
        super().__init__(daemon=True)
        self.collector = collector
        self.path_prefix = path_prefix
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.fill_mode = True
        self.gate = SampleGate(fill_mode=True)
        self.buf = deque(maxlen=PLOT_N)   # bounded ring for the live rolling window
        # Full-session history (unbounded, one session) so the frozen view after
        # Stop can show the ENTIRE run, not just the last rolling window.
        self.full = {'t': [], 'angle': [], 'incl_t': [], 'incl_s': [], 'rtt': []}
        self.current_path = None          # CSV of the session in progress (or last)

    def stop(self):
        self._stop.set()

    def set_fill_mode(self, fill):
        with self._lock:
            self.fill_mode = fill
            self.gate.fill_mode = fill

    def get_plot_arrays(self):
        """Live rolling window (bounded)."""
        with self._lock:
            t = [r['t'] for r in self.buf]
            ang = [r['angle'] for r in self.buf]
            it = [r['incl_t'] for r in self.buf]
            is_ = [r['incl_s'] for r in self.buf]
            rtt = [r['rtt'] for r in self.buf]
        return t, ang, it, is_, rtt

    def get_full_arrays(self):
        """The entire session so far (for the frozen full-run view after Stop)."""
        with self._lock:
            fu = self.full
            return (list(fu['t']), list(fu['angle']), list(fu['incl_t']),
                    list(fu['incl_s']), list(fu['rtt']))

    def valid_pct(self):
        with self._lock:
            return self.gate.valid_pct()

    def _open_session(self):
        path = datetime.now().strftime(self.path_prefix + "_%Y%m%d_%H%M%S.csv")
        f = open(path, 'w', newline='')
        writer = csv.writer(f)
        writer.writerow(self.HEADER)
        f.flush()
        with self._lock:
            self.buf.clear()                       # fresh plot for the new session
            for v in self.full.values():
                v.clear()                          # fresh full-session history
            self.gate = SampleGate(fill_mode=self.fill_mode)   # fresh valid% stats
            self.current_path = path
        return f, writer

    def run(self):
        nan = float('nan')
        f = writer = None
        active = False
        session_start = 0.0
        next_t = time.monotonic()
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                s = self.collector.snapshot()
                phase = s['phase']
                is_active = phase in self.ACTIVE

                if is_active and not active:          # session begins
                    f, writer = self._open_session()
                    session_start = now
                    active = True
                elif not is_active and active:        # session ends (Stop -> idle)
                    f.flush(); f.close(); f = writer = None
                    active = False

                if active:
                    t_rel = now - session_start
                    with self._lock:
                        if phase == 'running':
                            fresh = s['valid'] and (now - s['recv_mono'] < STALE_SEC)
                            out_angle, status = self.gate.process(
                                fresh, s['angle'] if fresh else None)
                            incl_t = s['incl_t'] if (fresh and s['incl_t'] is not None) else nan
                            incl_s = s['incl_s'] if (fresh and s['incl_s'] is not None) else nan
                        else:                          # zeroing / sweep: no angle yet
                            out_angle, status = None, phase
                            incl_t = incl_s = nan
                        plot_angle = out_angle if out_angle is not None else nan
                        self.buf.append({
                            't': t_rel, 'angle': plot_angle,
                            'incl_t': incl_t, 'incl_s': incl_s, 'rtt': s['rtt'],
                        })
                        self.full['t'].append(t_rel)
                        self.full['angle'].append(plot_angle)
                        self.full['incl_t'].append(incl_t)
                        self.full['incl_s'].append(incl_s)
                        self.full['rtt'].append(s['rtt'])
                        fill_mode = self.gate.fill_mode

                    tq = s['thigh_q']; sq = s['shank_q']
                    tq_cols = [f"{c:.4f}" for c in tq] if tq else ['', '', '', '']
                    sq_cols = [f"{c:.4f}" for c in sq] if sq else ['', '', '', '']
                    writer.writerow([
                        datetime.now().isoformat(timespec='milliseconds'),
                        f"{t_rel:.3f}", s['t_us'],
                        *tq_cols, *sq_cols,
                        f"{out_angle:.2f}" if out_angle is not None else '',
                        f"{incl_t:.2f}" if incl_t == incl_t else '',
                        f"{incl_s:.2f}" if incl_s == incl_s else '',
                        status, phase, s['rtt'], int(fill_mode),
                    ])

                next_t += GRID_DT
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.monotonic()   # fell behind; resync the grid
        finally:
            if f is not None:
                f.flush(); f.close()


# --------------------------------------------------------------------------- #
# Tkinter application. Imports of tkinter/matplotlib are deferred to run_gui()
# so --selftest stays headless.
# --------------------------------------------------------------------------- #
class App:
    PHASE_STYLE = {
        'connecting': ('#607d8b', 'Connecting ...'),
        'waiting':    ('#607d8b', 'Waiting for data'),
        'idle':       ('#455a64', 'Streaming OK -- press Calibrate & collect to begin'),
        'zeroing':    ('#f9a825', 'CALIBRATING - HOLD STILL (straight leg)'),
        'sweep':      ('#f9a825', 'CALIBRATING - SWEEP (bend knee + hip)'),
        'running':    ('#2e7d32', 'RUNNING - logging at 100 Hz'),
    }

    def __init__(self, tk, ttk, filedialog, messagebox, Figure, FigureCanvasTkAgg,
                 NavigationToolbar2Tk, initial_simulate=False):
        self.tk = tk; self.ttk = ttk
        self.filedialog = filedialog; self.messagebox = messagebox
        self.collector = None
        self.sampler = None
        self._last_phase = None
        self._stopped_fitted = False   # did we already fit-all the frozen view?

        self.root = tk.Tk()
        self.root.title("Knee Goniometer -- live collection")
        self.root.geometry("1000x800")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_toolbar()
        self._build_banner()
        self._build_plots(Figure, FigureCanvasTkAgg, NavigationToolbar2Tk)
        self._build_controls()

        if initial_simulate:
            self.port_var.set(SIM_PORT)
            self.port_combo['values'] = [SIM_PORT]
        else:
            self._rescan()

        self._tick()   # start the ~30 Hz UI refresh loop

    # -- widget construction --
    def _build_toolbar(self):
        tk, ttk = self.tk, self.ttk
        bar = ttk.Frame(self.root, padding=6)
        bar.pack(fill='x')
        ttk.Label(bar, text="Port:").pack(side='left')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var,
                                       width=42, state='readonly')
        self.port_combo.pack(side='left', padx=4)
        self.scan_btn = ttk.Button(bar, text="Rescan", command=self._rescan)
        self.scan_btn.pack(side='left', padx=2)
        self.connect_btn = ttk.Button(bar, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side='left', padx=6)
        self.health_var = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.health_var).pack(side='right')

    def _build_banner(self):
        self.banner = self.tk.Label(self.root, text="", font=("TkDefaultFont", 16, "bold"),
                                    fg="white", bg="#607d8b", pady=12)
        self.banner.pack(fill='x')

    def _build_plots(self, Figure, FigureCanvasTkAgg, NavigationToolbar2Tk):
        self.fig = Figure(figsize=(9, 5.2), dpi=100)
        self.ax_knee, self.ax_incl, self.ax_rtt = self.fig.subplots(
            3, 1, sharex=True, gridspec_kw={'height_ratios': [3, 2, 1]})
        self.fig.subplots_adjust(hspace=0.35, left=0.09, right=0.98, top=0.96, bottom=0.08)

        (self.line_knee,) = self.ax_knee.plot([], [], color='#1565c0', lw=1.6)
        self.ax_knee.set_ylabel("knee (deg)")
        self.ax_knee.grid(alpha=0.3)

        (self.line_it,) = self.ax_incl.plot([], [], color='#6a1b9a', lw=1.2, label='thigh')
        (self.line_is,) = self.ax_incl.plot([], [], color='#00838f', lw=1.2, label='shank')
        self.ax_incl.set_ylabel("incl (deg)")
        self.ax_incl.legend(loc='upper right', fontsize=8)
        self.ax_incl.grid(alpha=0.3)

        (self.line_rtt,) = self.ax_rtt.plot([], [], color='#546e7a', lw=1.0)
        self.ax_rtt.set_ylabel("rtt (us)")
        self.ax_rtt.set_xlabel("time (s)")
        self.ax_rtt.grid(alpha=0.3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=6, pady=4)

        # Pan/zoom/home/save toolbar. It is most useful once collecting stops:
        # the live loop stops driving the axes then, so pan and zoom stick.
        tb_frame = self.ttk.Frame(self.root)
        tb_frame.pack(fill='x')
        self.nav = NavigationToolbar2Tk(self.canvas, tb_frame, pack_toolbar=False)
        self.nav.update()
        self.nav.pack(side='left')

    def _build_controls(self):
        tk, ttk = self.tk, self.ttk
        ctl = ttk.Frame(self.root, padding=6)
        ctl.pack(fill='x')
        self.cal_btn = ttk.Button(ctl, text="Calibrate & collect", command=self._calibrate,
                                  state='disabled')
        self.cal_btn.pack(side='left')
        self.stop_btn = ttk.Button(ctl, text="Stop collecting", command=self._stop_collecting,
                                   state='disabled')
        self.stop_btn.pack(side='left', padx=6)

        self.mode_var = tk.StringVar(value='fill')
        mode = ttk.LabelFrame(ctl, text="Dropouts", padding=4)
        mode.pack(side='left', padx=12)
        ttk.Radiobutton(mode, text="Fill (line)", value='fill', variable=self.mode_var,
                        command=self._apply_mode).pack(side='left')
        ttk.Radiobutton(mode, text="Gap (points)", value='gap', variable=self.mode_var,
                        command=self._apply_mode).pack(side='left')

        self.save_btn = ttk.Button(ctl, text="Save copy...", command=self._save_copy,
                                   state='disabled')
        self.save_btn.pack(side='left', padx=6)

        self.stats_var = tk.StringVar(value="")
        ttk.Label(ctl, textvariable=self.stats_var).pack(side='right')

    # -- port scanning --
    def _rescan(self):
        self.scan_btn.config(state='disabled')
        self.health_var.set("scanning ports ...")

        def work():
            try:
                results = scan_ports()
            except Exception as exc:
                results = [(SIM_PORT, 0, f"{SIM_PORT}  [scan error: {exc}]")]
            self.root.after(0, lambda: self._scan_done(results))

        threading.Thread(target=work, daemon=True).start()

    def _scan_done(self, results):
        labels = [r[2] for r in results]
        self._scan_map = {r[2]: r[0] for r in results}
        self.port_combo['values'] = labels
        if labels:
            self.port_var.set(labels[0])
        self.scan_btn.config(state='normal')
        self.health_var.set("")

    def _selected_device(self):
        label = self.port_var.get()
        if hasattr(self, '_scan_map') and label in self._scan_map:
            return self._scan_map[label]
        return label

    # -- connect / disconnect --
    def _toggle_connect(self):
        if self.collector is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        device = self._selected_device()
        if not device:
            self.messagebox.showwarning("No port", "Pick a port first (or Rescan).")
            return
        simulate = (device == SIM_PORT)
        self.collector = Collector(device, simulate=simulate)
        self.sampler = Sampler(self.collector)     # opens a CSV per collection
        self.collector.start()
        self.sampler.start()
        self._apply_mode()
        self._last_phase = None                    # force a button-state refresh

        self.connect_btn.config(text="Disconnect")
        self.scan_btn.config(state='disabled')
        self.port_combo.config(state='disabled')
        self.save_btn.config(state='normal')

    def _disconnect(self):
        if self.sampler:
            self.sampler.stop()
        if self.collector:
            self.collector.stop()
        self.collector = self.sampler = None
        self.connect_btn.config(text="Connect")
        self.scan_btn.config(state='normal')
        self.port_combo.config(state='readonly')
        self.cal_btn.config(state='disabled')
        self.stop_btn.config(state='disabled')
        self.save_btn.config(state='disabled')
        self.health_var.set("")

    # -- controls --
    def _calibrate(self):
        if self.collector:
            self.collector.request_calibration()

    def _stop_collecting(self):
        if self.collector:
            self.collector.stop_collecting()

    def _apply_mode(self):
        fill = (self.mode_var.get() == 'fill')
        if self.sampler:
            self.sampler.set_fill_mode(fill)
        if fill:
            self.line_knee.set_linestyle('-'); self.line_knee.set_marker('')
        else:
            self.line_knee.set_linestyle('none'); self.line_knee.set_marker('.')
            self.line_knee.set_markersize(3)

    def _save_copy(self):
        path = self.sampler.current_path if self.sampler else None
        if not path:
            self.messagebox.showinfo("Nothing to save",
                                     "No collection yet -- press Calibrate & collect first.")
            return
        dst = self.filedialog.asksaveasfilename(
            defaultextension='.csv', initialfile=path,
            filetypes=[('CSV', '*.csv')])
        if not dst:
            return
        import shutil
        try:
            shutil.copy(path, dst)
            self.messagebox.showinfo("Saved", f"Copied to\n{dst}")
        except Exception as exc:
            self.messagebox.showerror("Save failed", str(exc))

    # -- periodic UI refresh --
    def _tick(self):
        if self.collector is not None:
            s = self.collector.snapshot()
            self._update_buttons(s['phase'])
            self._update_banner(s)
            self._update_plots(s['phase'])
        self.root.after(33, self._tick)

    def _update_buttons(self, phase):
        """Calibrate is available only from idle; Stop only while collecting.
        Reacts to the actual phase, so auto-transitions keep the buttons honest."""
        if phase == getattr(self, '_last_phase', None):
            return
        self._last_phase = phase
        active = phase in Sampler.ACTIVE
        self.cal_btn.config(state='normal' if phase == 'idle' else 'disabled')
        self.stop_btn.config(state='normal' if active else 'disabled')
        path = self.sampler.current_path if self.sampler else None
        if active and path:
            self.health_var.set(f"logging -> {path}")
        elif phase == 'idle':
            self.health_var.set("idle -- ready to calibrate & collect")

    def _update_banner(self, s):
        phase = s['phase']
        if s['link_error']:
            self.banner.config(bg='#c62828', text="LINK ERROR: " + (s['link_msg'] or ''))
        else:
            color, text = self.PHASE_STYLE.get(phase, ('#607d8b', phase))
            if phase in ('zeroing', 'sweep') and s['phase_end'] is not None:
                remain = max(0.0, s['phase_end'] - time.monotonic())
                text = f"{text}   {remain:0.1f}s"
                if phase == 'sweep' and s['sweep_preview'] is not None:
                    text += f"   (shank tilt {s['sweep_preview']:0.0f} deg)"
            elif phase == 'running' and s['cal_note']:
                text = f"{text}   -   {s['cal_note']}"
            self.banner.config(bg=color, text=text)

        # right-hand health readout
        pct = self.sampler.valid_pct() if self.sampler else None
        bits = []
        if s['rtt']:
            bits.append(f"rtt {s['rtt']} us")
        if pct is not None:
            bits.append(f"valid {pct:0.1f}%")
        self.stats_var.set("   ".join(bits))

    def _update_plots(self, phase):
        if not self.sampler:
            return
        active = phase in Sampler.ACTIVE

        if not active:
            # Collection stopped: freeze. Do a single "fit the whole session"
            # autoscale, then stop touching the axes so the toolbar's pan/zoom
            # (and Home) stay put and the trace can be inspected freely.
            if not self._stopped_fitted:
                t, ang, it, is_, rtt = self.sampler.get_full_arrays()
                if t:
                    self.line_knee.set_data(t, ang)
                    self.line_it.set_data(t, it)
                    self.line_is.set_data(t, is_)
                    self.line_rtt.set_data(t, rtt)
                    for ax in (self.ax_knee, self.ax_incl, self.ax_rtt):
                        ax.set_xlim(t[0], t[-1] if t[-1] > t[0] else t[0] + 1)
                        ax.relim(); ax.autoscale_view()
                    self.canvas.draw()
                    self.nav.update()   # make this fitted view the toolbar's Home
                self._stopped_fitted = True
            return

        # Actively collecting: live rolling window with y-autoscale.
        self._stopped_fitted = False
        t, ang, it, is_, rtt = self.sampler.get_plot_arrays()
        if not t:
            return
        self.line_knee.set_data(t, ang)
        self.line_it.set_data(t, it)
        self.line_is.set_data(t, is_)
        self.line_rtt.set_data(t, rtt)
        t_max = t[-1]
        t_min = max(0.0, t_max - PLOT_WINDOW_SEC)
        for ax in (self.ax_knee, self.ax_incl, self.ax_rtt):
            ax.set_xlim(t_min, t_max if t_max > t_min else t_min + 1)
            ax.relim(); ax.autoscale_view(scalex=False, scaley=True)
        self.canvas.draw_idle()

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def run_gui(simulate=False):
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import (
        FigureCanvasTkAgg, NavigationToolbar2Tk)
    App(tk, ttk, filedialog, messagebox, Figure, FigureCanvasTkAgg,
        NavigationToolbar2Tk, initial_simulate=simulate).run()


# --------------------------------------------------------------------------- #
# Headless self-test: the sample-gate fill/gap logic and the synthetic source.
# --------------------------------------------------------------------------- #
def _self_test():
    print("Running knee_gui self-test...")

    # FILL: forward-fill up to max_fill, then missing; recovers on next valid.
    g = SampleGate(max_fill=3, fill_mode=True)
    assert g.process(True, 10.0) == (10.0, 'valid')
    assert g.process(False, None) == (10.0, 'filled')
    assert g.process(False, None) == (10.0, 'filled')
    assert g.process(False, None) == (10.0, 'filled')
    assert g.process(False, None) == (None, 'missing')
    assert g.process(True, 20.0) == (20.0, 'valid')
    print("  fill-mode gate OK")

    # GAP: never fills; every invalid tick is a discrete miss.
    g2 = SampleGate(max_fill=3, fill_mode=False)
    assert g2.process(True, 5.0) == (5.0, 'valid')
    assert g2.process(False, None) == (None, 'missing')
    assert g2.process(False, None) == (None, 'missing')
    assert g2.process(True, 6.0) == (6.0, 'valid')
    assert abs(g2.valid_pct() - 50.0) < 1e-9
    print("  gap-mode gate OK")

    # Switching to gap mid-run stops filling immediately.
    g3 = SampleGate(max_fill=5, fill_mode=True)
    g3.process(True, 1.0)
    assert g3.process(False, None) == (1.0, 'filled')
    g3.fill_mode = False
    assert g3.process(False, None) == (None, 'missing')
    print("  live mode switch OK")

    # Synthetic source emits parseable, mostly-valid 18-field 'D' lines.
    fs = FakeSerial()
    seen = valid = 0
    for _ in range(30):
        rec = parse_line(fs.readline().decode('ascii', 'ignore'))
        if rec is not None:
            seen += 1
            if is_valid(rec):
                valid += 1
    fs.close()
    assert seen == 30, seen
    assert valid > 0
    print(f"  synthetic source OK ({valid}/{seen} valid)")

    print("knee_gui self-test PASSED.\n")


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description="Knee goniometer live GUI")
    ap.add_argument('--simulate', action='store_true',
                    help='start with the synthetic source preselected (no board)')
    ap.add_argument('--selftest', action='store_true',
                    help='run the headless sample-gate self-test and exit')
    args = ap.parse_args()

    if args.selftest:
        _self_test()
    else:
        run_gui(simulate=args.simulate)
