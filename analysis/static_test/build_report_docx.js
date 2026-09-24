// Builds Static_Stability_Test_Report.docx from the same content as REPORT.md.
// Usage: npm install docx && node build_report_docx.js Static_Stability_Test_Report.docx
// (run plot_report.py first so report_stability.png is up to date).
const fs = require('fs');
const {
  Document, Packer, Paragraph, TextRun, HeadingLevel, Table, TableRow, TableCell,
  WidthType, ShadingType, ImageRun, AlignmentType, LevelFormat, BorderStyle,
  Footer, PageNumber, TableLayoutType,
} = require('docx');

const SRC = __dirname + '/';
const OUT = process.argv[2];
const FONT = 'Calibri', MONO = 'Consolas';
const INK = '1F1F1F', MUTED = '5A5A5A', ACCENT = '1F4E79', HEAD_FILL = 'DCE6F1', RULE = 'BFBFBF';
const PAGE_W = 12240, MARGIN = 1080, CONTENT_W = PAGE_W - 2 * MARGIN;   // 0.75" margins

// inline markup: **bold**, `code`, *italic*
function runs(text, base = {}) {
  const out = [];
  const re = /(\*\*[^*]+\*\*|`[^`]+`|\*[^*]+\*)/g;
  let last = 0, m;
  while ((m = re.exec(text))) {
    if (m.index > last) out.push(new TextRun({ text: text.slice(last, m.index), ...base }));
    const t = m[0];
    if (t.startsWith('**')) out.push(new TextRun({ text: t.slice(2, -2), bold: true, ...base }));
    else if (t.startsWith('`')) out.push(new TextRun({ text: t.slice(1, -1), font: MONO, size: 19, ...base }));
    else out.push(new TextRun({ text: t.slice(1, -1), italics: true, ...base }));
    last = m.index + t.length;
  }
  if (last < text.length) out.push(new TextRun({ text: text.slice(last), ...base }));
  return out;
}
const P = (text, opts = {}) => new Paragraph({ children: runs(text, opts.run), spacing: { after: 120 }, ...opts.para });
const H1 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun(t)] });
const H2 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun(t)] });
const bullet = (t, level = 0) => new Paragraph({ numbering: { reference: 'bullets', level }, children: runs(t), spacing: { after: 60 } });
let numRef = 0;
const numbered = (items) => { const ref = `num${numRef++}`; numberingConfigs.push(numCfg(ref));
  return items.map((t) => new Paragraph({ numbering: { reference: ref, level: 0 }, children: runs(t), spacing: { after: 60 } })); };
const numberingConfigs = [];
const numCfg = (ref) => ({ reference: ref, levels: [{ level: 0, format: LevelFormat.DECIMAL, text: '%1.', alignment: AlignmentType.LEFT,
  style: { paragraph: { indent: { left: 360, hanging: 360 } } } }] });
const caption = (t) => new Paragraph({ children: runs(t, { italics: true, color: MUTED, size: 18 }), spacing: { after: 200 } });

function table(header, rows, widths) {
  const total = widths.reduce((a, b) => a + b, 0);
  const cell = (text, isHead, w) => new TableCell({
    width: { size: w, type: WidthType.DXA },
    shading: isHead ? { type: ShadingType.CLEAR, color: 'auto', fill: HEAD_FILL } : undefined,
    margins: { top: 60, bottom: 60, left: 100, right: 100 },
    children: [new Paragraph({ children: runs(text, isHead ? { bold: true, size: 19 } : { size: 19 }) })],
  });
  return new Table({
    width: { size: total, type: WidthType.DXA }, columnWidths: widths, layout: TableLayoutType.FIXED,
    borders: { top: { style: BorderStyle.SINGLE, size: 4, color: RULE }, bottom: { style: BorderStyle.SINGLE, size: 4, color: RULE },
      left: { style: BorderStyle.SINGLE, size: 4, color: RULE }, right: { style: BorderStyle.SINGLE, size: 4, color: RULE },
      insideHorizontal: { style: BorderStyle.SINGLE, size: 4, color: RULE }, insideVertical: { style: BorderStyle.SINGLE, size: 4, color: RULE } },
    rows: [new TableRow({ tableHeader: true, children: header.map((h, i) => cell(h, true, widths[i])) }),
      ...rows.map((r) => new TableRow({ cantSplit: true, children: r.map((c, i) => cell(c, false, widths[i])) }))],
  });
}
const gap = () => new Paragraph({ spacing: { after: 120 }, children: [] });
const W = (...fr) => { const s = fr.reduce((a, b) => a + b, 0); const w = fr.map((f) => Math.floor(CONTENT_W * f / s)); w[w.length - 1] += CONTENT_W - w.reduce((a, b) => a + b, 0); return w; };

const img = fs.readFileSync(SRC + 'report_stability.png');
const IMG_W = 672, IMG_H = Math.round(672 * 494 / 1430);   // keep the PNG's aspect ratio

const body = [
  new Paragraph({ heading: HeadingLevel.TITLE, children: [new TextRun('Dual-IMU Goniometer')] }),
  new Paragraph({ children: [new TextRun({ text: 'Static Test Report', size: 32, color: ACCENT })], spacing: { after: 160 } }),
  P('**Test dates:** 2026-09-23 and 2026-09-24   ·   **Hardware:** 2 × Arduino Nano 33 BLE Rev2 (BMI270), wired UART link   ·   **Logger:** `knee_gui.py`', { run: { size: 19, color: MUTED } }),
  new Paragraph({ border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: ACCENT, space: 4 } }, spacing: { after: 200 }, children: [] }),

  H1('Summary'),
  P('With nothing moving, the goniometer\'s reading stays still. Jitter is about **0.01–0.02°**. On a stable setup the reading changed by **less than 0.1° over 5 minutes, and by about 0.06° over a full hour** (drift −0.001 °/min).'),
  P('A preliminary check on 45° and 90° rigs read **45.0° and 89.0°**. That\'s within 1° of nominal, but the setup couldn\'t give a rigorous accuracy figure (see below).'),
  P('Testing also found three problems:'),
  ...numbered([
    '**The sample rate was ~40 Hz, not 50 Hz,** and ~20 % of CSV rows were duplicates. **Fixed and confirmed:** true 50 Hz, 0 duplicate rows. The static results were unaffected.',
    '**One board\'s sensor fails after 20–47 minutes of running.** The fault followed that physical board when the boards swapped roles; the other board ran a full hour without a single bad reading. **Action: replace that board.**',
    '**Only one node was exercised** by these tests, because of how calibration works when one node never tilts.',
  ]),

  H1('Purpose'),
  bullet('**Stability:** if nothing moves, does the measured angle also stay unchanged?'),
  bullet('**Preliminary angle check:** do fixed 45° and 90° rigs read close to nominal?'),
  P('The rigs weren\'t precise enough for a formal accuracy figure, so that part is a preliminary inspection only.', { para: { spacing: { before: 80, after: 120 } } }),

  H1('Setup'),
  table(['', 'Session 1 (runs 1–3)', 'Session 2 (runs 4–6)', 'Long runs (3 runs)', 'Angle rig (45°, 90°)'], [
    ['Date', '2026-09-23', '2026-09-24', '2026-09-24', '2026-09-24'],
    ['Thigh node', 'Taped to the desk', 'Taped to a raised rig', 'Taped to the rig', 'On the flat upper segment'],
    ['Shank node', 'Taped to a ruler', 'Taped to a ruler, ~1 cm lower, ~5 cm apart', 'Taped to a ruler', 'On the angled lower segment'],
    ['Duration', '~5 min per run', '~5–5.5 min per run', '25–65 min, unattended', '~20–25 s hold each'],
  ], W(1, 1.3, 1.6, 1.3, 1.4)),
  gap(),
  P('All nodes were powered over USB.'),
  P('**Procedure (every run):**', { para: { spacing: { after: 60 } } }),
  ...numbered([
    '**Zeroing, 2 s:** both nodes held still. This pose becomes 0°.',
    '**Calibration sweep, 6 s:** the shank node rotated and returned, so the software learns the direction it bends in.',
    '**Hold:** nothing touched. For the angle rig, the shank segment was set at the rig angle and held.',
  ]),
  P('The first 20 s (calibration plus setting the node down) are excluded from the stability analysis.', { para: { spacing: { before: 80, after: 120 } } }),

  H1('Results'),
  H2('The reading stays still when nothing moves'),
  new Paragraph({ alignment: AlignmentType.CENTER, children: [new ImageRun({ type: 'png', data: img, transformation: { width: IMG_W, height: IMG_H },
    altText: { title: 'Change in angle during each hold', description: 'Session 1 runs stay within 0.06 degrees; session 2 runs creep 0.1 to 0.5 degrees, run 5 with two steps.', name: 'stability' } })] }),
  caption('Change in the measured angle during each hold (5 s rolling average), relative to the first 10 s of the hold.'),
  table(['Run', 'Resting angle', 'Jitter (SD)', 'Change over the hold', 'Drift'], [
    ['1', '−0.30°', '0.015°', '−0.06°', '−0.005 °/min'],
    ['2', '−0.02°', '0.011°', '−0.03°', '−0.0003 °/min'],
    ['3', '+0.03°', '0.011°', '−0.01°', '+0.001 °/min'],
    ['4', '+6.38°', '0.016°*', '+0.16°', '+0.033 °/min'],
    ['5', '−1.27°', '0.047°*', '+0.49° (two steps)', '+0.082 °/min'],
    ['6', '−0.84°', '0.012°*', '+0.11°', '+0.019 °/min'],
    ['1-hour hold', '+0.23°', '0.018°', '~0.06° over 57 min', '−0.001 °/min'],
  ], W(1, 1.2, 1.1, 1.7, 1.3)),
  caption('*Session 2 SD with the slow trend removed, so it measures jitter rather than the creep.'),
  bullet('**Session 1 (flat desk):** the reading moved by 0.06° at most over 5 minutes, and the drift direction flips between runs. That\'s noise, not drift.'),
  bullet('**Session 2 (raised rig):** the reading crept by 0.1–0.5°, and run 5 has two sudden steps. The desk node, which isn\'t part of the angle calculation, moved at the same time, so the creep most likely comes from the rig or wires settling. In a later run, the raw accelerometer confirmed that a similar creep was the ruler physically tilting.'),
  bullet('**1-hour hold** (from the board-swap run, where the angle came from the healthy board): 57 minutes at 0.23° with 0.018° jitter. The 5 s averages stayed within a 0.1° range. This is the longest clean static record.'),
  bullet('**Resting angle** is where the node came to rest after the calibration sweep, relative to where it was zeroed. It reflects placement, and it stays constant during the hold.'),

  H2('Noise does not grow with time'),
  P('Allan deviation shows how much averages over a time window τ differ from one window to the next. If it stays flat or falls as τ grows, there\'s no drift at those time scales.'),
  table(['Averaging window τ', 'Session 1', 'Session 2'], [
    ['0.1 s', '0.003°', '0.003°'],
    ['1 s', '0.005°', '0.005–0.007°'],
    ['60 s', '0.004–0.007°', '— (rig creep dominates)'],
  ], W(1.4, 1.3, 1.8)),
  gap(),
  P('Session 1 is flat from 10 s to 60 s, and the 1-hour hold drifted only −0.001 °/min, so no sensor drift shows up at any time scale tested.'),

  H2('Preliminary angle check: 45° and 90° rigs'),
  table(['Rig', 'Reading (steady part)', 'Diff from nominal', 'Jitter (SD)', 'Raw-accelerometer check'], [
    ['45°', '45.0°', '0.0°', '0.034°', '45.2°'],
    ['90°', '89.0°', '1.0°', '0.093°', '89.0°'],
  ], W(0.7, 1.5, 1.3, 1.1, 1.6)),
  caption('The GUI shows these as negative angles because of the bending direction the calibration learned.'),
  bullet('**The shank node\'s tilt is measured correctly.** The raw accelerometer, which doesn\'t depend on the filter or the calibration, agrees with the reported angle within about 0.2°. So the calibration learned the right bending direction, and a 90° hold is as steady as a 0° hold.'),
  bullet('**Why this is preliminary:**'),
  bullet('**One placement each.**', 1),
  bullet('**The reference (thigh) node shifted 2.7° and 4.5° during placement,** probably pulled by its cable. The software ignores that node because it didn\'t tilt during calibration. If the upper segment stayed flat, as intended, the readings above stand. If the segment itself moved, the true angle between the segments could differ by up to ~2–3°.', 1),
  bullet('The data suggests the board shifted on its segment rather than the segment rotating, since it tilted about as much across the bending plane as along it. But this can\'t be confirmed.', 1),

  H2('Other checks'),
  bullet('**Heading drift doesn\'t affect the angle.** Each sensor\'s heading (compass direction) drifted by 0.2–5 °/min, as expected without a magnetometer, and the angle ignored it in every run. That\'s by design.'),
  bullet('**Calibration copes with an imperfect setup.** In session 2 the ruler node was zeroed 4–15° off level, and the sweep was shorter (~55°) and in the opposite direction. Calibration still worked and the reading was just as steady.'),
  bullet('**Link reliability:** 100 % valid samples in all six short runs. The long runs had no checksum failures while the boards were healthy, apart from 2 at start-up in one run.'),

  H1('Issues found'),
  H2('1. Sample rate was ~40 Hz, not 50 Hz — fixed'),
  table(['', 'Before', 'After the fixes'], [
    ['Central board\'s cycle', '~23 ms (~43.5 Hz)', '20.0 ms (50 Hz)'],
    ['Printing each data line', '~11.7 ms (~36 separate prints)', '~1.0–1.8 ms (one write)'],
    ['Unique samples in the CSV', '~40.5 per second', '~50 per second'],
    ['CSV rows repeating the previous sample', '19–20 %', '0 %'],
  ], W(2, 1.6, 1.4)),
  gap(),
  P('**Cause.** Two separate problems:', { para: { spacing: { after: 60 } } }),
  bullet('**The central board ran slow.** Each output cycle spent ~10.7 ms reading its IMU and ~11.7 ms printing the data line as ~36 separate blocking prints.'),
  bullet('**The GUI wrote on its own timer.** It wrote a row every 20 ms whatever had arrived, and on Windows that timer ticks unevenly, so rows repeated some samples and skipped others.'),
  P('**Fixes.**', { para: { spacing: { before: 80, after: 60 } } }),
  bullet('The central now builds each line in memory and sends it with one write.'),
  bullet('The GUI now writes exactly one row per sample received, timed by the board\'s own clock.'),
  P('Both were confirmed in the hour-long run: 50 rows/s, 0 duplicates, rows 19.9 ms apart.', { para: { spacing: { before: 80, after: 120 } } }),
  P('**Effect on these results: none.** With duplicates removed, every run\'s mean, SD and drift are identical to within 0.0002°. The fix matters for movement data.'),

  H2('2. One board\'s sensor fails after 20–47 minutes — traced to that board'),
  P('**What happens.** On that board, individual sensor reads start **hanging for ~190–200 ms** and returning garbage while the board sits still: accelerometer up to 4.4 g, gyro up to 2400 °/s. A few minutes later the sensor **stops producing data entirely**. Restarting the sensor, or rebooting the board, doesn\'t bring it back.'),
  table(['Recording', 'That board\'s role', 'Sensor trouble starts', 'Sensor dead'], [
    ['No-motion test', 'Shank (ruler)', 'before 26 min', '~26 min'],
    ['Long run 1', 'Shank', '20.4 min', '26.3 min'],
    ['Long run 2', 'Shank', '24.8 min (abrupt)', '24.8 min; still dead after the automatic reboot'],
    ['Long run 3, **roles swapped**', 'Thigh (central)', '6.5 min (missed updates); 37.8 min (hung reads)', '47.3 min'],
  ], W(1.4, 1.1, 1.7, 1.8)),
  gap(),
  P('In the swapped run, **the other board ran the shank role for over an hour with 0 bad readings.** Across all runs, the other board never had a sensor problem in either role.'),
  P('**Ruled out:**', { para: { spacing: { after: 60 } } }),
  bullet('**The link:** no checksum failures before the fault in any run (apart from 2 at start-up in one).'),
  bullet('**Power loss or a board reset:** its uptime counter never restarted by itself.'),
  bullet('**Calibration:** unrelated to how or whether the node moved.'),
  bullet('**The firmware:** both firmware roles failed on this board, and neither failed on the other.'),
  P('**Conclusion.** The fault follows that physical board. Its cable, USB port and position also stayed with it, so a one-off test with the other board\'s cable and port would exclude those. The board itself is the most likely cause: a faulty sensor chip, a weak joint, or heat. **Replace it.**', { para: { spacing: { before: 80, after: 120 } } }),
  P('**Firmware protection (in place).** The shank board now rejects implausible samples and sends them as gaps instead of wrong angles, resets its filter after each sensor restart, and reboots itself if its sensor produces nothing usable for 5 s. In long run 2 the guard caught the bad readings and the reboot fired, but the sensor stayed dead, which is why it points to hardware.'),

  H2('3. Only one node was exercised, and validity can hide a failure'),
  P('The angle is the difference between the two nodes\' tilts, but the software only uses a node\'s tilt if it moved at least 5° during the calibration sweep. The desk node never did, so it was treated as fixed and the angle came from the shank node alone.'),
  P('This also means **"valid %" only reflects the shank node\'s data.** In the swapped run, the failing board was the reference node: validity stayed ~100 % and the angle stayed flat while its sensor died. The failure only showed in the diagnostic log and the CSV\'s `thigh_ax..az` columns.'),

  H1('Limitations'),
  bullet('**Accuracy is preliminary only.** The 45°/90° check was one placement each, and the reference node shifted during placement.'),
  bullet('**Jitter is limited by the log format.** Angles are logged to 0.01°, so the ~0.01° jitter is an upper bound; the true sensor noise is probably lower.'),
  bullet('**Rig vs. sensor.** Where the rig moved (session 2), its movement couldn\'t be separated from sensor drift at the time. The GUI now logs the raw accelerometer, which does that; it confirmed a later creep was physical.'),
  bullet('**One node per test.** The reference node\'s behaviour within the angle calculation hasn\'t been tested.'),

  H1('Next steps'),
  table(['Step', 'Status'], [
    ['Fix the sample rate (one write per line; one CSV row per sample)', 'Done and confirmed'],
    ['Log the raw accelerometer in the CSV', 'Done'],
    ['Guard the shank node against a failing sensor', 'Done'],
    ['Board-swap test to locate the sensor fault', 'Done: fault follows one board'],
    ['Replace the faulty board (optionally, first test it on the other cable and port)', 'To do'],
    ['One-hour static run on two healthy boards as the clean baseline', 'After replacement'],
    ['Tilt both nodes during calibration, so the angle uses both', 'Next test'],
    ['Keep the reference node still (tape the board and its cable down)', 'Next test'],
    ['Repeat 0°/45°/90° placements 3–5 times each', 'If a steadier rig is available'],
    ['Log angles with more decimal places', 'To do'],
  ], W(3.4, 1.6)),
  gap(),
  P('Detailed analysis, per-run tables and method notes are in `analysis/static_test/FINDINGS.md` in the project repository.', { run: { color: MUTED, size: 19 } }),
];

const doc = new Document({
  creator: 'Dual-IMU Goniometer project', title: 'Static Test Report',
  styles: {
    default: { document: { run: { font: FONT, size: 21, color: INK } } },
    paragraphStyles: [
      { id: 'Title', name: 'Title', basedOn: 'Normal', run: { size: 44, bold: true, color: INK }, paragraph: { spacing: { after: 40 } } },
      { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 28, bold: true, color: ACCENT }, paragraph: { spacing: { before: 280, after: 120 }, keepNext: true, outlineLevel: 0 } },
      { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 23, bold: true, color: INK }, paragraph: { spacing: { before: 200, after: 100 }, keepNext: true, outlineLevel: 1 } },
    ],
  },
  numbering: { config: [
    { reference: 'bullets', levels: [
      { level: 0, format: LevelFormat.BULLET, text: '•', alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 360, hanging: 240 } } } },
      { level: 1, format: LevelFormat.BULLET, text: '–', alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 720, hanging: 240 } } } },
    ] },
    ...numberingConfigs,
  ] },
  sections: [{
    properties: { page: { size: { width: PAGE_W, height: 15840 }, margin: { top: MARGIN, bottom: MARGIN, left: MARGIN, right: MARGIN } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER,
      children: [new TextRun({ text: 'Static Test Report   ·   page ', size: 16, color: MUTED }), new TextRun({ children: [PageNumber.CURRENT], size: 16, color: MUTED })] })] }) },
    children: body,
  }],
});
Packer.toBuffer(doc).then((b) => { fs.writeFileSync(OUT, b); console.log('wrote', OUT); });
