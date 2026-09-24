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
  new Paragraph({ children: [new TextRun({ text: 'Static Stability Test Report', size: 32, color: ACCENT })], spacing: { after: 160 } }),
  P('**Test dates:** 2026-09-23 and 2026-09-24   ·   **Hardware:** 2 × Arduino Nano 33 BLE Rev2 (BMI270), wired UART link   ·   **Logger:** `knee_gui.py`', { run: { size: 19, color: MUTED } }),
  new Paragraph({ border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: ACCENT, space: 4 } }, spacing: { after: 200 }, children: [] }),

  H1('Summary'),
  P('With nothing moving, the goniometer\'s reading stays still. Jitter is about **0.01°**, and on a stable setup the reading changed by **less than 0.1° over 5 minutes**, with no detectable drift. Every sample in all six runs was valid.'),
  P('Testing also turned up three problems to fix before dynamic (movement) testing:'),
  ...numbered([
    '**The sample rate is ~40 Hz, not the configured 50 Hz**, and ~20 % of CSV rows are duplicates. The static results are unaffected.',
    '**The ruler-side node\'s sensor can go bad.** In one recording it reported spinning while completely still, and the valid-sample rate fell to 93 %.',
    '**Only one of the two sensors was exercised** by these tests, because of how calibration works when one node never tilts.',
  ]),

  H1('Purpose'),
  P('To check **stability, not accuracy**: if nothing moves, does the measured angle also stay unchanged? No reference angle was used, so absolute accuracy is out of scope.'),

  H1('Setup'),
  table(['', 'Session 1 (runs 1–3)', 'Session 2 (runs 4–6)'], [
    ['Date', '2026-09-23', '2026-09-24'],
    ['Rig', 'Both nodes on a flat desk', 'Raised rig: desk node ~1 cm higher, ~5 cm apart, roughly in line'],
    ['Thigh node', 'Taped to the desk', 'Taped to the rig'],
    ['Shank node', 'Taped to a ruler', 'Taped to a ruler'],
    ['Firmware', 'Original', 'Emit-timer change (no effect, see issue 1)'],
    ['Duration', '~5 min per run', '~5–5.5 min per run'],
  ], W(1.1, 2, 2.6)),
  gap(),
  P('**Procedure (every run):**', { para: { spacing: { after: 60 } } }),
  ...numbered([
    '**Zeroing, 2 s:** both nodes held still. This pose becomes 0°.',
    '**Calibration sweep, 6 s:** the ruler rotated ~90° and returned, so the software learns the direction it bends in.',
    '**Hold, ~5 min:** nothing touched.',
  ]),
  P('The first 20 s (calibration plus setting the ruler down) are excluded from the analysis.', { para: { spacing: { before: 80, after: 120 } } }),

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
  ], W(0.6, 1.3, 1.2, 1.7, 1.4)),
  caption('*Session 2 SD with the slow trend removed, so it measures jitter rather than the creep.'),
  bullet('**Session 1 (flat desk):** the reading moved by 0.06° at most over 5 minutes, and the drift direction flips between runs. That\'s noise, not drift.'),
  bullet('**Session 2 (raised rig):** the reading crept by 0.1–0.5°. Run 5 has two sudden steps. The desk node, which isn\'t part of the angle calculation, moved at the same time, so the creep most likely comes from the rig or wires settling rather than the sensor. Short-term jitter was the same as in session 1.'),
  bullet('**Resting angle** is where the ruler came to rest after the calibration sweep, relative to where it was zeroed. It reflects placement, and it stays constant during the hold.'),

  H2('Noise does not grow with time'),
  P('Allan deviation shows how much averages over a time window τ differ from one window to the next. If it stayed flat or fell as τ grew, there\'s no drift at those time scales.'),
  table(['Averaging window τ', 'Session 1', 'Session 2'], [
    ['0.1 s', '0.003°', '0.003°'],
    ['1 s', '0.005°', '0.005–0.007°'],
    ['60 s', '0.004–0.007°', '— (rig creep dominates)'],
  ], W(1.4, 1.3, 1.8)),
  gap(),
  P('Session 1 is flat from 10 s to 60 s, so no drift appears over these time scales.'),

  H2('Other checks'),
  bullet('**Heading drift doesn\'t affect the angle.** Each sensor\'s heading (compass direction) drifted by 0.2–5 °/min, as expected without a magnetometer, and the angle ignored it in both sessions. That\'s by design.'),
  bullet('**Calibration copes with an imperfect setup.** In session 2 the ruler node was zeroed 4–15° off level, and the sweep was shorter (~55°) and in the opposite direction. Calibration still worked and the reading was just as steady.'),
  bullet('**Link reliability:** 100 % valid samples in all six runs, no dropouts.'),

  H1('Issues found'),
  H2('1. Sample rate is ~40 Hz, not 50 Hz'),
  table(['', 'Rate'], [
    ['Configured', '50 Hz'],
    ['Produced by the central board', '~43.5 Hz (one sample every ~23 ms)'],
    ['Unique samples in the CSV', '~40.5 Hz'],
    ['CSV rows repeating the previous sample', '19–20 %'],
  ], W(2.2, 2.3)),
  gap(),
  P('**Cause.** The central board takes ~23 ms per output cycle instead of 20 ms. The GUI writes a row every 20 ms whatever it has received, so when no new sample has arrived it writes the previous one again. Repeated rows share the board\'s timestamp (`t_thigh_us`), which confirms they are copies of the same measurement.'),
  P('**Effect on these results: none.** With duplicates removed, every run\'s mean, SD and drift are identical to within 0.0002°. For movement data it would matter: repeated values, dropped samples, and row times up to ~20 ms off.'),
  P('**Status.**', { para: { spacing: { after: 60 } } }),
  bullet('A first firmware fix to the output timer had no effect, which showed the problem is inside each output cycle.'),
  bullet('A diagnostic build is ready. It times the parts of that cycle; the suspects are about 36 separate print calls per line and slow sensor reads.'),
  bullet('Planned fixes:'),
  bullet('Speed up the output cycle.', 1),
  bullet('Write one CSV row per received sample, so duplicates can\'t happen.', 1),

  H2('2. The ruler-side node\'s sensor can go bad'),
  P('In a separate 1-minute recording where nothing moved at all (the central board had been running ~26 minutes since its last reset, from its `t_thigh_us` clock):'),
  bullet('**The ruler node\'s reported orientation spun** by a median of ~27° between consecutive samples, up to ~2000 °/s. The desk node stayed within 0.05°.'),
  bullet('**The ruler node stalled 26 times**, each for ~0.2 s. That matches its built-in sensor-restart routine firing repeatedly without fixing the problem.'),
  bullet('**Valid samples fell to 93 %.** The rest were gaps filled with the previous value.'),
  P('This is consistent with the drop in data quality seen after ~15 minutes of collection. It isn\'t caused by calibration or by the sample-rate issue. The diagnostic build reports the ruler node\'s sensor health once a second, to show when and how it fails.', { para: { spacing: { before: 80, after: 120 } } }),

  H2('3. Only the ruler node was tested'),
  P('The angle is the difference between the two nodes\' tilts, but the software only uses a node\'s tilt if it moved at least 5° during the calibration sweep. The desk node never did, so it was treated as fixed and the angle came from the ruler node alone. Its own data shows it was just as stable (jitter ~0.01°, no drift), but a future test should tilt both nodes during calibration.'),

  H1('Limitations'),
  bullet('**Stability only, not accuracy.** There was no reference angle, and resting angles were at most 6.4°, too small to reveal scale errors.'),
  bullet('**Jitter is limited by the log format.** Angles are logged to 0.01°, so the ~0.01° jitter is an upper bound; the true sensor noise is probably lower.'),
  bullet('**5-minute holds.** Drift over longer periods isn\'t measured.'),
  bullet('**Rig vs. sensor.** Where the rig moved (session 2), its movement can\'t yet be separated from sensor drift. The GUI now logs the raw accelerometer, which will allow that.'),

  H1('Next steps'),
  table(['Step', 'Status'], [
    ['Run the diagnostic build past the ~15 min mark', 'Ready: flash both boards'],
    ['Log the raw accelerometer in the CSV', 'Done (`knee_gui.py`)'],
    ['Fix the sample rate (faster output cycle; one row per received sample)', 'After diagnostics'],
    ['Investigate and fix the ruler-node sensor fault', 'After diagnostics'],
    ['Log angles with more decimal places', 'To do'],
    ['Stiffen the raised rig (clamp the ruler, fix both nodes rigidly)', 'To do'],
    ['Tilt both nodes during calibration', 'Next test'],
    ['Static test at known angles (30°, 45°, 90°) for accuracy', 'Next test'],
    ['Longer static run (30–60 min)', 'Next test'],
  ], W(3.4, 1.6)),
  gap(),
  P('Detailed analysis, per-run tables and method notes are in `analysis/static_test/FINDINGS.md` in the project repository.', { run: { color: MUTED, size: 19 } }),
];

const doc = new Document({
  creator: 'Dual-IMU Goniometer project', title: 'Static Stability Test Report',
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
      children: [new TextRun({ text: 'Static Stability Test Report   ·   page ', size: 16, color: MUTED }), new TextRun({ children: [PageNumber.CURRENT], size: 16, color: MUTED })] })] }) },
    children: body,
  }],
});
Packer.toBuffer(doc).then((b) => { fs.writeFileSync(OUT, b); console.log('wrote', OUT); });
