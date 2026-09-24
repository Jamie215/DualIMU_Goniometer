// Builds Static_Stability_Test_Report.docx by converting REPORT.md, so the Word
// version always matches the Markdown. Handles the subset of Markdown the report
// uses: headings, paragraphs, bullets (2-space nesting), numbered lists, tables,
// the figure, italic captions, blockquotes, **bold**, `code`, *italic*, [links](url).
// NOTE: Static_Stability_Test_Report.docx is now edited directly in Word and is the
// maintained version. Don't point this script at that file, or it will overwrite
// those edits; write to a different name if you need a Markdown-based draft.
// Usage: npm install docx && node build_report_docx.js draft_from_markdown.docx
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
  text = text.replace(/\[([^\]]+)\]\([^)]+\)/g, '$1');   // [label](url) -> label
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

// ---- Markdown -> docx body ------------------------------------------------ //
const md = fs.readFileSync(SRC + 'REPORT.md', 'utf8').split('\n');
const body = [];
// A Markdown-escaped \* becomes U+2217 (a star glyph) so the inline parser
// doesn't treat it as italics.
const runsEsc = (t, base = {}) => runs(t.replace(/\\\*/g, '∗'), base);
const para2 = (t, opts) => new Paragraph({ children: runsEsc(t, opts && opts.run), spacing: { after: 120 }, ...(opts && opts.para) });
let i = 0;
let pendingPara = [];
const flushPara = () => { if (pendingPara.length) { body.push(para2(pendingPara.join(' '))); pendingPara = []; } };
while (i < md.length) {
  const line = md[i];
  const t = line.trim();
  if (t === '') { flushPara(); i++; continue; }
  if (line.startsWith('# ')) {                       // title: "Name — Subtitle"
    flushPara();
    const [name, sub] = line.slice(2).split(' — ');
    body.push(new Paragraph({ heading: HeadingLevel.TITLE, children: [new TextRun(name)] }));
    if (sub) body.push(new Paragraph({ children: [new TextRun({ text: sub, size: 32, color: ACCENT })], spacing: { after: 160 } }));
    i++; continue;
  }
  if (line.startsWith('## ')) { flushPara(); body.push(H1(line.slice(3))); i++; continue; }
  if (line.startsWith('### ')) { flushPara(); body.push(H2(line.slice(4))); i++; continue; }
  if (t.startsWith('**Test dates:**')) {             // metadata line + rule under it
    flushPara();
    body.push(para2(t, { run: { size: 19, color: MUTED } }));
    body.push(new Paragraph({ border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: ACCENT, space: 4 } }, spacing: { after: 200 }, children: [] }));
    i++; continue;
  }
  if (t.startsWith('>')) {                           // blockquote -> muted note
    flushPara(); const q = [];
    while (i < md.length && md[i].trim().startsWith('>')) { q.push(md[i].trim().replace(/^>\s?/, '')); i++; }
    const text = q.join(' ').replace('[FINDINGS.md](FINDINGS.md)', '`analysis/static_test/FINDINGS.md` in the project repository');
    body.push(para2(text, { run: { color: MUTED, size: 19 } }));
    continue;
  }
  const im = t.match(/^!\[([^\]]*)\]\(([^)]+)\)$/);
  if (im) {                                          // the figure
    flushPara();
    const data = fs.readFileSync(SRC + im[2]);
    body.push(new Paragraph({ alignment: AlignmentType.CENTER, children: [new ImageRun({ type: 'png', data, transformation: { width: IMG_W, height: IMG_H },
      altText: { title: im[1], description: im[1], name: 'figure' } })] }));
    i++; continue;
  }
  if (t.startsWith('|')) {                           // table
    flushPara(); const rows = [];
    while (i < md.length && md[i].trim().startsWith('|')) {
      const cells = md[i].trim().replace(/^\|/, '').replace(/\|$/, '').split('|')
        .map((c) => c.trim().replace(/\\\*/g, '∗'));
      if (!cells.every((c) => /^:?-+:?$/.test(c))) rows.push(cells);
      i++;
    }
    const [head, ...rest] = rows;
    // column widths proportional to the longest cell text, within sensible bounds
    const len = head.map((_, c) => Math.min(60, Math.max(8, ...rows.map((r) => (r[c] || '').replace(/[*`]/g, '').length))));
    body.push(table(head, rest, W(...len)));
    body.push(gap());
    continue;
  }
  if (/^\*[^*].*\*$/.test(t) || t.startsWith('\\*')) {   // italic caption / footnote line
    flushPara(); const c = [t];
    i++;
    while (i < md.length && md[i].trim() !== '' && !md[i].trim().startsWith('|') && !md[i].startsWith('#') && !/^\s*- /.test(md[i])) { c.push(md[i].trim()); i++; }
    let text = c.join(' ');
    if (text.startsWith('*') && text.endsWith('*') && !text.startsWith('**')) text = text.slice(1, -1);
    body.push(new Paragraph({ children: runs(text.replace(/\\\*/g, '∗'), { italics: true, color: MUTED, size: 18 }), spacing: { after: 200 } }));
    continue;
  }
  const bm = line.match(/^(\s*)- (.*)$/);
  if (bm) {                                          // bullet (with wrapped continuation lines)
    flushPara();
    const level = Math.min(1, Math.floor(bm[1].length / 2));
    const parts = [bm[2]]; i++;
    while (i < md.length && md[i].trim() !== '' && /^\s+\S/.test(md[i]) && !/^\s*- /.test(md[i]) && !/^\s*\d+\. /.test(md[i])) { parts.push(md[i].trim()); i++; }
    body.push(new Paragraph({ numbering: { reference: 'bullets', level }, children: runs(parts.join(' ').replace(/\\\*/g, '∗')), spacing: { after: 60 } }));
    continue;
  }
  if (/^\d+\. /.test(line)) {                        // numbered list
    flushPara(); const items = [];
    while (i < md.length && /^\d+\. /.test(md[i])) {
      const parts = [md[i].replace(/^\d+\. /, '')]; i++;
      while (i < md.length && md[i].trim() !== '' && /^\s+\S/.test(md[i]) && !/^\d+\. /.test(md[i])) { parts.push(md[i].trim()); i++; }
      items.push(parts.join(' '));
    }
    body.push(...numbered(items));
    continue;
  }
  pendingPara.push(t); i++;
}
flushPara();

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
