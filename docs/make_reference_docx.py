#!/usr/bin/env python3
"""Build docs/reference.docx: pandoc's default Word template restyled to match the
stationary test report (Calibri, dark-blue headings, 0.75" margins, US Letter).

Pandoc needs a template that contains its own table and list styles, so the
report itself can't be used directly as --reference-doc.

Usage: python docs/make_reference_docx.py   (requires pandoc and python-docx)
"""
import os
import subprocess

import docx
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Inches, Pt, RGBColor

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'reference.docx')
BLUE = RGBColor(0x1F, 0x4E, 0x79)
INK = RGBColor(0x0B, 0x0B, 0x0B)

subprocess.run(['pandoc', '-o', OUT, '--print-default-data-file', 'reference.docx'], check=True)

doc = docx.Document(OUT)
styles = {st.name.lower(): st for st in doc.styles}
for name, st in styles.items():
    if hasattr(st, 'font') and name not in ('source code', 'verbatim char'):
        st.font.name = 'Calibri'
for name, size, color in [('Normal', 10.5, INK), ('Body Text', 10.5, INK), ('First Paragraph', 10.5, INK),
                          ('Compact', 10.5, INK), ('Title', 22, INK), ('Subtitle', 13, INK),
                          ('Heading 1', 14, BLUE), ('Heading 2', 11.5, BLUE), ('Heading 3', 11, BLUE)]:
    st = styles[name.lower()]
    st.font.size = Pt(size)
    st.font.color.rgb = color
    if name.startswith('Heading') or name == 'Title':
        st.font.bold = True
    if name in ('Title', 'Subtitle'):
        st.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
for sec in doc.sections:
    sec.page_width, sec.page_height = Inches(8.5), Inches(11)
    sec.left_margin = sec.right_margin = Inches(0.75)
    sec.top_margin = sec.bottom_margin = Inches(0.75)
doc.save(OUT)
print('Wrote', OUT)
