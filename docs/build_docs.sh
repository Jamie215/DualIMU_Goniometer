#!/usr/bin/env bash
# Regenerate the Word copies of the handover documents from their Markdown source.
# Styling matches the stationary test report (see make_reference_docx.py / style_tables.py).
# Requires pandoc (https://pandoc.org) and python-docx (pip install python-docx).
set -euo pipefail
cd "$(dirname "$0")/.."
python3 docs/make_reference_docx.py
REF=docs/reference.docx
pandoc docs/DESIGN_DECISIONS.md   -o docs/Design_Decisions.docx  --reference-doc="$REF"
# The SOP is edited directly in Word (docs/SOP_Build_and_Use.docx is the master), so it is
# not regenerated here. docs/SOP_Build_and_Use.md is a mirror of it (see the note at its top).
pandoc docs/HANDOVER.md           -o docs/Handover_Summary.docx  --reference-doc="$REF"
python3 docs/style_tables.py docs/Design_Decisions.docx docs/Handover_Summary.docx
echo "Wrote docs/Design_Decisions.docx, docs/Handover_Summary.docx"
