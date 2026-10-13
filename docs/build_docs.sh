#!/usr/bin/env bash
# Regenerate the Word copy of the handover document from its Markdown source.
# Styling matches the stationary test report (see make_reference_docx.py / style_tables.py).
# Requires pandoc (https://pandoc.org) and python-docx (pip install python-docx).
set -euo pipefail
cd "$(dirname "$0")/.."
python3 docs/make_reference_docx.py
REF=docs/reference.docx
pandoc docs/Project_Handover.md -o docs/Project_Handover.docx --reference-doc="$REF"
# The SOP is edited directly in Word (docs/SOP_Build_and_Use.docx is the master), so it is
# not regenerated here. docs/SOP_Build_and_Use.md is a mirror of it (see the note at its top).
python3 docs/style_tables.py docs/Project_Handover.docx
echo "Wrote docs/Project_Handover.docx"
