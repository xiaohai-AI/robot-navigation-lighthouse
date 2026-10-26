#!/bin/bash
# Build the textbook PDF from docs/book/src/*.md
#
# Build-time transforms (the sources stay clean/GitHub-readable):
#   1. ✅ ❌ ✗ ⏹ are replaced by PASS/FAIL/x/STOP — the available Noto CJK fonts
#      do not carry those codepoints, and xeCJK is unavailable (no ctexhook.sty).
#   2. Chapter numbers are prepended here ("1 — …") instead of relying on
#      --number-sections, because section headings already carry manual numbers
#      (as in "## 9.3") and pandoc would otherwise print both.
set -euo pipefail
cd "$(dirname "$0")"

OUT=lighthouse-textbook.pdf
rm -rf build && mkdir -p build/src

clean() {  # $1 = input, $2 = output
  sed -e 's/✅/PASS/g' -e 's/❌/FAIL/g' -e 's/✗/x/g' -e 's/⏹/STOP/g' \
      -e 's/⚠️/!/g' -e 's/⚠/!/g' "$1" > "$2"
}

n=1
for f in src/0[1-9]_*.md src/1[0-5]_*.md; do
  clean "$f" "build/.tmp.md"
  sed -e "0,/^# /s||# ${n} — |" build/.tmp.md > "build/$f"
  n=$((n + 1))
done
clean src/16_appendix.md build/src/16_appendix.md   # appendices keep "Appendix A …"
rm -f build/.tmp.md

pandoc build/src/0*.md build/src/1*.md \
  -o "$OUT" \
  --pdf-engine=xelatex \
  --top-level-division=chapter \
  --toc --toc-depth=2 \
  --highlight-style=tango \
  --resource-path=.:figures:src \
  -V documentclass=report \
  -V classoption=oneside \
  -V geometry:margin=2.3cm \
  -V fontsize=10pt \
  -V mainfont="Noto Serif CJK SC" \
  -V sansfont="Noto Sans CJK SC" \
  -V monofont="Noto Sans Mono CJK SC" \
  -V colorlinks=true \
  -V linkcolor=blue \
  -V urlcolor=blue \
  -V toccolor=black \
  -V linestretch=1.05 \
  2>&1 | tee build_pdf.log

echo "---"
ls -lh "$OUT"
echo "missing-glyph warnings: $(grep -c 'Missing character' build_pdf.log || true)"
echo "overfull boxes:         $(grep -c 'Overfull' build_pdf.log || true)"
command -v pdfinfo >/dev/null && pdfinfo "$OUT" | grep -E "^Pages|^Page size"
