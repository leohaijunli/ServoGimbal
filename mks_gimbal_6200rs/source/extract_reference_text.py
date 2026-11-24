"""Extract the cached SKF PDF's text via its explicit Unicode font CMap.

Small artifact-specific extractor. This is not a general PDF reader and does
not execute PDF content. Avoids installing another document tool for one file.
"""
from pathlib import Path
import re
import zlib

ROOT = Path(__file__).resolve().parent
path = ROOT/"reference/SKF_61800_dimensions.pdf"
streams = []
for match in re.finditer(rb"stream\r?\n(.*?)\r?\nendstream", path.read_bytes(), re.S):
    try:
        streams.append(zlib.decompress(match.group(1)))
    except zlib.error:
        pass
cmap = {}
for stream in streams:
    if b"begincmap" not in stream:
        continue
    for block in re.findall(rb"beginbfrange(.*?)endbfrange", stream, re.S):
        for a, b, c in re.findall(rb"<([0-9a-f]+)>\s*<([0-9a-f]+)>\s*<([0-9a-f]+)>", block, re.I):
            lo, hi, base = int(a, 16), int(b, 16), int(c, 16)
            for number in range(lo, hi+1):
                cmap[number] = chr(base+number-lo)
assert cmap[0x393] == "≈" and cmap[0x14] == "1"
lines = []
for stream in streams:
    if b"/G1 1 Tf" not in stream:
        continue
    for block in re.findall(rb"\bBT\b(.*?)\bET\b", stream, re.S):
        if b"/G1 1 Tf" not in block:
            continue
        chunks = re.findall(rb"<([0-9a-f]+)>", block, re.I)
        text = "".join("".join(cmap.get(int(h[j:j+4], 16), "�") for j in range(0, len(h), 4))
                       for h in chunks)
        if text:
            lines.append(text)
assert any("12.74" in text for text in lines)
assert any("16.26" in text for text in lines)
output = path.with_suffix(".txt")
output.write_text("PDF text, decoded using embedded Adobe-Identity-UCS CMap.\n"+"\n".join(lines)+"\n", encoding="utf-8")
print("EXTRACTED", output, "text runs", len(lines))
