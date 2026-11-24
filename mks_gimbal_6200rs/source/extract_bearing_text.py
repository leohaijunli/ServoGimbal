"""Read the cached SKF sheet's embedded Unicode map, without executing PDFs.

Reuses the existing SKF-sheet extraction method. This is intentionally limited
to the SKF-generated G1/Unicode text sheet, not a general-purpose PDF parser.
"""
import argparse
from pathlib import Path
import re
import zlib


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pdf", type=Path)
    args = parser.parse_args()
    content = args.pdf.read_bytes()
    assert content.startswith(b"%PDF"), "Downloaded source is not a PDF"
    streams = []
    for match in re.finditer(rb"stream\r?\n(.*?)\r?\nendstream", content, re.S):
        try:
            streams.append(zlib.decompress(match.group(1)))
        except zlib.error:
            continue
    maps = []
    for stream in streams:
        if b"begincmap" not in stream:
            continue
        cmap = {}
        for block in re.findall(rb"beginbfrange(.*?)endbfrange", stream, re.S):
            for a, b, c in re.findall(rb"<([0-9a-f]+)>\s*<([0-9a-f]+)>\s*<([0-9a-f]+)>", block, re.I):
                lo, hi, base = int(a, 16), int(b, 16), int(c, 16)
                for number in range(lo, hi + 1):
                    cmap[number] = chr(base + number - lo)
        for block in re.findall(rb"beginbfchar(.*?)endbfchar", stream, re.S):
            for a, b in re.findall(rb"<([0-9a-f]+)>\s*<([0-9a-f]+)>", block, re.I):
                cmap[int(a, 16)] = bytes.fromhex(b.decode()).decode("utf-16-be")
        if cmap:
            maps.append(cmap)
    assert len(maps) == 1, "Expected one SKF G1 Unicode map; use a full PDF reader otherwise"
    cmap = maps[0]
    lines = []
    for stream in streams:
        if b"/G1 1 Tf" not in stream:
            continue
        for block in re.findall(rb"\bBT\b(.*?)\bET\b", stream, re.S):
            if b"/G1 1 Tf" not in block:
                continue
            chunks = re.findall(rb"<([0-9a-f]+)>\s*Tj", block, re.I)
            text = "".join("".join(cmap.get(int(h[j:j+4], 16), "�")
                                   for j in range(0, len(h), 4)) for h in chunks)
            if text:
                assert "�" not in text, "Unmapped source glyph"
                lines.append(text)
    assert lines and any("6200" in line for line in lines), "No SKF 6200 text extracted"
    output = args.pdf.with_suffix(".txt")
    output.write_text("SKF sheet text decoded from its embedded Unicode CMap.\n"
                      + "\n".join(lines) + "\n", encoding="utf-8")
    print(output.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
