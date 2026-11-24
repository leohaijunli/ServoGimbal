"""Read MP4 container metadata without installing another video tool."""
from pathlib import Path
import hashlib
import json
import struct
import argparse

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("file", type=Path)
PATH = parser.parse_args().file
DATA = PATH.read_bytes()


def boxes(start, end):
    while start+8 <= end:
        size, kind = struct.unpack_from(">I4s", DATA, start)
        header = 8
        if size == 1:
            size = struct.unpack_from(">Q", DATA, start+8)[0]
            header = 16
        if size == 0:
            size = end-start
        assert size >= header and start+size <= end
        yield kind.decode("ascii"), start+header, start+size
        start += size


def child(start, end, name):
    return next((a, b) for k, a, b in boxes(start, end) if k == name)


def timing(start):
    version = DATA[start]
    offset = start+(20 if version == 1 else 12)
    scale = struct.unpack_from(">I", DATA, offset)[0]
    duration = struct.unpack_from(">Q" if version == 1 else ">I", DATA, offset+4)[0]
    return duration/scale


moov = child(0, len(DATA), "moov")
tracks = []
for kind, start, end in boxes(*moov):
    if kind != "trak":
        continue
    mdia = child(start, end, "mdia")
    hdlr = child(*mdia, "hdlr")
    handler = DATA[hdlr[0]+8:hdlr[0]+12].decode("ascii")
    if handler != "vide":
        tracks.append({"handler": handler})
        continue
    tkhd = child(start, end, "tkhd")
    width, height = struct.unpack_from(">II", DATA, tkhd[1]-8)
    seconds = timing(child(*mdia, "mdhd")[0])
    stbl = child(*child(*mdia, "minf"), "stbl")
    stsz = child(*stbl, "stsz")
    frames = struct.unpack_from(">I", DATA, stsz[0]+8)[0]
    stsd = child(*stbl, "stsd")
    codec = DATA[stsd[0]+12:stsd[0]+16].decode("ascii")
    tracks.append({"handler": handler, "width": width/65536,
                   "height": height/65536, "duration_s": seconds,
                   "frames": frames, "fps": frames/seconds, "codec": codec})
video = next(t for t in tracks if t["handler"] == "vide")
assert video["width"] == 1280 and video["height"] == 720
assert video["frames"] == 288 and abs(video["duration_s"]-12.0) < 0.01
assert video["codec"] == "avc1" and abs(video["fps"]-24) < 0.01
report = {"file": PATH.name, "bytes": len(DATA), "sha256": hashlib.sha256(DATA).hexdigest(),
          "tracks": tracks, "pass": True,
          "coverage": "MP4 header/sample-table checks plus Blender completed render log; not a full external-decoder test or physical mechanism validation"}
PATH.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))
