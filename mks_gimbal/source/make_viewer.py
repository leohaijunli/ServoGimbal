"""Build an offline, double-clickable viewer. No HTTP server or CDN needed."""
from pathlib import Path
import base64
import hashlib
import json
import re

CODE = Path(__file__).resolve().parent
HERE = CODE.parent if CODE.name == "source" and (CODE.parent/"reference").is_dir() else CODE
OUT = HERE/"output"
glb = OUT/"mks_gimbal_assembly.glb"
template = (CODE/"viewer.html").read_text(encoding="utf-8")
for name in ("three.min.js", "OrbitControls.js", "GLTFLoader.js"):
    code = (CODE/"viewer_vendor"/name).read_text(encoding="utf-8")
    code = re.sub(r"</script", r"<\\/script", code, flags=re.I)
    template = template.replace(f'<script src="viewer_vendor/{name}"></script>', "<script>\n"+code+"\n</script>")
encoded = base64.b64encode(glb.read_bytes()).decode("ascii")
template = template.replace("window.__MKS_MODEL_BASE64__ = null;",
                            "window.__MKS_MODEL_BASE64__ = '"+encoded+"';")
assert '<script src=' not in template
(OUT/"viewer.html").write_text(template, encoding="utf-8")
report = {"offline": True, "three_version": "0.128.0",
          "embedded_GLB_sha256": hashlib.sha256(glb.read_bytes()).hexdigest(),
          "html_sha256": hashlib.sha256((OUT/"viewer.html").read_bytes()).hexdigest(),
          "sources": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (CODE/"viewer_vendor").glob("*.js")},
          "browser_validation": "pending"}
(OUT/"viewer_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print("OFFLINE VIEWER", OUT/"viewer.html", "bytes", (OUT/"viewer.html").stat().st_size)
