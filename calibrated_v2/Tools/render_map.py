"""Render source rectangles and the exact host-planner field-test route."""
from pathlib import Path
import re
import json
import subprocess
import math

root = Path(__file__).resolve().parents[1]
source = (root / "Src/grid_map.c").read_text(encoding="utf-8")
data = source.split("grid_static_obstacles[18] = {", 1)[1].split("};", 1)[0]
rects = [tuple(map(int, r)) for r in re.findall(r"\{(\d+),(\d+),(\d+),(\d+)\}", data)]
assert len(rects) == 18
plan = None
exe = root / "build/check_navigation_config.exe"
if exe.exists():
    plan = json.loads(subprocess.check_output([str(exe), "--json"], text=True))
parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 2000 1600">',
         '<rect width="2000" height="1600" fill="#f8fafc"/>',
         '<g font-family="Arial, sans-serif" fill="#172554">',
         '<text x="100" y="52" font-size="32">MAP 180 x 120 cm — LUOI 5 x 5 cm</text>',
         '<text x="100" y="91" font-size="22">Xe 9x9 cm; xuat phat (20,-4.5) cm huong Bac; ToF +25 / 0 / -25 do.</text>']
for v in range(37):
    px = 100 + v * 50
    parts.append(f'<path d="M{px} 140 V1340" stroke="#cbd5e1" stroke-width="1"/>')
for v in range(25):
    py = 140 + v * 50
    parts.append(f'<path d="M100 {py} H1900" stroke="#cbd5e1" stroke-width="1"/>')
for i, (x0, y0, x1, y1) in enumerate(rects):
    x, y = 100 + x0, 1340 - y1
    label = f"T{i + 1}" if i < 17 else "VUNG CAM"
    color = "#e2e8f0" if i < 17 else "#fed7aa"
    parts.append(f'<rect x="{x}" y="{y}" width="{x1-x0}" height="{y1-y0}" '
                 f'fill="{color}" stroke="#334155" stroke-width="4"/>')
    parts.append(f'<text x="{x+(x1-x0)/2}" y="{y+(y1-y0)/2+8}" '
                 f'text-anchor="middle" font-size="24">{label}</text>')
if plan:
    sx, sy = plan["start"]
    points = [(sx,sy),(sx,plan["entry_y"])]
    points += [(n%36*50+25,n//36*50+25) for n in plan["path"]]
    coords = " ".join(f"{100+x},{1340-y}" for x,y in points)
    parts.append(f'<polyline points="{coords}" fill="none" stroke="#2563eb" stroke-width="7" stroke-linejoin="round"/>')
    length,width = plan["body"]
    parts.append(f'<rect x="{100+sx-width/2}" y="{1340-sy-length/2}" width="{width}" height="{length}" fill="#bfdbfe" stroke="#1d4ed8" stroke-width="4"/>')
    for forward,left,angle in plan["sensors"]:
        ox,oy = sx-left,sy+forward
        ex,ey = ox-50*math.sin(math.radians(angle)),oy+50*math.cos(math.radians(angle))
        parts.append(f'<path d="M{100+ox} {1340-oy} L{100+ex} {1340-ey}" stroke="#f97316" stroke-width="4"/>')
        parts.append(f'<circle cx="{100+ox}" cy="{1340-oy}" r="5" fill="#ea580c"/>')
    parts.append(f'<text x="100" y="1535" font-size="22">A* {len(plan["path"])} o; duong xanh la duong danh dinh. Can test sai so encoder/ToF tren san that.</text>')
parts.extend(['<path d="M100 1340 V140 H1900 V1340 M700 1340 V740 H1300 V1340" '
              'fill="none" stroke="#172554" stroke-width="10"/>',
              '<path d="M100 1340 H700" stroke="#059669" stroke-width="10" stroke-dasharray="22 15"/>',
              '<path d="M1300 1340 H1900" stroke="#7c3aed" stroke-width="10" stroke-dasharray="22 15"/>',
              '<text x="400" y="1475" text-anchor="middle" font-size="28">XUAT PHAT</text>',
              '<text x="1600" y="1475" text-anchor="middle" font-size="28">DICH</text>',
              '<text x="100" y="1575" font-size="22">Goc (0,0) o duoi trai; +X sang phai, +Y len tren. Toa do vat can doc tu anh, can doi chieu tren san.</text>',
              '</g></svg>'])
out = root / "build/map_review.svg"
out.parent.mkdir(exist_ok=True)
out.write_text("\n".join(parts), encoding="utf-8")
print(out)
