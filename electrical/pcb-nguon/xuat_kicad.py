"""Xuất schematic + PCB nháp cho KiCad 9 từ mạch SKiDL trong pcb_nguon.py.
SKiDL 3.0 không sinh được .kicad_sch từ template tự khai báo (không có ký hiệu thư viện), nên script này
tự viết .kicad_sch: mỗi linh kiện là ký hiệu hộp chữ nhật, mỗi chân gắn nhãn toàn cục theo tên net
(không vẽ dây). Chạy ERC bằng: kicad-cli sch erc pcb_nguon.kicad_sch
.kicad_pcb là NHÁP: khối thô theo nhóm, pad giả theo số chân, chưa đi dây, chưa DRC, không Gerber.
Chạy: python xuat_kicad.py"""
import os, uuid, math
import pcb_nguon  # noqa: dựng mạch (chdir về thư mục này)
import builtins
default_circuit = builtins.default_circuit

OUT = os.path.dirname(os.path.abspath(__file__))
U = lambda: str(uuid.uuid4())
ROOT = U()
esc = lambda s: str(s).replace("\\", "\\\\").replace('"', "'")
PT = {"PASSIVE": "passive", "PWRIN": "power_in", "INPUT": "input", "OUTPUT": "output",
      "BIDIR": "bidirectional", "OPENCOLL": "open_collector"}

parts = sorted(default_circuit.parts, key=lambda p: p.ref)
netname = lambda pin: (pin.nets[0].name if pin.nets else None)

# ---------- schematic ----------
def sym_geom(p):
    n = len(p.pins); left = math.ceil(n / 2); right = n - left
    longest = max(len(str(pn.name)) for pn in p.pins)
    w = 20.32 if longest <= 7 else 30.48
    rows = max(left, right, 1)
    h = (rows + 1) * 2.54
    return n, left, right, w, h

def lib_symbol(p):
    n, left, right, w, h = sym_geom(p)
    name = "Faibotic:" + p.name
    pins = []
    order = sorted(p.pins, key=lambda x: int(x.num) if str(x.num).isdigit() else 0)
    for i, pn in enumerate(order):
        if i < left:
            x, ang, y = -(w / 2 + 2.54), 0, (left - 1) * 1.27 - i * 2.54
        else:
            j = i - left
            x, ang, y = (w / 2 + 2.54), 180, (right - 1) * 1.27 - j * 2.54
        pins.append(f'(pin {PT[pn.func.name]} line (at {x:.2f} {y:.2f} {ang}) (length 2.54) '
                    f'(name "{esc(pn.name)}" (effects (font (size 1 1)))) (number "{pn.num}" (effects (font (size 1 1)))))')
    return (f'(symbol "{name}" (pin_names (offset 0.5)) (exclude_from_sim no) (in_bom yes) (on_board yes)'
            f'(property "Reference" "X" (at 0 {h/2+1.5:.2f} 0) (effects (font (size 1.27 1.27))))'
            f'(property "Value" "{p.name}" (at 0 {-h/2-1.5:.2f} 0) (effects (font (size 1.27 1.27))))'
            f'(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f'(property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f'(property "Description" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f'(symbol "{p.name}_0_1" (rectangle (start {-w/2:.2f} {h/2:.2f}) (end {w/2:.2f} {-h/2:.2f}) '
            f'(stroke (width 0.254) (type default)) (fill (type background))))'
            f'(symbol "{p.name}_1_1" {" ".join(pins)}))'), order

FLAG = ('(symbol "Faibotic:PWR_FLAG" (power) (pin_names (offset 0) hide) (exclude_from_sim no) (in_bom no) (on_board no)'
        '(property "Reference" "#FLG" (at 0 1.9 0) (effects (font (size 1.27 1.27)) (hide yes)))'
        '(property "Value" "PWR_FLAG" (at 0 3.8 0) (effects (font (size 1.27 1.27))))'
        '(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
        '(property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
        '(property "Description" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'
        '(symbol "PWR_FLAG_0_1" (polyline (pts (xy 0 0) (xy 0 1.27) (xy -1.016 1.905) (xy 0 2.54) (xy 1.016 1.905) (xy 0 1.27)) '
        '(stroke (width 0) (type default)) (fill (type none))))'
        '(symbol "PWR_FLAG_1_1" (pin power_out line (at 0 0 90) (length 0) (name "pwr" (effects (font (size 1.27 1.27)))) '
        '(number "1" (effects (font (size 1.27 1.27)))))))')

libs, seen, orders = [], set(), {}
for p in parts:
    if p.name not in seen:
        seen.add(p.name); s, o = lib_symbol(p); libs.append(s); orders[p.name] = o
libs.append(FLAG)

body = []
# vị trí: cột, mỗi cột dài tối đa 190 mm
x0, y0, colw, maxh = 40.64, 40.64, 111.76, 190.5
cx, cy, colh = x0, y0, 0
snap = lambda v: round(v / 2.54) * 2.54
def label(net, x, y, ang):
    return (f'(global_label "{esc(net)}" (shape passive) (at {x:.2f} {y:.2f} {ang}) (effects (font (size 1.27 1.27)) '
            f'(justify {"right" if ang == 180 else "left"})) (uuid "{U()}") '
            f'(property "Intersheetrefs" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes))))')
pwr_nets = {}
for p in parts:
    n, left, right, w, h = sym_geom(p)
    if cy + h > y0 + maxh:
        cx += colw; cy = y0
    px, py = snap(cx + 38.1), snap(cy + h / 2)
    cy += h + 7.62
    inst = (f'(symbol (lib_id "Faibotic:{p.name}") (at {px:.2f} {py:.2f} 0) (unit 1) (exclude_from_sim no) (in_bom yes) '
            f'(on_board yes) (dnp no) (uuid "{U()}") '
            f'(property "Reference" "{esc(p.ref)}" (at {px:.2f} {py-h/2-1.5:.2f} 0) (effects (font (size 1.27 1.27)))) '
            f'(property "Value" "{esc(p.value)}" (at {px:.2f} {py+h/2+1.5:.2f} 0) (effects (font (size 1.0 1.0)))) '
            f'(property "Footprint" "{esc(p.footprint or "")}" (at {px:.2f} {py:.2f} 0) (effects (font (size 1.27 1.27)) (hide yes))) '
            f'(property "Datasheet" "" (at {px:.2f} {py:.2f} 0) (effects (font (size 1.27 1.27)) (hide yes)))')
    for pn in p.pins: inst += f' (pin "{pn.num}" (uuid "{U()}"))'
    inst += f' (instances (project "pcb_nguon" (path "/{ROOT}" (reference "{esc(p.ref)}") (unit 1)))))'
    body.append(inst)
    for i, pn in enumerate(sorted(p.pins, key=lambda x: int(x.num) if str(x.num).isdigit() else 0)):
        if i < left:
            x, y, ang = px - (w / 2 + 2.54), py - ((left - 1) * 1.27 - i * 2.54), 180
        else:
            j = i - left
            x, y, ang = px + (w / 2 + 2.54), py - ((right - 1) * 1.27 - j * 2.54), 0
        nm = netname(pn)
        if nm is None or nm.startswith("NC_") or nm.startswith("N$"):
            body.append(f'(no_connect (at {x:.2f} {y:.2f}) (uuid "{U()}"))')
            continue
        body.append(label(nm, x, y, ang))
        if pn.func.name == "PWRIN": pwr_nets.setdefault(nm, (x, y, ang))
        if pn.func.name == "OUTPUT": pwr_nets[nm] = None  # có nguồn đẩy
for nm, pos in pwr_nets.items():
    if pos is None: continue
    x, y, ang = pos   # PWR_FLAG đặt ở góc riêng, nối bằng nhãn cùng tên
fx, fy = 40.64, y0 + maxh + 25.4
for k, (nm, pos) in enumerate(p for p in pwr_nets.items() if p[1] is not None):
    xx = snap(fx + k * 25.4)
    body.append(f'(symbol (lib_id "Faibotic:PWR_FLAG") (at {xx:.2f} {fy:.2f} 0) (unit 1) (exclude_from_sim no) (in_bom no) '
                f'(on_board no) (dnp no) (uuid "{U()}") (property "Reference" "#FLG{k+1:02d}" (at {xx:.2f} {fy:.2f} 0) '
                f'(effects (font (size 1.27 1.27)) (hide yes))) (property "Value" "PWR_FLAG" (at {xx:.2f} {fy-5:.2f} 0) '
                f'(effects (font (size 1.0 1.0)))) (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes))) '
                f'(property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes))) (pin "1" (uuid "{U()}")) '
                f'(instances (project "pcb_nguon" (path "/{ROOT}" (reference "#FLG{k+1:02d}") (unit 1)))))')
    body.append(label(nm, xx, fy, 270 if False else 0))
body.append(f'(text "Faibotic M1 PCB nguon v0.1 - NHAP, CHUA DUYET. Sinh tu pcb_nguon.py (SKiDL). Ket noi bang nhan toan cuc, khong ve day." '
            f'(exclude_from_sim no) (at 40.64 25.4 0) (effects (font (size 3 3)) (justify left)) (uuid "{U()}"))')
sch = (f'(kicad_sch (version 20250114) (generator "eeschema") (generator_version "9.0") (uuid "{ROOT}") (paper "A0") '
       f'(lib_symbols {" ".join(libs)}) {" ".join(body)} (sheet_instances (path "/" (page "1"))) (embedded_fonts no))')
open(os.path.join(OUT, "pcb_nguon.kicad_sch"), "w", encoding="utf-8").write(sch)

# ---------- PCB nháp ----------
BW, BH = 100.0, 80.0   # cad/ASSUMPTIONS.md: PCB nguồn 100 x 80 mm
SIZE = {"J1": (20, 12), "J9": (20, 12), "J10": (10, 8), "J11": (20, 8), "J8": (20, 8), "J2": (20, 12), "J3": (20, 12),
        "J4": (14, 9), "J5": (12, 8), "J6": (24, 4), "J7": (6, 4), "K2": (16, 13), "K3": (15, 11), "K4": (15, 11),
        "F2": (5, 3), "F3": (22, 11), "F4": (22, 11), "F5": (22, 11), "F6": (16, 8), "RPC": (12, 6), "RS1": (16, 8),
        "RS2": (8, 4), "C1": (10, 10), "C6": (8, 8), "C14": (8, 8), "TVS1": (7, 6), "TVS2": (7, 6), "D2": (7, 6),
        "L1": (12, 12), "U1": (4, 3.5), "U3": (5, 5), "U4": (4, 3), "U2": (4, 3.5), "U5": (3, 3), "U6": (3, 3)}
def size_of(p):
    if p.ref in SIZE: return SIZE[p.ref]
    t = p.name
    if p.ref[0] == "C": return (5, 5) if "47u" in str(p.value) else (3, 2)
    if p.ref[0] == "Q": return (3, 3)
    if t in ("D", "ZENER"): return (5, 3)
    if t == "LED": return (3, 2)
    if t == "R": return (3, 2)
    return (4, 3)
GROUP = {}
for r in "J1 J9 TVS1 TVS2 RS1 C1 J2 J3 J4 F3 F4 F5 K4 RPC F10 F11".split(): GROUP[r] = "A"
for r in "U3 L1 D2 J5 F6 RS2 U2 R5 R6 R7 R8 R9 R10 C2 C3 C4 C5 C6 C7 C8 C9 C10 C11 C12".split(): GROUP[r] = "B"
for r in ("R30 R31 R32 R33 R34 R35 R36 R37 R38 ZDES ZDLA ZDAR R3L LED3 J6 J7 U1 U4 U5 U6 U7 R39 R20 C15 D9 R21 C16 C17 C18 R22 Q4 Q5 R11 R12 R1 R2 C13 D5 LED1 LED2 R1L R2L").split(): GROUP[r] = "C"
REG = {"A": (2, 33, 67, 78, "A nguon lon + do dong: J1 J9 K4/RPC RS1 C1 F3-F5 J2-J4"),
       "B": (69, 2, 98, 44, "B buck 5 V + nhanh Pi: U3 L1 F6 RS2 U2 J5"),
       "C": (69, 46, 98, 78, "C logic Pi + POR: U4 U5 U6 J6 J7 U1"),
       "D": (2, 2, 67, 31, "D E-stop / reset: J8 J10 J11 K2 K3 F2 Q1 D1 Z1")}
by = {k: [] for k in REG}
for p in parts: by[GROUP.get(p.ref, "D")].append(p)
pos, overflow = {}, []
for k, lst in by.items():
    xa, ya, xb, yb, _ = REG[k]
    lst = sorted(lst, key=lambda p: (-size_of(p)[1], -size_of(p)[0]))
    shelves = []   # [y, h, xcursor]
    for p in lst:
        w, h = size_of(p)
        for sh in shelves:
            if sh[2] + w <= xb and h <= sh[1]:
                x, y = sh[2], sh[0]; sh[2] += w + 1.2; break
        else:
            y = shelves[-1][0] + shelves[-1][1] + 1.2 if shelves else ya
            shelves.append([y, h, xa + w + 1.2]); x = xa
        if y + h > yb: overflow.append((k, p.ref))
        pos[p.ref] = (x + w / 2, y + h / 2, w, h)
nets = sorted({pn.nets[0].name for p in parts for pn in p.pins if pn.nets})
nid = {n: i + 1 for i, n in enumerate(nets)}
OX, OY = 100.0, 60.0
fps = []
for p in parts:
    cxp, cyp, w, h = pos[p.ref]
    n = len(p.pins); pw = max(0.25, min(1.6, 0.6 * w / n)); order = sorted(p.pins, key=lambda x: int(x.num) if str(x.num).isdigit() else 0)
    pads = []
    for i, pn in enumerate(order):
        px = -w / 2 + (i + 0.5) * w / n
        nn = pn.nets[0].name if pn.nets else None
        net = f'(net {nid[nn]} "{esc(nn)}")' if nn else ""
        pads.append(f'(pad "{pn.num}" smd rect (at {px:.3f} 0) (size {pw:.2f} {min(pw*1.5,h-0.4):.2f}) '
                    f'(layers "F.Cu" "F.Mask" "F.Paste") {net} (uuid "{U()}"))')
    fps.append(f'(footprint "Faibotic:BLOCK_{esc(p.name)}" (layer "F.Cu") (uuid "{U()}") (at {OX+cxp:.3f} {OY+cyp:.3f}) '
               f'(property "Reference" "{esc(p.ref)}" (at 0 {-h/2-0.8:.2f} 0) (layer "F.SilkS") (uuid "{U()}") (effects (font (size 0.8 0.8) (thickness 0.12)))) '
               f'(property "Value" "{esc(p.value)[:40]}" (at 0 {h/2+0.8:.2f} 0) (layer "F.Fab") (uuid "{U()}") (effects (font (size 0.6 0.6) (thickness 0.1)))) '
               f'(fp_rect (start {-w/2:.2f} {-h/2:.2f}) (end {w/2:.2f} {h/2:.2f}) (stroke (width 0.05) (type default)) (fill no) (layer "F.CrtYd") (uuid "{U()}")) '
               f'{" ".join(pads)})')
gr = [f'(gr_rect (start {OX} {OY}) (end {OX+BW} {OY+BH}) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{U()}"))']
for k, (xa, ya, xb, yb, t) in REG.items():
    gr.append(f'(gr_rect (start {OX+xa-0.5} {OY+ya-0.5}) (end {OX+xb+0.5} {OY+yb+0.5}) (stroke (width 0.15) (type dash)) (fill no) (layer "Dwgs.User") (uuid "{U()}"))')
    gr.append(f'(gr_text "{t}" (at {OX+xa} {OY+ya-1.2}) (layer "Dwgs.User") (uuid "{U()}") (effects (font (size 0.9 0.9) (thickness 0.12)) (justify left)))')
gr.append(f'(gr_text "NHAP: chua di day, chua DRC, khong Gerber. Pad gia, khoi tho. Chua duyet (REVIEW.md)." (at {OX} {OY+BH+3}) (layer "Cmts.User") (uuid "{U()}") (effects (font (size 1.2 1.2) (thickness 0.15)) (justify left)))')
layers = ('(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (9 "F.Adhes" user "F.Adhesive") (11 "B.Adhes" user "B.Adhesive") '
          '(13 "F.Paste" user) (15 "B.Paste" user) (5 "F.SilkS" user "F.Silkscreen") (7 "B.SilkS" user "B.Silkscreen") '
          '(1 "F.Mask" user) (3 "B.Mask" user) (17 "Dwgs.User" user "User.Drawings") (19 "Cmts.User" user "User.Comments") '
          '(21 "Eco1.User" user "User.Eco1") (23 "Eco2.User" user "User.Eco2") (25 "Edge.Cuts" user) (27 "Margin" user) '
          '(31 "F.CrtYd" user "F.Courtyard") (29 "B.CrtYd" user "B.Courtyard") (35 "F.Fab" user) (33 "B.Fab" user))')
netdecl = '(net 0 "") ' + " ".join(f'(net {i} "{esc(n)}")' for n, i in nid.items())
pcb = (f'(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0") (general (thickness 1.6) (legacy_teardrops no)) '
       f'(paper "A4") {layers} (setup (pad_to_mask_clearance 0)) {netdecl} {" ".join(gr)} {" ".join(fps)} (embedded_fonts no))')
open(os.path.join(OUT, "pcb_nguon.kicad_pcb"), "w", encoding="utf-8").write(pcb)
print("parts", len(parts), "nets", len(nets), "overflow", overflow)
for k, lst in by.items(): print(k, len(lst), round(sum(size_of(p)[0] * size_of(p)[1] for p in lst)), "mm2 /", (REG[k][2]-REG[k][0])*(REG[k][3]-REG[k][1]))
