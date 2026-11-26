"""Phòng lab mô phỏng cho M1: một nguồn sinh cả sim/lab/lab.xml (MuJoCo, include ../m1.xml) và sim/lab/lab-scene.json (web dựng lại bằng hình khối).

Chạy từ gốc repo:  .venv/bin/python sim/lab/build_lab.py         (ghi 2 file)
                   .venv/bin/python sim/lab/build_lab.py --check (mã 1 nếu file trên đĩa lệch với nguồn)

Hệ toạ độ (giống MuJoCo, mét): x hướng từ chỗ robot đứng lúc đầu vào bàn, y sang trái robot, z lên; robot bắt đầu ở gốc, mặt hướng +x.
Web three.js (y lên): (x, y, z)_web = (x, z, -y).

Bố cục theo tầm với (SOR-006): bàn 0,75 m, robot đứng cách mép bàn 0,25 m (thân đế bán kính 0,20 m), lift 0,45-0,50 m;
mọi chỗ cần kẹp nằm trong dải 0,07-0,24 m tính từ mép bàn, độ cao 0,77-0,98 m. Robot đi ngang dọc bàn (đế omni) để tới từng trạm.
Màu: bảng màu bộ render (media/renders-web/src/README.md) và ui-tokens: navy #1e3a5f, ngà #f1efe8, nền #d6d3cc, nhôm, slate;
màu vật: danger #b91c1c (đỏ), secondary #2563eb (xanh dương), success #15803d (xanh lá). Không cam.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# ---------------- bảng màu ----------------
C = {
    "navy": "#1e3a5f", "ivory": "#f1efe8", "bg": "#d6d3cc", "white": "#fafaf9", "alu": "#b3b7bd",
    "slate": "#64748b", "slate_dk": "#475569", "ink": "#0f172a", "tint": "#eaf0f7", "border": "#e2e8f0",
    "red": "#b91c1c", "blue": "#2563eb", "green": "#15803d",
}

# ---------------- kích thước chính ----------------
TABLE_H = 0.75
TABLE_EDGE_X = 0.50          # mép bàn phía robot
TABLE_DEPTH = 0.60
TABLE_Y = (-1.0, 1.0)
TOP_T = 0.03
ROBOT_STANDOFF = 0.21        # tâm đế tới mép bàn khi làm việc (S1G: 0,25 -> 0,19 vì trục pan CAD lùi 6 cm, 0,12 -> 0,06; trục pan vẫn cách mép bàn 0,13; khay và bánh trước chui dưới bàn)
ROW_D = 0.13                 # hàng vật cách mép bàn
LIFT_WORK = 0.45             # lift khi kẹp trên mặt bàn = đỉnh hành trình P-L (S1G: trục vai 1,012 m; S1C 0,50 với trục vai 1,05)

OBJECTS = [  # id, hình, kích thước (đầy đủ, m), màu
    ("obj_red_cube", "box", (0.04, 0.04, 0.04), "red"),
    ("obj_blue_cyl", "cylinder", (0.02, 0.05), "blue"),
    ("obj_green_cube", "box", (0.04, 0.04, 0.04), "green"),
    ("obj_red_cyl", "cylinder", (0.02, 0.05), "red"),
    ("obj_blue_cube", "box", (0.045, 0.045, 0.045), "blue"),
    ("obj_green_cyl", "cylinder", (0.02, 0.05), "green"),
    ("obj_red_block", "box", (0.03, 0.05, 0.04), "red"),
    ("obj_blue_block", "box", (0.035, 0.05, 0.035), "blue"),
]
OBJ_Y0, OBJ_DY = 0.12, 0.06
OBJ_MASS = 0.06
ZONE = {"pos": (TABLE_EDGE_X + 0.14, -0.02), "size": (0.14, 0.20)}     # vùng đích gắp đặt (thảm navy)
ZONE_SLOTS = [(0.0, 0.05), (0.0, -0.01), (0.0, -0.07)]                   # chỗ đặt k = 3 vật, so với tâm vùng
# vật trong tủ lạnh: lon xanh lá đứng trên kệ (mặt kệ 0,75 m), lấy ra bằng tay phải, dụng cụ nằm ngang
FRIDGE_OBJ = ("obj_green_can", "cylinder", (0.02, 0.06), "green")
FRIDGE_SHELF_Z = 0.75
FRIDGE_OBJ_XY = (TABLE_EDGE_X + 0.12, -1.28)

TRAY = {"center": (TABLE_EDGE_X + ROW_D, -0.32), "cell_w": 0.10, "cell_d": 0.13, "wall": 0.006, "h": 0.04,
        "colors": ["red", "green", "blue"], "shapes": ["tron", "vuong", "tam_giac"],   # màu và hình khác nhau từng ngăn (S5: người mù màu)
        "sym_floor": 0.05, "sym_front": 0.022}
SHAPE_LABEL = {"tron": "Tròn", "vuong": "Vuông", "tam_giac": "Tam giác"}
# lid_start: góc nắp lúc đầu (rad, 0 = đóng); MJCF dùng đúng góc này (ref + quay body), web đọc box.lid.start
# S1C (P-L): hộp thấp và ngắn hơn (0,16 x 0,20 x 0,08 -> 0,16 x 0,16 x 0,05, núm cách bản lề 0,14 -> 0,11) để núm khi mở nắp không lên quá
# ~0,92 m: kẹp chĩa xuống của P-L (TCP kẹp cũ dài) ở lift 0,50 chỉ tới z ~0,93
BOX = {"center": (TABLE_EDGE_X + 0.15, -0.58),   # S1G: y -0,75 -> -0,58 (khay trước của đế P-L chạm chân bàn khi đứng ở y -0,94)
       "size": (0.16, 0.16, 0.05), "wall": 0.006, "lid_t": 0.008, "lid_start": 0.0, "knob_dy": 0.08}   # S1G: núm 0,11 -> 0,08 (núm khi mở nắp <= 0,88 m: trục vai P-L đỉnh 1,012 m)
# ngân sách tam giác khi web dựng cảnh bằng hình khối three.js (không tính robot m1.glb)
TRI = {"box": 12, "cylinder": 128, "sphere": 960}   # BoxGeometry; CylinderGeometry 32 cạnh; SphereGeometry 32x16
LAB_MAX_TRIANGLES = 20000
CAB = {"front_x": TABLE_EDGE_X + 0.24, "center_y": 0.85, "size": (0.26, 0.30, 0.16), "wall": 0.01, "travel": 0.13}   # S1C P-L: cao 0,25 -> 0,20; S1G: 0,20 -> 0,16 (tay nắm ngăn trên 0,895 -> 0,865 m: trục vai P-L đỉnh 1,012 m)
FRIDGE = {"front_x": TABLE_EDGE_X, "center_y": -1.40, "size": (0.50, 0.50, 0.95), "door_t": 0.04, "stand_z": 0.25}   # S1C P-L: cao 0,85 -> 0,95 (khoang trên kệ 7 -> 17 cm: kẹp cũ cao ~11 cm)
# S1G: tủ lạnh mini đặt trên giá 4 chân cao 0,25 m (độ cao kệ giữ nguyên): khay trước (mép x 0,32 m, mặt 0,21 m) và bánh trước của đế P-L chui dưới tủ


def drawer_handle_z(i: int) -> float:
    lo = TABLE_H + CAB["wall"]
    mid = TABLE_H + CAB["size"][2] / 2
    hi = TABLE_H + CAB["size"][2] - CAB["wall"]
    return (lo + mid) / 2 if i == 0 else (mid + hi) / 2


# ---------------- mô tả cảnh (dùng chung MJCF + JSON) ----------------
# item = {id, label, category, links: [link]}; link = {name, parent, pos (khung cha), joint|None, geoms, free?}
# geom = {shape: box|cylinder|sphere, size: đầy đủ (box: [lx,ly,lz]; cylinder: [r,h] trục z; sphere: [r]), pos, color, collide, euler?}

def G(shape, size, pos, color, collide=True, **kw):
    d = {"shape": shape, "size": [round(float(s), 5) for s in size], "pos": [round(float(p), 5) for p in pos],
         "color": C.get(color, color), "collide": collide}
    d.update(kw)
    return d


def table():
    x0, y0, y1 = TABLE_EDGE_X, *TABLE_Y
    cx, cy, ly = x0 + TABLE_DEPTH / 2, (y0 + y1) / 2, y1 - y0
    geoms = [G("box", (TABLE_DEPTH, ly, TOP_T), (0, 0, TABLE_H - TOP_T / 2), "alu")]
    leg_h = TABLE_H - TOP_T
    for i, (sx, sy) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)]):
        geoms.append(G("box", (0.04, 0.04, leg_h), (sx * (TABLE_DEPTH / 2 - 0.04), sy * (ly / 2 - 0.04), leg_h / 2), "alu"))
    geoms.append(G("box", (0.004, ly, 0.03), (-TABLE_DEPTH / 2 + 0.002, 0, TABLE_H - TOP_T - 0.015), "navy"))
    return {"id": "table", "label": "Bàn thí nghiệm 0,75 m", "category": "furniture",
            "links": [{"name": "table", "parent": None, "pos": [cx, cy, 0], "joint": None, "geoms": geoms}]}


def objects(seed: int | None = None, can_jitter: bool = False):
    import random
    rnd = random.Random(seed) if seed is not None else None
    out = []
    for k, (oid, shape, size, color) in enumerate(OBJECTS):
        h = size[2] if shape == "box" else size[1]
        x, y = TABLE_EDGE_X + ROW_D, OBJ_Y0 + OBJ_DY * k
        yaw = 0.0
        if rnd:
            x += rnd.uniform(-0.01, 0.01); y += rnd.uniform(-0.008, 0.008); yaw = rnd.uniform(-0.2, 0.2)
        g = G(shape, size, (0, 0, 0), color, mass=OBJ_MASS)
        out.append({"id": oid, "label": f"Vật {k + 1}", "category": "object", "color_class": color, "shape": shape,
                    "links": [{"name": oid, "parent": None, "pos": [x, y, TABLE_H + h / 2 + 0.0005], "yaw": yaw,
                               "joint": {"type": "free"}, "geoms": [g]}]})
    oid, shape, size, color = FRIDGE_OBJ
    cx, cy = FRIDGE_OBJ_XY
    cyaw = 0.0
    if rnd and can_jitter:   # lon trong tủ xáo theo seed khi chạy demo/đo (±1 cm, ±0,2 rad; lab_common.scatter). Rút sau các vật trên bàn nên
                             # vật trên bàn theo seed không đổi; bố cục ghi trong lab.xml/lab-scene.json (objects(SEED)) giữ lon ở chỗ gốc
        cx += rnd.uniform(-0.01, 0.01); cy += rnd.uniform(-0.01, 0.01); cyaw = rnd.uniform(-0.2, 0.2)
    out.append({"id": oid, "label": "Lon trong tủ lạnh", "category": "object", "color_class": color, "shape": shape,
                "inside": "fridge",
                "links": [{"name": oid, "parent": None, "pos": [cx, cy, FRIDGE_SHELF_Z + size[1] / 2 + 0.0005],
                           **({"yaw": cyaw} if rnd and can_jitter else {}),
                           "joint": {"type": "free"}, "geoms": [G(shape, size, (0, 0, 0), color, mass=OBJ_MASS)]}]})
    return out


def pad():
    x, y = ZONE["pos"]
    lx, ly = ZONE["size"]
    return {"id": "target_zone", "label": "Vùng đích gắp đặt", "category": "zone", "size": [lx, ly],
            "links": [{"name": "target_zone", "parent": None, "pos": [x, y, TABLE_H], "joint": None,
                       "geoms": [G("box", (lx, ly, 0.002), (0, 0, 0.001), "navy", collide=False)]}]}


def symbol(shape: str, s: float, plane: str, c, color: str) -> list:
    """Ký hiệu hình phẳng không va chạm, cạnh ~s: tron = đĩa, vuong = ô vuông, tam_giac = viền tam giác đều 3 thanh (MuJoCo không có tam giác).
    plane "xy": nằm trên đáy, đỉnh tam giác hướng +x (xa robot); "yz": dựng trên mặt trước, nhìn từ robot (-x), đỉnh hướng lên."""
    th = 0.001 if plane == "xy" else 0.002
    cx, cy, cz = c
    if shape == "tron":
        return [G("cylinder", (s / 2, th), c, color, collide=False, **({} if plane == "xy" else {"euler": [0, math.pi / 2, 0]}))]
    if shape == "vuong":
        a = s * 0.86
        return [G("box", (a, a, th) if plane == "xy" else (th, a, a), c, color, collide=False)]
    if shape != "tam_giac":
        raise ValueError(shape)
    b = s * 0.2
    L = s + b * 0.9                               # kéo dài để góc khép kín
    r = s / (2 * math.sqrt(3))                    # tâm tới cạnh
    out = []
    for k in range(3):                            # cạnh đáy rồi hai cạnh bên; (u ngang, v lên/xa) trong mặt phẳng ký hiệu
        a = math.radians(-90 + 120 * k)           # hướng pháp tuyến cạnh
        mu, mv = r * math.cos(a), r * math.sin(a)
        du, dv = -math.sin(a), math.cos(a)        # hướng dọc cạnh
        if plane == "xy":                         # u -> -y (phải robot), v -> +x
            out.append(G("box", (L, b, th), (cx + mv, cy - mu, cz), color, collide=False, euler=[0, 0, round(math.atan2(-du, dv), 6)]))
        else:                                     # u -> -y, v -> +z
            out.append(G("box", (th, L, b), (cx, cy - mu, cz + mv), color, collide=False, euler=[round(math.atan2(dv, -du), 6), 0, 0]))
    return out


def tray():
    cx, cy = TRAY["center"]
    w, d, t, h = TRAY["cell_w"], TRAY["cell_d"], TRAY["wall"], TRAY["h"]
    n = len(TRAY["colors"])
    ly = n * w + (n + 1) * t
    lx = d + 2 * t
    geoms = [G("box", (lx, ly, 0.004), (0, 0, 0.002), "slate")]
    for sx in (-1, 1):
        geoms.append(G("box", (t, ly, h), (sx * (lx - t) / 2, 0, h / 2), "slate"))
    for i in range(n + 1):
        geoms.append(G("box", (d, t, h), (0, ly / 2 - t / 2 - i * (w + t), h / 2), "slate"))
    cells = []
    xf = -(lx - t) / 2 - 0.004                       # tâm nhãn mặt trước (phía robot)
    for i, (col, shp) in enumerate(zip(TRAY["colors"], TRAY["shapes"])):
        yc = ly / 2 - t - w / 2 - i * (w + t)
        geoms.append(G("box", (d, w, 0.001), (0, yc, 0.0045), col, collide=False))      # đáy màu
        geoms += symbol(shp, TRAY["sym_floor"], "xy", (0, yc, 0.0055), "white")       # hình trên đáy (nhìn từ trên)
        geoms.append(G("box", (0.004, w * 0.8, h - 0.01), (xf, yc, h / 2 + 0.002), col, collide=False))  # nhãn màu mặt trước
        geoms += symbol(shp, TRAY["sym_front"], "yz", (xf - 0.003, yc, h / 2 + 0.002), "white")      # hình trên nhãn
        cells.append({"color_class": col, "shape": shp, "center": [round(cx, 5), round(cy + yc, 5), TABLE_H],
                      "size": [d, w]})
    return {"id": "sort_tray", "label": "Khay phân loại 3 ngăn", "category": "fixture", "cells": cells,
            "links": [{"name": "sort_tray", "parent": None, "pos": [cx, cy, TABLE_H], "joint": None, "geoms": geoms}]}


def lid_box():
    cx, cy = BOX["center"]
    lx, ly, lz = BOX["size"]
    t, lt = BOX["wall"], BOX["lid_t"]
    body = [G("box", (lx, ly, t), (0, 0, t / 2), "navy")]
    for sx in (-1, 1):
        body.append(G("box", (t, ly, lz), (sx * (lx - t) / 2, 0, lz / 2), "navy"))
    for sy in (-1, 1):
        body.append(G("box", (lx - 2 * t, t, lz), (0, sy * (ly - t) / 2, lz / 2), "navy"))
    # bản lề dọc cạnh +y, trục -x: nắp mở thì mép -y nâng lên
    hy = ly / 2
    lid = [G("box", (lx + 0.004, ly + 0.004, lt), (0, -ly / 2, lt / 2), "ivory"),
           G("cylinder", (0.005, 0.02), (0, -BOX["knob_dy"], lt + 0.01), "slate_dk"),
           G("sphere", (0.012,), (0, -BOX["knob_dy"], lt + 0.03), "slate_dk")]
    return {"id": "lid_box", "label": "Hộp có nắp bản lề", "category": "articulated",
            "links": [{"name": "lid_box", "parent": None, "pos": [cx, cy, TABLE_H], "joint": None, "geoms": body},
                      {"name": "lid_box_lid", "parent": "lid_box", "pos": [0, hy, lz],
                       "joint": {"name": "lid_hinge", "type": "hinge", "axis": [-1, 0, 0], "range": [0.0, 1.75], "start": BOX["lid_start"],
                                 "damping": 0.05, "frictionloss": 0.01, "open_min": 1.2, "closed_max": 0.09},
                       "geoms": lid, "knob": [0, -BOX["knob_dy"], lt + 0.03]}]}


def cabinet():
    dx, wy, hz = CAB["size"]
    t = CAB["wall"]
    cx = CAB["front_x"] + dx / 2
    cy = CAB["center_y"]
    body = [G("box", (dx, wy, t), (0, 0, t / 2), "navy"), G("box", (dx, wy, t), (0, 0, hz - t / 2), "navy"),
            G("box", (dx, wy, t), (0, 0, hz / 2), "navy"), G("box", (t, wy, hz), (dx / 2 - t / 2, 0, hz / 2), "navy")]
    for sy in (-1, 1):
        body.append(G("box", (dx, t, hz), (0, sy * (wy - t) / 2, hz / 2), "navy"))
    links = [{"name": "drawer_cabinet", "parent": None, "pos": [cx, cy, TABLE_H], "joint": None, "geoms": body}]
    in_w = wy - 2 * t - 0.006
    for i in range(2):
        zc = drawer_handle_z(i) - TABLE_H
        fh = hz / 2 - 1.5 * t - 0.004
        dd = dx - t - 0.02
        g = [G("box", (0.012, wy - 0.006, fh), (-dx / 2 + 0.006, 0, 0), "ivory"),
             G("box", (dd, in_w, 0.006), (-dx / 2 + dd / 2, 0, -fh / 2 + 0.01), "ivory"),
             G("box", (0.006, in_w, fh - 0.03), (-dx / 2 + dd, 0, -fh / 2 + (fh - 0.03) / 2 + 0.01), "ivory")]
        for sy in (-1, 1):
            g.append(G("box", (dd, 0.006, fh - 0.03), (-dx / 2 + dd / 2, sy * (in_w - 0.006) / 2, -fh / 2 + (fh - 0.03) / 2 + 0.01), "ivory"))
        # tay nắm: thanh ngang dọc y, hai chân cách mặt trước 40 mm
        g.append(G("cylinder", (0.006, 0.10), (-dx / 2 - 0.04, 0, 0), "slate_dk", euler=[math.pi / 2, 0, 0]))
        for sy in (-1, 1):
            g.append(G("box", (0.04, 0.008, 0.008), (-dx / 2 - 0.02, sy * 0.045, 0), "slate_dk"))
        links.append({"name": f"drawer_{i}", "parent": "drawer_cabinet", "pos": [0, 0, zc],
                      "joint": {"name": f"drawer_{i}_slide", "type": "slide", "axis": [-1, 0, 0], "range": [0.0, CAB["travel"]], "start": 0.0,
                                "damping": 3.0, "frictionloss": 0.3, "open_min": 0.08, "closed_max": 0.01},
                      "geoms": g, "handle": [-dx / 2 - 0.04, 0, 0]})
    return {"id": "drawer_cabinet", "label": "Tủ 2 ngăn kéo", "category": "articulated", "links": links}


def fridge():
    dx, wy, hz = FRIDGE["size"]
    t = 0.03
    cx = FRIDGE["front_x"] + dx / 2
    cy = FRIDGE["center_y"]
    z0 = FRIDGE["stand_z"]
    body = [G("box", (dx, wy, t), (0, 0, z0 + t / 2), "white"), G("box", (dx, wy, t), (0, 0, hz - t / 2), "white"),
            G("box", (t, wy, hz - z0), (dx / 2 - t / 2, 0, z0 + (hz - z0) / 2), "white"),
            G("box", (dx - 0.06, wy - 0.06, 0.012), (0.01, 0, FRIDGE_SHELF_Z - 0.006), "tint"),
            G("box", (dx - 0.06, wy - 0.06, 0.012), (0.01, 0, 0.45), "tint")]
    for sx in (-1, 1):
        for sy in (-1, 1):
            body.append(G("box", (0.03, 0.03, z0), (sx * (dx - 0.03) / 2, sy * (wy - 0.03) / 2, z0 / 2), "slate"))
    for sy in (-1, 1):
        body.append(G("box", (dx, t, hz - z0), (0, sy * (wy - t) / 2, z0 + (hz - z0) / 2), "white"))
    dt = FRIDGE["door_t"]
    hinge = [-dx / 2 - dt / 2, -wy / 2 + 0.005, 0]
    door = [G("box", (dt, wy - 0.01, hz - z0 - 0.01), (0, (wy - 0.01) / 2, z0 + 0.005 + (hz - z0 - 0.01) / 2), "ivory", mass=2.5),
            G("box", (0.004, wy * 0.6, 0.012), (-dt / 2 - 0.002, (wy - 0.01) / 2, hz - 0.08), "navy", collide=False),
            # tay nắm đứng: thanh tròn, hai chân phía dưới, đầu trên tự do để kẹp chĩa xuống ôm từ trên
            G("cylinder", (0.008, 0.18), (-dt / 2 - 0.05, 0.44, 0.71), "slate_dk")]
    for z in (0.64, 0.70):
        door.append(G("box", (0.05, 0.014, 0.014), (-dt / 2 - 0.025, 0.44, z), "slate_dk"))
    return {"id": "fridge", "label": "Tủ lạnh mini", "category": "articulated",
            "links": [{"name": "fridge", "parent": None, "pos": [cx, cy, 0], "joint": None, "geoms": body},
                      {"name": "fridge_door", "parent": "fridge", "pos": hinge,
                       "joint": {"name": "fridge_hinge", "type": "hinge", "axis": [0, 0, 1], "range": [0.0, 1.75], "start": 0.0,
                                 "damping": 1.0, "frictionloss": 0.2, "open_min": 1.2, "closed_max": 0.05},
                       "geoms": door, "handle": [-dt / 2 - 0.05, 0.44, 0.775]}]}


def room():
    return {"id": "room", "label": "Phòng", "category": "room",
            "links": [{"name": "room", "parent": None, "pos": [0, 0, 0], "joint": None,
                       "geoms": [G("box", (6.0, 6.0, 0.01), (0.5, 0, -0.005), "slate"),
                                 G("box", (0.05, 6.0, 1.6), (2.0, 0, 0.8), "white"),
                                 G("box", (6.0, 0.05, 1.6), (0.5, 2.4, 0.8), "white")]}]}


SEED = 0   # bố cục ghi trong lab.xml/lab-scene.json = objects(SEED); demo --seed khác thì xáo lại trước khi chạy
PICK_PLACE_OBJS = ["obj_red_cube", "obj_blue_cyl", FRIDGE_OBJ[0]]
TRAY_LABEL = {"red": "Đỏ", "green": "Xanh lá", "blue": "Xanh dương"}


def _full_size(shape, size):
    return list(size) if shape == "box" else ([2 * size[0], 2 * size[0], size[1]] if shape == "cylinder" else [2 * size[0]] * 3)


def _jflat(j):
    return {k: j[k] for k in ("name", "type", "axis", "range", "start", "open_min", "closed_max")}


def _world(link_pos, rel):
    return [round(float(a + b), 5) for a, b in zip(link_pos, rel)]


def contract(items) -> dict:
    """Khoá phẳng theo hợp đồng check:m1-assets của web (seed, fridge, cabinet, box, trays, objects, tasks). Toạ độ thế giới."""
    by = {it["id"]: it for it in items}
    fr, cab, bx, tr = by["fridge"], by["drawer_cabinet"], by["lid_box"], by["sort_tray"]
    fb, fd = fr["links"]
    door_hinge = _world(fb["pos"], fd["pos"])
    fridge = {"id": "fridge", "size": list(FRIDGE["size"]), "position": fb["pos"],
              "door": {"joint": _jflat(fd["joint"]), "hinge": door_hinge, "handle": _world(door_hinge, fd["handle"])}}
    cb = cab["links"][0]
    drawers = []
    for ln in cab["links"][1:]:
        o = _world(cb["pos"], ln["pos"])
        drawers.append({"id": ln["name"], "joint": _jflat(ln["joint"]), "handle": _world(o, ln["handle"])})
    cabinet = {"id": "drawer_cabinet", "size": list(CAB["size"]), "position": cb["pos"], "drawers": drawers}
    bb, bl = bx["links"]
    lh = _world(bb["pos"], bl["pos"])
    box = {"id": "lid_box", "size": list(BOX["size"]), "position": bb["pos"],
           "lid": {"joint": _jflat(bl["joint"]), "start": bl["joint"]["start"], "hinge": lh, "knob": _world(lh, bl["knob"])}}
    trays = [{"id": f"tray_{c['color_class']}", "color": C[c["color_class"]], "color_class": c["color_class"],
              "label": TRAY_LABEL[c["color_class"]], "shape": c["shape"], "shape_label": SHAPE_LABEL[c["shape"]], "size": [c["size"][0], c["size"][1], TRAY["h"]], "position": c["center"]}
             for c in tr["cells"]]
    objs = []
    for it in items:
        if it["category"] != "object":
            continue
        ln = it["links"][0]
        o = {"id": it["id"], "label": it["label"], "color": C[it["color_class"]], "color_class": it["color_class"],
             "shape": it["shape"], "size": _full_size(it["shape"], ln["geoms"][0]["size"]), "position": ln["pos"],
             "yaw": ln.get("yaw", 0.0), "mass": OBJ_MASS, "tray": f"tray_{it['color_class']}"}
        if it.get("inside"):
            o["inside"] = it["inside"]
        objs.append(o)
    zx, zy = ZONE["pos"]
    tasks = {
        "pick_place": {"k": len(PICK_PLACE_OBJS), "objects": PICK_PLACE_OBJS, "target": "target_zone",
                       "target_zone": {"position": [zx, zy, TABLE_H], "size": list(ZONE["size"])},
                       "script": "demos/lab_pick_place.py", "run": "./run.sh --lab-pick",
                       "done_when": "mỗi vật có tâm trong vùng target_zone, đáy trên mặt bàn ±1,5 cm, |v| <= 0,02 sau 1 s; vật trong tủ lạnh lấy ra sau khi mở cửa"},
        "sorting": {"objects": [o[0] for o in OBJECTS], "script": "demos/lab_sorting.py", "run": "./run.sh --lab-sort",
                    "done_when": "mỗi vật có tâm trong ngăn trays[].id = objects[].tray, đáy trên đáy khay ±1,5 cm, đứng yên"},
        "open_close": {"targets": ["drawer_1", "lid_box", "fridge"], "script": "demos/lab_open_close.py", "run": "./run.sh --lab-open",
                       "fridge_open_min_rad": fd["joint"]["open_min"],
                       "done_when": "mỗi khớp mở tới >= open_min rồi đóng về <= closed_max, đo sau khi nhả kẹp 1 s"},
    }
    return {"fridge": fridge, "cabinet": cabinet, "box": box, "trays": trays, "objects": objs, "tasks": tasks}


def scene(seed: int | None = SEED) -> dict:
    items = [room(), table(), pad(), tray(), lid_box(), cabinet(), fridge()] + objects(seed)
    for it in items:   # trường chung cho trình dựng hình khối: name, type, pos (gốc link đầu)
        it["name"] = it["id"]
        it["type"] = it["category"]
        it["pos"] = [round(float(v), 5) for v in it["links"][0]["pos"]]
    out = {
        "schema": "faibot-m1-lab-scene/2",
        "generator": "sim/lab/build_lab.py",
        "mjcf": "sim/lab/lab.xml",
        "seed": SEED if seed is None else seed,
        "units": {"length": "m", "angle": "rad", "mass": "kg",
                  "frame": "z lên, x từ chỗ robot đứng lúc đầu vào bàn, y sang trái robot; three.js (y lên): (x, z, -y)",
                  "sizes": "size = kích thước đầy đủ [lx, ly, lz]; trong items[].links[].geoms: box [lx,ly,lz], cylinder [bán kính, cao] trục z, sphere [bán kính]",
                  "position": "fridge/cabinet/box/table: tâm đáy; trays: tâm đáy ngăn; objects: tâm vật",
                  "geom_pos": "items: geom.pos theo khung link; link.pos theo khung link cha (link gốc: thế giới); link.yaw quanh z",
                  "geom_euler": "geom.euler (nếu có) xyz nội tại, rad",
                  "joint_start": "joint.start / box.lid.start: góc (rad) hoặc khoảng trượt (m) lúc đầu, MJCF qpos0 = ref = start; 0 = đóng. "
                                 "items[].links[].geoms, handle, knob, hinge vẽ ở trạng thái 0; trạng thái start = quay/dời quanh khớp theo joint.axis"},
        "robot": {"mjcf": "sim/m1.xml", "start": [0, 0, 0], "work_standoff_from_table_m": ROBOT_STANDOFF,
                  "lift_work_m": LIFT_WORK, "gripper_max_gap_m": 0.0973},   # kẹp AM-ARM200, khoảng cách hai đầu ngón khi mở mềm (spec.GRIPPER_MAX_GAP; kẹp cũ: 0,084)
        "table": {"height": TABLE_H, "edge_x": TABLE_EDGE_X, "depth": TABLE_DEPTH, "y_range": list(TABLE_Y),
                  "size": [TABLE_DEPTH, TABLE_Y[1] - TABLE_Y[0], TABLE_H], "position": [TABLE_EDGE_X + TABLE_DEPTH / 2, 0, 0]},
        "palette": C,
    }
    out.update(contract(items))
    out["items"] = items
    tri = sum(TRI[g["shape"]] for it in items for ln in it["links"] for g in ln["geoms"])
    assert tri <= LAB_MAX_TRIANGLES, tri
    out["max_triangles"] = LAB_MAX_TRIANGLES
    out["budget"] = {"max_triangles": LAB_MAX_TRIANGLES, "triangles_est": tri,
                     "assume": "box 12, cylinder 128 (32 cạnh), sphere 960 (32x16) tam giác mỗi geom; không tính robot m1.glb"}
    return out


# ---------------- MJCF ----------------

def _rgba(h: str, a: float = 1.0) -> str:
    h = h.lstrip("#")
    return " ".join(f"{int(h[i:i + 2], 16) / 255:.4g}" for i in (0, 2, 4)) + f" {a:g}"


def _f(v) -> str:
    return " ".join(f"{float(x):.5g}" for x in v)


def _geom(g, name: str, cls: str) -> str:
    s = g["size"]
    if g["shape"] == "box":
        size = _f([x / 2 for x in s])
    elif g["shape"] == "cylinder":
        size = _f([s[0], s[1] / 2])
    else:
        size = _f(s)
    extra = ""
    if "euler" in g:
        extra += f' euler="{_f(g["euler"])}"'
    if "mass" in g:
        extra += f' mass="{g["mass"]}"'
    if not g["collide"]:
        extra += ' contype="0" conaffinity="0"'
    return f'<geom name="{name}" class="{cls}" type="{g["shape"]}" size="{size}" pos="{_f(g["pos"])}" rgba="{_rgba(g["color"])}"{extra}/>'


def _link_xml(item, link, children, depth=2) -> str:
    ind = "  " * depth
    # lớp va chạm: thân đồ đạc và phần động (ngăn kéo/cửa/nắp) không va chạm nhau, cả hai va chạm robot và vật
    cls = {"object": "lab_obj", "articulated": "lab_fix"}.get(item["category"], "lab_fix")
    if link["parent"] is not None:
        cls = "lab_moving"
    pos = _f(link["pos"])
    yaw = link.get("yaw", 0.0)
    rot = f' euler="0 0 {yaw:.5g}"' if yaw else ""
    j = link["joint"]
    st = float(j.get("start", 0.0)) if j else 0.0
    if st and j["type"] == "hinge":   # vẽ body ở góc start, ref = start => qpos0 = start, qpos 0 vẫn là đóng
        assert not yaw
        ax = [a / math.sqrt(sum(x * x for x in j["axis"])) for a in j["axis"]]
        rot = f' quat="{_f([math.cos(st / 2)] + [a * math.sin(st / 2) for a in ax])}"'
    elif st and j["type"] == "slide":
        pos = _f([p + a * st for p, a in zip(link["pos"], j["axis"])])
    lines = [f'{ind}<body name="{link["name"]}" pos="{pos}"{rot}>']
    if j and j["type"] == "free":
        lines.append(f'{ind}  <freejoint name="{link["name"]}"/>')
    elif j:
        lines.append(f'{ind}  <joint name="{j["name"]}" type="{j["type"]}" axis="{_f(j["axis"])}" range="{_f(j["range"])}" ref="{st:g}" '
                     f'damping="{j["damping"]}" frictionloss="{j["frictionloss"]}" armature="0.01"/>')
    for i, g in enumerate(link["geoms"]):
        lines.append(f"{ind}  " + _geom(g, f'{link["name"]}_g{i}', cls))
    for key in ("handle", "knob"):
        if key in link:
            lines.append(f'{ind}  <site name="{link["name"]}_{key}" pos="{_f(link[key])}" size="0.006" rgba="1 1 1 0"/>')
    for ch in children.get(link["name"], []):
        lines.append(_link_xml(item, ch, children, depth + 1))
    lines.append(f"{ind}</body>")
    return "\n".join(lines)


def to_mjcf(sc: dict) -> str:
    bodies = []
    for item in sc["items"]:
        children = {}
        for ln in item["links"]:
            if ln["parent"]:
                children.setdefault(ln["parent"], []).append(ln)
        for ln in item["links"]:
            if ln["parent"] is None:
                bodies.append(_link_xml(item, ln, children))
    cams = {
        "lab_overview": ((-0.9, -1.6, 1.9), (0.55, -0.25, 0.7)),
        "lab_table": ((0.05, -0.95, 1.45), (0.62, 0.0, 0.78)),
        "lab_sort": ((0.0, 0.25, 1.5), (0.62, -0.2, 0.78)),
        "lab_fridge": ((-0.9, -0.4, 1.55), (0.3, -1.45, 0.6)),
        "lab_cabinet": ((0.1, 1.5, 1.35), (0.62, 0.82, 0.9)),
        "lab_fridge_in": ((-0.25, -0.75, 1.15), (0.55, -1.3, 0.76)),
        "lab_front": ((1.55, -0.45, 1.45), (0.45, 0.02, 0.8)),
        "lab_front_sort": ((1.55, -0.75, 1.5), (0.45, -0.15, 0.78)),
    }
    sys.path.insert(0, str(HERE.parents[1] / "sdk"))
    from faibotic.model import lookat_xyaxes
    cam_xml = "\n".join(f'    <camera name="{n}" pos="{_f(p)}" xyaxes="{lookat_xyaxes(p, t)}" fovy="55"/>' for n, (p, t) in cams.items())
    body_xml = "\n".join(bodies)
    return f"""<mujoco model="faibotic_m1_lab">
  <!-- Phòng lab M1. File sinh tự động bởi sim/lab/build_lab.py (cùng nguồn với lab-scene.json); sửa build_lab.py rồi chạy lại. -->
  <include file="../m1.xml"/>
  <compiler meshdir="../assets"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096" offsamples="4"/>
    <headlight ambient="0.12 0.12 0.12" diffuse="0.15 0.15 0.15" specular="0.02 0.02 0.02"/></visual>
  <default>
    <default class="lab_fix"><geom contype="2" conaffinity="1" friction="1 0.01 0.001" solref="0.004 1"/></default>
    <default class="lab_moving"><geom contype="4" conaffinity="1" friction="1 0.01 0.001" solref="0.004 1" density="300"/></default>
    <default class="lab_obj"><geom contype="1" conaffinity="1" condim="4" friction="1.2 0.02 0.001" solref="0.004 1" solimp="0.95 0.99 0.001"/></default>
  </default>
  <worldbody>
    <light pos="0.5 -1.0 2.6" dir="0.1 0.3 -1" directional="false" diffuse="0.12 0.12 0.12" castshadow="false" attenuation="1 0 0"/>
{cam_xml}
{body_xml}
  </worldbody>
</mujoco>
"""


def write(check: bool = False) -> int:
    sc = scene()
    files = {HERE / "lab.xml": to_mjcf(sc), HERE / "lab-scene.json": json.dumps(sc, ensure_ascii=False, indent=1) + "\n"}
    bad = 0
    for p, txt in files.items():
        if check:
            if not p.exists() or p.read_text() != txt:
                print(f"lệch: {p}")
                bad = 1
        else:
            p.write_text(txt)
            print(f"ghi {p}")
    return bad


if __name__ == "__main__":
    raise SystemExit(write(check="--check" in sys.argv))
