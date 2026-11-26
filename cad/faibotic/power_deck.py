# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Đế nguồn: khay pin 3S, giá PCB nguồn, bảng E-stop. build123d."""
from build123d import Box, Cylinder, Pos, Rot, Align
from params import *

MIN = (Align.MIN, Align.MIN, Align.MIN)
INSERT_D, INSERT_DEPTH = 4.0, 6.5   # v0.4.3: lỗ hạt chèn nhiệt M3 [cờ: kiến thức chung, lỗ ~Ø4,0 cho hạt OD ~4,6; hạt M1-AR-012 chưa chọn, sửa theo datasheet]
T = 4.0      # bề dày tấm đế
WALL = 3.0


def deck():
    bl, bw, bh = BATT
    # khay pin (x 0..bl+2c), PCB bên phải, E-stop bảng đứng ở mép trước
    gap = 0.5
    tray_x, tray_y = bl + 2 * gap, bw + 2 * gap
    pcb_x0 = tray_x + 2 * WALL + 10
    total_x = pcb_x0 + PCB[0] + 10
    total_y = max(tray_y + 2 * WALL, PCB[1] + 20) + 0.0
    base = Box(total_x, total_y, T, align=MIN)
    # tường khay pin (hở 2 đầu để ra cáp, cao 25 mm)
    ty0 = (total_y - tray_y) / 2
    walls = Pos(0, ty0 - WALL, T) * Box(tray_x + 2 * WALL, WALL, 25, align=MIN)
    walls += Pos(0, ty0 + tray_y, T) * Box(tray_x + 2 * WALL, WALL, 25, align=MIN)
    walls += Pos(0, ty0 - WALL, T) * Box(WALL, tray_y + 2 * WALL, 25, align=MIN)
    walls += Pos(tray_x + WALL, ty0 - WALL, T) * Box(WALL, tray_y + 2 * WALL, 25, align=MIN)
    # trụ đỡ PCB 4 góc, cao 8 mm, lỗ M3
    posts = None
    for dx in (4, PCB[0] - 4):
        for dy in (4, PCB[1] - 4):
            p = Pos(pcb_x0 + dx, (total_y - PCB[1]) / 2 + dy, T) * Cylinder(4.5, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))   # v0.4.3: Ø9 (Ø6 -> thành 0,9 mm)
            p = p - Pos(pcb_x0 + dx, (total_y - PCB[1]) / 2 + dy, T + 8.0 - INSERT_DEPTH) * Cylinder(INSERT_D / 2, INSERT_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN))
            posts = p if posts is None else posts + p
    # bảng E-stop: tấm đứng dọc mép sau (y = total_y), lỗ Ø22
    panel_h = 52 + ESTOP_HEAD / 2 + 8
    panel = Pos(pcb_x0, total_y, 0) * Box(PCB[0], 4.0, T + panel_h, align=MIN)
    panel = panel - Pos(pcb_x0 + PCB[0] / 2, total_y, T + 52) * Rot(X=-90) * Cylinder(ESTOP_HOLE / 2, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    d = base + walls + posts + panel
    meta = dict(total_x=total_x, total_y=total_y, tray=(ty0, tray_x, tray_y), pcb_x0=pcb_x0, panel_h=panel_h)
    return d, meta


def dummies(meta):
    bl, bw, bh = BATT
    ty0 = meta['tray'][0]
    batt = Pos(0.5 + WALL, ty0 + 0.5, T) * Box(bl, bw, bh, align=MIN)
    pcb = Pos(meta['pcb_x0'], (meta['total_y'] - PCB[1]) / 2, T + 8.0) * Box(PCB[0], PCB[1], PCB[2], align=MIN)
    pcb_comp = Pos(meta['pcb_x0'] + 5, (meta['total_y'] - PCB[1]) / 2 + 5, T + 8.0 + PCB[2]) * Box(PCB[0] - 10, PCB[1] - 10, PCB_COMP_H, align=MIN)
    zc = T + 52
    xc = meta['pcb_x0'] + PCB[0] / 2
    # thân E-stop Ø22 xuyên lỗ phía trong (y < total_y) dài 35, đầu nấm Ø40 dày 20 phía ngoài
    estop_body = Pos(xc, meta['total_y'] - 35 + 4.0, zc) * Rot(X=-90) * Cylinder(ESTOP_HOLE / 2 - 0.4, 35 + 0.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    estop_head = Pos(xc, meta['total_y'] + 4.0, zc) * Rot(X=-90) * Cylinder(ESTOP_HEAD / 2, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return dict(battery=batt, pcb=pcb, pcb_components=pcb_comp, estop_body=estop_body, estop_head=estop_head)
