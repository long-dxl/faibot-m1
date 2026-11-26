# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Gá vai ghép đôi 2 x STS3095 (đồng trục, hai bên tấm link). build123d."""
import math
from build123d import Box, Cylinder, Pos, Rot, Align, Part, Location, Polygon, extrude
from params import *

C = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN = (Align.MIN, Align.MIN, Align.MIN)


def servo(side: int) -> Part:
    """side=-1: servo bên y<0; +1: y>0. Trục ra hướng vào tấm link, trục quay là trục y qua gốc."""
    y_in = (LINK_T / 2 + HORN_T)                      # mặt thân sát horn
    y0 = y_in if side > 0 else -y_in - SERVO_W
    body = Pos(-(SERVO_L - SHAFT_FROM_END), y0, -SERVO_H / 2) * Box(SERVO_L, SERVO_W, SERVO_H, align=MIN)
    hy0 = LINK_T / 2 if side > 0 else -LINK_T / 2 - HORN_T
    horn = Pos(0, hy0, 0) * Rot(X=-90) * Cylinder(HORN_R, HORN_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body + horn


def cradle() -> Part:
    y_out = LINK_T / 2 + HORN_T + SERVO_W + CLEAR     # mặt trong má
    x_back = -(SERVO_L - SHAFT_FROM_END) - CLEAR      # mặt trong tường sau
    cheeks = None
    for s in (-1, 1):
        y0 = y_out if s > 0 else -y_out - CHEEK_T
        c = Pos(x_back - BACK_T, y0, -CRADLE_H / 2) * Box(SERVO_L + CLEAR * 2 + BACK_T, CHEEK_T, CRADLE_H, align=MIN)
        bore = Pos(0, y0, 0) * Rot(X=-90) * Cylinder(BORE_OUT / 2, CHEEK_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
        c = c - bore
        # v0.4.3 (DFM): lỗ giọt nước, đỉnh 45 độ (z+) để in đế xuống bàn không cần support nhịp cầu 16 mm
        r = BORE_OUT / 2
        tear = Pos(0, y0, 0) * Rot(X=-90) * extrude(Polygon((0, -r * math.sqrt(2)), (r / math.sqrt(2), -r / math.sqrt(2)), (-r / math.sqrt(2), -r / math.sqrt(2)), align=None), amount=CHEEK_T)
        c = c - tear
        cheeks = c if cheeks is None else cheeks + c
    width = 2 * (y_out + CHEEK_T)
    back = Pos(x_back - BACK_T, -width / 2, -CRADLE_H / 2) * Box(BACK_T, width, CRADLE_H, align=MIN)
    foot = Pos(x_back - BACK_T - FOOT_LEN, -width / 2, -CRADLE_H / 2) * Box(FOOT_LEN + BACK_T, width, FOOT_T, align=MIN)
    body = cheeks + back + foot
    for xh in (x_back - BACK_T - 10, x_back - BACK_T - 30):
        for sy in (-1, 1):
            body = body - Pos(xh, sy * (width / 2 - 12), -CRADLE_H / 2) * Cylinder(1.7, FOOT_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body


def link(angle_deg: float = 0.0) -> Part:
    """Tấm link vai: hub R18 + tay 140 x 30, dày 8 (dọc y). Quay quanh trục y."""
    plate = Pos(0, -LINK_T / 2, 0) * Rot(X=-90) * Cylinder(HUB_R, LINK_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    arm = Pos(0, -LINK_T / 2, -LINK_NECK_H / 2) * Box(LINK_LEN, LINK_T, LINK_NECK_H, align=MIN)
    end = Pos(LINK_LEN, -LINK_T / 2, 0) * Rot(X=-90) * Cylinder(LINK_NECK_H / 2, LINK_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = plate + arm + end
    import math
    for k in range(4):
        a = math.radians(45 + 90 * k)
        h = Pos(HORN_PCD / 2 * math.cos(a), -LINK_T / 2, HORN_PCD / 2 * math.sin(a)) * Rot(X=-90) * Cylinder(HORN_HOLE / 2, LINK_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
        p = p - h
    # quay quanh trục y (dương = đầu link hạ xuống, quy ước spec.py)
    return Rot(Y=angle_deg) * p
