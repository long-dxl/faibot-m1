# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Tham số chung (mm). Mọi số có cờ nguồn trong ../ASSUMPTIONS.md."""

# Servo Feetech STS3095-C002 (12 V): 65 x 30 x 48 mm [trang bên bán, chưa có bản vẽ hãng]
SERVO_L, SERVO_W, SERVO_H = 65.0, 30.0, 48.0   # W = chiều dọc trục ra (giả định)
SHAFT_FROM_END = 10.0     # trục cách đầu +x của thân [giả định, chưa xác minh]
HORN_R, HORN_T = 12.5, 3.5
CLEAR = 0.4               # khe thiết kế servo-vỏ (yêu cầu >= 0.3)

# Gá vai
LINK_T = 8.0              # bề dày tấm link (dọc trục y)
CHEEK_T = 6.0
BACK_T = 8.0
CRADLE_H = 60.0           # chiều cao má (z)
FOOT_LEN = 40.0
FOOT_T = 6.0
HUB_R = 18.0
LINK_LEN = 140.0          # vai -> khuỷu (spec.py L1)
LINK_NECK_H = 30.0
HORN_PCD, HORN_HOLE = 20.0, 3.4   # 4 x M3 trên đường tròn Ø20 [giả định]
BORE_OUT = 16.0

# Giả định khối ngoài (ASSUMPTIONS.md)
BATT = (160.0, 80.0, 70.0)    # pin 3S, có vỏ [giả định, 22/11]
PCB = (100.0, 80.0, 1.6)      # PCB nguồn <= 100x80 [giả định]
PCB_COMP_H = 18.0
ESTOP_HOLE = 22.0
ESTOP_HEAD = 40.0

# v0.4.5 (22/11/2026): 2 lỗ M3 xuyên link cho tab M1-AR-023 (x, z) hệ link. Lỗ dưới dời (105,5; -8) -> (91,5; -8): đai ốc/đầu vít ở lỗ cũ
# giao servo khuỷu từ ~118 độ; lỗ mới cách vùng quét servo khuỷu (-32..133 độ) >= 6,0 mm (tính 2D), vòng đệm Ø7 còn >= 2,5 mm.
TAB_HOLES = ((105.5, 8.0), (91.5, -8.0))
