# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Đế ngàm (nửa cố định của giao diện lắp nhanh) và đế bàn (BỘ TUỲ CHỌN bán riêng). build123d.

Khung trạm S (mm) như tram_hai_tay.py. Cùng một đế ngàm M1-TR-020 (in PETG, chữ U: hai khối bên ôm hai mặt bên khung T giữa + thanh ngang
trước khung T) mang 2 chốt côn thép Ø8, 2 móc giữ khoá kéo, nửa cái giắc 21W4 (gắn nổi ±0,8 mm). Trên xe nâng: bắt vào khung T giữa M1-LF-057
(mặt bên |y| 70,8, mặt trước x 9); trên đế bàn: bắt lên khung 2040 của đế bàn. Trạm hạ thẳng đứng: chốt vào bạc trước 8 mm -> giắc tự dẫn ->
tựa mặt z -67 -> kéo 2 khoá.
Đế bàn (mã M1-TB-*, KHÔNG nằm trong danh sách in / BOM robot chuẩn): khung 2040 0,60 x 0,28 m, 2 tai kẹp chữ F ở mép sau, hộp rơ-le 12 V +
Reset + 2 cầu chì lá 20 A + cổng USB ra laptop, 4 chân cao su. Đặt trên mặt bàn z = -H_J1_TREN_BAN."""
import numpy as np
from tram_hai_tay import box, cyl_z, mirror_y, PIN_X, PIN_Y, DOCK_TOP, H_J1_TREN_BAN

DOCK_Z = (DOCK_TOP - 30.0, DOCK_TOP)          # khối ngàm dày 30
DOCK_X = (-30.0, 62.0)                        # trước Post3 (x <= -33,4)
DOCK_Y = (71.0, 120.0)                        # sát mặt bên khung T giữa (|y| 70,8), dưới tai trạm
STRAP_X = (10.0, 20.0)                        # thanh ngang trước khung T giữa (mặt trước x 9)
CONN_X, CONN_Y = (7.0, 60.0), (103.0, 117.0)  # nửa cái 21W4, chìm trong khối trái, mặt trên ngang mặt tựa


def dock_parts(prefix="M1-TR-02", tag="xe"):
    """Đế ngàm (khung trạm). prefix: mã chi tiết (xe: M1-TR-02x; bàn: M1-TB-01x dùng cùng hình)."""
    d = box(DOCK_X, (-DOCK_Y[1], DOCK_Y[1]), DOCK_Z) - box((STRAP_X[1], DOCK_X[1] + 1), (-DOCK_Y[0], DOCK_Y[0]), (DOCK_Z[0] - 1, DOCK_Z[1] + 1))
    d -= box((DOCK_X[0] - 1, STRAP_X[0]), (-DOCK_Y[0], DOCK_Y[0]), (DOCK_Z[0] - 1, DOCK_Z[1] + 1))
    d -= box(CONN_X, CONN_Y, (DOCK_Z[1] - 10.0, DOCK_Z[1] + 1))
    out = []
    for s, sd in ((1, "L"), (-1, "R")):
        d -= cyl_z(PIN_X, s * PIN_Y, (DOCK_Z[1] - 12.5, DOCK_Z[1] + 1), 4.0)
        out.append((f"{prefix}1_chot_con_{sd}_{tag}", "steel", cyl_z(PIN_X, s * PIN_Y, (DOCK_Z[1] - 12.0, DOCK_Z[1] + 16.0), 3.95)))
        out.append((f"{prefix}2_moc_giu_{sd}_{tag}", "steel", box((-20.0, 0.0), mirror_y((DOCK_Y[1], 134.0), s), (DOCK_Z[1] - 12.0, DOCK_Z[1]))))
    out.insert(0, (f"{prefix}0_de_ngam_{tag}", "navy", d))
    out.append((f"{prefix}3_giac_21W4_cai_{tag}", "steel", box(CONN_X, CONN_Y, (DOCK_Z[1] - 10.0, DOCK_Z[1]))))
    return out


TB_Z = (-H_J1_TREN_BAN + 3.0, DOCK_Z[0])     # khung 2040 nằm (20 cao) trên 4 chân cao su 3 mm


def de_ban_parts():
    """Đế bàn tuỳ chọn (khung trạm, chế độ bàn): đế ngàm M1-TB-01x + khung + tai kẹp + hộp rơ-le + chân."""
    out = dock_parts("M1-TB-01", "ban")
    z = TB_Z; zt = -H_J1_TREN_BAN
    for s, sd in ((1, "L"), (-1, "R")):
        out.append((f"M1-TB-002_thanh_doc_2040_{sd}", "alu", box((-220.0, 80.0), mirror_y((75.0, 115.0), s), z)))
        out.append((f"M1-TB-003_tai_kep_F_{sd}", "navy", box((-320.0, -260.0), mirror_y((200.0, 260.0), s), (z[0], z[0] + 10.0))))
        for x in (100.0, -240.0):
            out.append((f"M1-TB-005_chan_{sd}_{int(x)}", "rubber", cyl_z(x, s * 280.0, (zt, z[0]), 10.0)))
    out.append(("M1-TB-002_thanh_truoc_2040", "alu", box((80.0, 120.0), (-300.0, 300.0), z)))
    out.append(("M1-TB-002_thanh_sau_2040", "alu", box((-260.0, -220.0), (-300.0, 300.0), z)))
    out.append(("M1-TB-004_hop_role_reset", "navy", box((-200.0, -120.0), (-60.0, 60.0), (zt, zt + 40.0))))
    return out


def part_kg(n, s):
    """kg của chi tiết đế ngàm / đế bàn [giả định như tram_hai_tay.part_kg]."""
    import tram_hai_tay as T
    if "chot_con" in n or "moc_giu" in n: return s.volume * T.RHO_STEEL
    if "giac_21W4" in n: return 0.04
    if n.startswith("M1-TB-002"):
        b = s.bounding_box(); return 0.6e-3 * max(b.max.X - b.min.X, b.max.Y - b.min.Y)
    if n.startswith("M1-TB-004"): return 0.35       # rơ-le 12 V 40 A + đế, Reset, 2 cầu chì lá + đế, cọc vào, cổng USB, hộp [giả định]
    if n.startswith("M1-TB-005"): return 0.005
    return s.volume * T.RHO_PETG * T.INFILL


if __name__ == "__main__":
    import tram_hai_tay as T
    for n, m, s in dock_parts() + de_ban_parts():
        b = s.bounding_box(); print(f"{n:36s} {part_kg(n, s):6.3f}  x {b.min.X:.0f}..{b.max.X:.0f} y {b.min.Y:.0f}..{b.max.Y:.0f} z {b.min.Z:.0f}..{b.max.Z:.0f}")
