# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Trục nâng P-L khi gánh trạm hai tay (R3d): so phương án đưa servo lift về <= 70% định mức, không lò xo lộ ngoài.
 Nhẹ, chạy tại chỗ: python3 lift_tram_calc.py -> ../export/M1-LF-tram-calc.json (+ in bảng).
Tải lấy từ R3c mục 2 (8,28 / 8,77 / 9,28 kg); gia tốc 0,5 m/s²; lực răng = m(g+a), chưa cộng ma sát (như R3c).
Đường đặc tính servo tuyến tính: w = w0 (1 - T/T_chặn), w0 31 v/ph, chặn 95 kg·cm, định mức 22,5 kg·cm (docs/vi/servo-datasheet.md)."""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from faibotic import spec

G, A = spec.G, 0.5
KGCM = spec.KGCM
RATED = spec.STS3095.rated_kgcm * KGCM          # 2,207 N·m
STALL = spec.STS3095.stall_kgcm * KGCM          # 9,316 N·m
W0 = 31 * 2 * math.pi / 60                       # rad/s không tải [số hãng tự báo]
CAP = 0.70 * RATED                               # trần 70%
R0 = spec.LIFT_PINION_R                          # 21,4 mm hiện tại [giả định trong spec]
MOD = spec.LIFT_RACK_MODULE                      # 1,5 mm [giả định]
LOADS = {"thiet_ke_8.28": 8.28, "tay_can_tren_8.77": 8.77, "tai_1kg_tay_9.28": 9.28}
M_CARRIAGE_EMPTY = 0.91                          # kg phần xe nâng còn lại khi tháo trạm (R3c)


def rack(T_nm, r):
    """% định mức và tốc độ (mm/s) khi servo kéo mô-men T qua bánh bán kính r."""
    return round(100 * T_nm / RATED, 1), round(1000 * W0 * max(0.0, 1 - T_nm / STALL) * r, 1)


def row(name, fn):
    out = {}
    for k, m in LOADS.items():
        pct, v = fn(m)
        out[k] = {"pct_dinh_muc": pct, "v_mm_s": v}
    return {"ten": name, **out}


def F(m, mc=0.0):            # lực răng (N): treo m, đối trọng mc qua puly (đối trọng cũng phải tăng tốc)
    return (m - mc) * G + (m + mc) * A


res = {}
res["0_hien_tai_r21.4"] = row("Hiện tại: bánh r 21,4 mm (spec)", lambda m: rack(F(m) * R0, R0))
for z in (24, 22, 20, 18):
    r = z * MOD / 2 / 1000
    res[f"A_banh_{z}rang_r{r*1000:.1f}"] = row(f"A: bánh {z} răng m{MOD} r {r*1000:.1f} mm", lambda m, r=r: rack(F(m) * r, r))
ETA_STAGE = 0.95             # [giả định] một cấp bánh răng trụ in
res["B_them_cap_2to1"] = row("B: thêm cấp giảm 2:1, giữ bánh 28 răng", lambda m: (lambda T: (round(100 * T / RATED, 1), round(1000 * W0 * max(0, 1 - T / STALL) * R0 / 2, 1)))(F(m) * R0 / 2 / ETA_STAGE))
# C: vitme T8x2 tự hãm + servo tăng tốc 16:1 như v0.4 (lift_calc.py phương án C), hiệu suất ren mu 0,15
lam = math.atan(2.0 / (math.pi * 7.0)); phi = math.atan(0.15 / math.cos(math.radians(14.5))); eta_s = math.tan(lam) / math.tan(lam + phi)
def screw(m):
    T = F(m) * 0.002 / (2 * math.pi * eta_s) * 16 / (0.90 * 0.97)
    return round(100 * T / RATED, 1), round(1000 * W0 * max(0, 1 - T / STALL) * 16 * 0.002 / (2 * math.pi), 1)
res["C_vitme_T8x2_16to1"] = row("C: vitme T8x2 tự hãm + tăng tốc 16:1 (cột v0.4)", screw)
# F: đối trọng ẩn trong vỏ cột: khối lượng tối thiểu để <= 70% ở từng tải
cw_need = {k: round((F(m) - CAP / R0) / (G - A), 2) for k, m in LOADS.items()}
res["F_doi_trong_2.5kg"] = row("F: đối trọng 2,5 kg qua puly đỉnh cột", lambda m: rack(F(m, 2.5) * R0, R0))
# G: lò xo lực không đổi giấu trong cột: lực cần
spring_need = {k: round(F(m) - CAP / R0, 1) for k, m in LOADS.items()}
# H: nâng trần lên 85%: chỉ đổi trần, % như hiện tại
# E: giảm khối lượng trạm: khối lượng treo tối đa để <= 70% với bánh hiện tại
m_max_r0 = round(CAP / R0 / (G + A), 2)

# chi tiết phương án chọn A20
r20 = 20 * MOD / 2 / 1000
Y = {28: 0.353, 26: 0.346, 20: 0.322}            # hệ số Lewis góc áp 20°, răng đủ [bảng điển hình]
B_FACE = 16.0                                     # mm bề rộng răng [giả định như lift_calc.py]
lewis = {z: round(F(8.28) / (B_FACE * MOD * Y[z]), 1) for z in Y}   # MPa ở tải thiết kế, chưa nhân Kt
sel = {
    "r_mm": r20 * 1000, "duong_kinh_chia_mm": 20 * MOD, "duong_kinh_dinh_mm": 22 * MOD,
    "doi_tam_truc_servo_mm_so_r21.4": round((R0 - r20) * 1000, 1), "doi_tam_so_STEP_26rang_mm": round((26 * MOD / 2 - 20 * MOD / 2), 1),
    "luc_rang_toi_da_o_tran70_N": round(CAP / r20, 1), "luc_rang_o_chan_N": round(STALL / r20, 0),
    "luc_rang_o_chan_hien_tai_N": round(STALL / R0, 0),
    "steps_per_m": round(4096 / (math.pi * 2 * r20)), "steps_hanh_trinh_0.45m": round(0.45 * 4096 / (math.pi * 2 * r20)),
    "mm_moi_buoc": round(math.pi * 2 * r20 * 1000 / 4096, 4),
    "v_khong_tai_mm_s": round(1000 * W0 * r20, 1), "rpm_o_LIFT_SPEED_MAX": round(spec.LIFT_SPEED_MAX / (2 * math.pi * r20) * 60, 1),
    "lewis_MPa_tai_thiet_ke_chua_Kt": lewis,
    "F_do_rang_can_N_quy_tac_50pct": {k: round(2 * F(m), 0) for k, m in LOADS.items()},
}
out = {"ngay": "24/11/2026", "tran_70pct_Nm": round(CAP, 4), "luc_rang_tran_70_r21.4_N": round(CAP / R0, 1),
       "phuong_an": res, "F_doi_trong_toi_thieu_kg": cw_need, "G_luc_lo_xo_can_N": spring_need,
       "G_xe_trong_bi_keo_len_N_neu_lo_xo_cho_9.28": round(spring_need["tai_1kg_tay_9.28"] - M_CARRIAGE_EMPTY * G, 1),
       "E_khoi_luong_treo_toi_da_r21.4_kg": m_max_r0, "E_phai_giam_kg": round(8.28 - m_max_r0, 2),
       "chon_A20": sel}
os.makedirs(os.path.join(HERE, "../export"), exist_ok=True)
json.dump(out, open(os.path.join(HERE, "../export/M1-LF-tram-calc.json"), "w"), indent=1, ensure_ascii=False)
print(f"{'phương án':48s} " + "  ".join(f"{k:>18s}" for k in LOADS))
for v in res.values():
    print(f"{v['ten']:48s} " + "  ".join(f"{v[k]['pct_dinh_muc']:6.1f}% {v[k]['v_mm_s']:6.1f}mm/s" for k in LOADS))
print(json.dumps({k: v for k, v in out.items() if k != "phuong_an"}, indent=1, ensure_ascii=False))
