# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Bền chi tiết chịu lực lò xo (tính tay, PETG ngang 39 MPa [datasheet Prusa Polymers, nguồn trong bao-cao-cad.md], Kt = 3 giả định,
bu-lông M5 cấp 8.8 Rp0,2 >= 640 MPa [ISO 898-1, giá trị điển hình, chưa kiểm nguồn lần này]). Chạy: python balance_strength.py"""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(__file__))
import balance as b
from params import *
KT, PETG, BOLT = 3.0, 39.0, 640.0
G_STEEL = 79000.0
Fmax = max(b.SPR_F0 + b.SPR_K * (b.spring_len(q) - b.SPR_L0) for q in range(-100, 91))
Fq = {q: b.SPR_F0 + b.SPR_K * (b.spring_len(q) - b.SPR_L0) for q in (-100, -50, 0, 45, 90)}
# lực ngang lớn nhất lên trụ neo (thành phần x) trong dải
def fx(q):
    B = (b.A_B * math.cos(math.radians(q)), -b.A_B * math.sin(math.radians(q))); dx, dz = B[0] - b.X_A, B[1] - b.H_A; L = math.hypot(dx, dz)
    return (b.SPR_F0 + b.SPR_K * (L - b.SPR_L0)) * abs(dx) / L
Fx = max(fx(q) for q in range(-100, 91))
# 1) chốt M5: uốn, công-xôn từ mặt link tới mặt phẳng lò xo
lever = b.Y_S - LINK_T / 2
sig_pin = Fmax * lever / (math.pi * b.PIN_D ** 3 / 32)
# 2) lỗ chốt trên link PETG: ép mặt, 2 lò xo cùng chiều
sig_bear = 2 * Fmax / (b.PIN_D * LINK_T)
# 3) trụ neo liền gá (PETG): Fx uốn quanh y tại mặt má (cánh tay h - cạnh trên má); Fz lệch tâm (mặt phẳng lò xo - giữa trụ) uốn quanh x
t_post = (b.Y_OUT + b.POST_Y1) - b.Y_IN
arm = b.H_A - CRADLE_H / 2
sig_post_x = Fx * arm / (t_post * b.POST_W ** 2 / 6)
ecc = b.Y_S - (b.Y_IN + t_post / 2)
sig_post_y = Fmax * ecc / (b.POST_W * t_post ** 2 / 6) + Fmax / (b.POST_W * t_post)
sig_post = sig_post_x + sig_post_y
# 4) má tại chân trụ chịu uốn do Fmax lệch tâm so với mặt má (má dày CHEEK_T trong tiết diện trụ 14 mm: dùng tiết diện trụ ở trên)
# 5) ép mặt bu-lông M5 neo trong PETG, đường kính 5, dày trụ 14
sig_bear2 = Fmax / (b.PIN_D * t_post)
# 6) cổ link tại chốt: mô-men trọng lực cả tay (54,6 kg·cm = 5.355 N·m) trừ lò xo; lấy không trừ (bảo thủ), tiết diện trừ lỗ
M = 54.62 * 98.0665; Znet = LINK_T * (LINK_NECK_H ** 3 - b.PIN_HOLE ** 3) / (6 * LINK_NECK_H)
sig_neck = M / Znet
rows = [("chốt M6 uốn", sig_pin, BOLT, 1.0), ("link ép mặt lỗ chốt", sig_bear, PETG, KT), ("trụ neo uốn (Fx + lệch tâm Fz)", sig_post, PETG, KT),
        ("trụ neo ép mặt bu-lông", sig_bear2, PETG, KT), ("cổ link tại lỗ chốt (mô-men tay, không trừ lò xo)", sig_neck, PETG, KT)]
# 7) lò xo (thép đàn hồi ASTM A228 / EN 10270-1 SM): d, D trung bình, số vòng từ k; Wahl; ứng suất xoắn so với Rm (Shigley: Rm = 2211 / d^0.145 MPa)
d_w, D_w = 1.4, 15.4; C = D_w / d_w; Kw = (4 * C - 1) / (4 * C - 4) + 0.615 / C
Na = G_STEEL * d_w ** 4 / (8 * D_w ** 3 * b.SPR_K); Rm = 2211 / d_w ** 0.145
tau = Kw * 8 * Fmax * D_w / (math.pi * d_w ** 3); tau0 = 8 * b.SPR_F0 * D_w / (math.pi * d_w ** 3)
spring = {"d_mm": d_w, "D_mm": D_w, "C": round(C, 2), "Wahl": round(Kw, 3), "Na": round(Na, 1), "L0_tinh_mm": round((Na + 1) * d_w + 2 * (D_w - d_w), 1),
          "Rm_MPa": round(Rm), "tau_max_MPa": round(tau), "tau_max_over_Rm": round(tau / Rm, 3), "tau0_MPa": round(tau0), "F0_N": b.SPR_F0, "k_N_mm": b.SPR_K,
          "gian_max_mm": round(max(b.spring_len(q) for q in range(-100, 91)) - b.SPR_L0, 1), "L_min_mm": round(min(b.spring_len(q) for q in range(-100, 91)), 1)}
radial = 2 * Fmax   # tải hướng kính thêm lên hai ổ servo vai (chia hai ~Fmax mỗi servo + mô-men lệch tâm)
out = {"lo_xo": spring, "tai_huong_kinh_them_len_truc_vai_N": round(radial, 1), "F_max_N": round(Fmax, 1), "F_theo_q": {str(k): round(v, 1) for k, v in Fq.items()}, "Fx_max_N": round(Fx, 1),
       "ghi_chu": "SF = giới hạn / (sigma x Kt); Kt = 1 cho bu-lông thép", "ket_qua": []}
for n, s, lim, kt in rows:
    out["ket_qua"].append({"check": n, "sigma_MPa": round(s, 2), "gioi_han_MPa": lim, "Kt": kt, "SF": round(lim / (s * kt), 2)})
    print(f"{n}: sigma {s:.2f} MPa, SF {lim/(s*kt):.2f}")
print("Fmax", Fmax)
json.dump(out, open(os.path.join(b.EXP, "balance_strength.json"), "w"), ensure_ascii=False, indent=1)
