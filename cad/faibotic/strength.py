# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Tính bền sơ bộ gá vai (tính tay, công thức trong bao-cao-cad.md)."""
import json, math
from params import *
KGCM = 9.80665 / 10  # N·m trên kg·cm
KGCM_NMM = 98.0665   # N·mm trên kg·cm

T_DESIGN = 37.5 * KGCM_NMM        # N·mm, mốc F016 tính tay
T_STALL = 2 * 95.0 * KGCM_NMM     # 2 servo cùng chặn, dùng để biết giới hạn (không phải mốc đạt)
# Giới hạn bền nhựa: Prusament PETG chảy kéo mẫu in 39 MPa (ngang/XY), 42 MPa (đứng); PLA 51 / 59 MPa
# (datasheet Prusa Polymers, ISO 527-1; nguồn trong bao-cao-cad.md). Lấy giá trị NGANG nhỏ hơn.
MATS = {"PETG": 39.0, "PLA": 51.0}
KT = 3.0  # hệ số tập trung ứng suất giả định cho lỗ/góc

res = {}
def sf(sigma, lim):
    return lim / sigma

# (1) Cổ tấm link: uốn, tiết diện NECK_H x LINK_T, M = T (bảo thủ: toàn bộ mô-men qua cổ)
Z_neck = LINK_T * LINK_NECK_H ** 2 / 6
# (2) Bu-lông horn: 4 x M3 trên PCD 20, mỗi servo mang T/2. Tải ổ đỡ lên nhựa (d=3,t=LINK_T/2 mỗi bên? lấy t = HORN... tấm dày 8)
r = HORN_PCD / 2
F_bolt = (T_DESIGN / 2) / (4 * r)
bearing = F_bolt / (3.0 * LINK_T)
# xé mép: khoảng cách từ tâm lỗ tới mép (lỗ ở r=10, hub R=18 → 8 mm - 1.7) ; cắt 2 mặt: tau = F/(2*e*t)
edge = HUB_R - r
tearout = F_bolt / (2 * (edge - HORN_HOLE / 2) * LINK_T)
# (3) Tường sau cradle: chịu mômen phản lực T (qua 2 má) uốn quanh trục y, tiết diện rộng 'width' dày BACK_T... 
width = 2 * (LINK_T / 2 + HORN_T + SERVO_W + CLEAR + CHEEK_T)
Z_back = width * BACK_T ** 2 / 6
# (4) Má: lực phản lực cặp tại 2 tai vít cách nhau 55 mm, mỗi má T/2 → F = T/2/55, uốn má như dầm công xôn dài 'ARM'=35 mm
ear_F = (T_DESIGN / 2) / 55.0
Z_cheek = CRADLE_H * CHEEK_T ** 2 / 6   # tiết diện ngang má theo hướng ép
sig = {
  "neck_bending": T_DESIGN / Z_neck,
  "back_wall_bending": T_DESIGN / Z_back,
  "cheek_bending": ear_F * 35.0 / Z_cheek,
}
out = {"T_design_Nmm": T_DESIGN, "T_stall_both_Nmm": T_STALL, "Kt": KT, "F_bolt_N": F_bolt,
       "bearing_MPa": bearing, "tearout_shear_MPa": tearout, "sigma_MPa": sig, "Z": dict(neck=Z_neck, back=Z_back, cheek=Z_cheek)}
rows = []
for m, lim in MATS.items():
    for name, s in list(sig.items()) + [("bolt_bearing", bearing)]:
        rows.append(dict(mat=m, check=name, load="37.5 kg·cm", sigma=round(s, 2), sigma_Kt=round(s * KT, 2), SF_Kt=round(lim / (s * KT), 2)))
        s2 = s * T_STALL / T_DESIGN
        rows.append(dict(mat=m, check=name, load="2x95 kg·cm chặn", sigma=round(s2, 2), sigma_Kt=round(s2 * KT, 2), SF_Kt=round(lim / (s2 * KT), 2)))
    # tearout dùng 0.6*lim cắt
    rows.append(dict(mat=m, check="tearout(0.6*lim)", load="37.5 kg·cm", sigma=round(tearout, 2), sigma_Kt=round(tearout*KT,2), SF_Kt=round(0.6*lim/(tearout*KT), 2)))
    rows.append(dict(mat=m, check="tearout(0.6*lim)", load="2x95 kg·cm chặn", sigma=round(tearout*T_STALL/T_DESIGN,2), sigma_Kt=round(tearout*T_STALL/T_DESIGN*KT,2), SF_Kt=round(0.6*lim/(tearout*T_STALL/T_DESIGN*KT), 2)))
# mômen cho phép (kg·cm) để SF>=2 kể cả Kt=3 với PETG ngang 39 MPa; tuyến tính theo mômen
tl = {"neck": 39.0/(2*KT)/sig["neck_bending"], "back": 39.0/(2*KT)/sig["back_wall_bending"], "cheek": 39.0/(2*KT)/sig["cheek_bending"], "bolt": 39.0/(2*KT)/bearing}
out["torque_allow_kgcm_PETG_SF2_Kt3"] = {k: round(37.5*v, 1) for k, v in tl.items()}
out["rows"] = rows
print(json.dumps(out, indent=1, ensure_ascii=False))
json.dump(out, open("../export/strength.json", "w"), indent=1, ensure_ascii=False)
