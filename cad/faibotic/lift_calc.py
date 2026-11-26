# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Trục nâng M1: so phương án truyền động, tải, mô-men servo, tự hãm, bền.
Chạy: python lift_calc.py -> ../export/M1-LF-calc.json (+ in tóm tắt).
Mọi số có cờ trong archive/v0.4/cad/bao-cao-cot-nang.md. g = 9,81."""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from faibotic import spec
G = 9.81
KGCM = 0.0980665          # N*m / (kg*cm)
RATED, STALL, W0_RPM = spec.STS3095.rated_kgcm, spec.STS3095.stall_kgcm, 31.0   # 22,5 / 95 kg*cm, 31 vong/phut [nha ban tu bao, docs/vi/servo-datasheet.md]
W0 = W0_RPM * 2 * math.pi / 60
STROKE = spec.LIFT_RANGE[1] - spec.LIFT_RANGE[0]

# ---- tải ----
arm = sum(spec.LINK_MASS[k] for k in ("upper", "fore", "wrist", "hand"))     # 1,042 kg/tay (khong gom shoulder)
sh = spec.LINK_MASS["shoulder"] + 0.05                                        # than xoay pan + 2 tru neo [bao-cao-cad v0.3]
M_CARRIAGE = 1.75          # kg, xe nang theo CAD lift.py (1,73 kg: tam Al, dam, 4 khoi MGN12H, dai oc) [tu suy ra, thay spec 0,8]
PAYLOAD = 2 * 0.5
M_TOT = 2 * (arm + sh) + M_CARRIAGE + PAYLOAD
F_FRIC = 10.0              # N ma sat ray + nut lech tam [gia dinh]
W = M_TOT * G

def screw(lead, d2, mu, alpha_deg=14.5):
    lam = math.atan(lead / (math.pi * d2)); phi = math.atan(mu / math.cos(math.radians(alpha_deg)))
    eta_up = math.tan(lam) / math.tan(lam + phi)
    return lam, phi, eta_up

def drive(name, kind, F, i=1.0, r=None, lead=None, d2=7.0, mu=0.15, eta_t=0.90, eta_b=0.97, cw=0.0):
    """Tra ve mo-men servo (N*m), % dinh muc, toc do (m/s) khi nang tai F (N)."""
    if kind == "screw":
        lam, phi, eta_s = screw(lead, d2, mu)
        t_screw = F * lead / 1000 / (2 * math.pi * eta_s)
        ts = t_screw * i / (eta_t * eta_b)
        v = lambda t: W0 * (1 - t / (STALL * KGCM)) * i * lead / 1000 / (2 * math.pi)
        sl = lam < phi
        return dict(ten=name, lam_deg=round(math.degrees(lam), 2), phi_deg=round(math.degrees(phi), 2), eta_ren=round(eta_s, 3),
                    T_servo_Nm=round(ts, 3), T_pct=round(100 * ts / KGCM / RATED, 1), v_mm_s=round(1000 * v(ts), 1), tu_ham=sl)
    Fe = max(F - cw * G, 0.0) if cw else F
    ts = Fe * r / eta_b
    v = W0 * (1 - ts / (STALL * KGCM)) * r
    return dict(ten=name, T_servo_Nm=round(ts, 3), T_pct=round(100 * ts / KGCM / RATED, 1), v_mm_s=round(1000 * v, 1), tu_ham=False)

F_up = W + F_FRIC
opts = {
 "A_thanh_rang": drive("A: banh rang-thanh rang truc tiep (nhu AlohaMini2), pinion r=24 mm", "rack", F_up, r=0.024),
 "B_dai_doi_trong": drive("B: dai GT2 (puly 30T r=9,55 mm) + doi trong 4,3 kg", "rack", F_up, r=0.00955, cw=2 * (arm + sh) + M_CARRIAGE),
 "B2_dai_khong_doi_trong": drive("B2: dai GT2 30T, khong doi trong", "rack", F_up, r=0.00955),
 "C_vitme_T8x2_16to1": drive("C: vitme T8x2 (lead 2, 1 dau) + tang toc 16:1 (2 cap banh rang)", "screw", F_up, i=16, lead=2.0),
 "D_vitme_T8x8_4to1": drive("D: vitme T8x8 (4 dau) + tang toc 4:1", "screw", F_up, i=4, lead=8.0),
}
# nhay cam mu cua C
sens = {f"mu={m}": drive("C", "screw", F_up, i=16, lead=2.0, mu=m) for m in (0.08, 0.10, 0.12, 0.15, 0.20)}
lam_c = math.atan(2.0 / (math.pi * 7.0)); mu_min = math.tan(lam_c) * math.cos(math.radians(14.5))

# ---- C: nang / ha / suc nang toi da ----
c = opts["C_vitme_T8x2_16to1"]
def F_cap(frac, mu=0.15):
    lam, phi, eta_s = screw(2.0, 7.0, mu)
    ts = frac * RATED * KGCM
    return ts * eta_b_g / 16 * 2 * math.pi * eta_s / 0.002
eta_b_g = 0.90 * 0.97
cap70 = F_cap(0.70); cap70_lo = F_cap(0.70, 0.20)   # mu cao hon -> hieu suat thap hon
lam, phi, eta_s = screw(2.0, 7.0, 0.15)
# ha tai (can mo-men duong de ha vi tu ham): T = F*dm/2*tan(phi-lam)
t_low = F_up * 0.0035 * math.tan(phi - lam) * 16 / (0.90 * 0.97) if False else W * 0.0035 * math.tan(phi - lam) * 16 / 1.0
# cap rang hieu suat nguoc ~ (1/eta) -> mo-men servo khi ha ~ T_screw_low*i*eta_g
t_low = W * 0.0035 * math.tan(phi - lam) * 16 * (0.90 * 0.97)
# tai SOR-003: khong tu dat chua doi, M_TOT >= 4 kg
sor3 = dict(M_tong_kg=round(M_TOT, 2), F_nang_N=round(F_up, 1), suc_nang_70pct_N=round(cap70, 1), suc_nang_70pct_kg=round(cap70 / G, 1),
            he_so_tai_70pct=round(cap70 / F_up, 2), suc_nang_70pct_mu020_kg=round(cap70_lo / G, 1))

# ---- tự hãm khi mất điện ----
selflock = dict(lam_deg=round(math.degrees(lam_c), 2), mu_toi_thieu=round(mu_min, 3),
                phi_mu015_deg=round(math.degrees(math.atan(0.15 / math.cos(math.radians(14.5)))), 2),
                phi_mu012_deg=round(math.degrees(math.atan(0.12 / math.cos(math.radians(14.5)))), 2),
                phi_mu010_deg=round(math.degrees(math.atan(0.10 / math.cos(math.radians(14.5)))), 2))

# ---- mô-men uốn cột (tay duỗi ngang + tải 0,5 kg mỗi tay), tay x-ban kinh tu truc cot ----
XC, XS = spec.COLUMN_X, spec.SHOULDER_X
x_sh = XS - XC                                                   # vai cach truc cot 0,22 m
com_arm = {k: spec.LINK_COM[k] for k in spec.LINK_COM}
# tay duoi ngang: upper com 0.0489, fore tu khuyu (L1), wrist tu L1+L2, hand tu L1+L2+LW
xs = {"upper": spec.LINK_COM["upper"], "fore": spec.L1 + spec.LINK_COM["fore"], "wrist": spec.L1 + spec.L2 + spec.LINK_COM["wrist"],
      "hand": spec.L1 + spec.L2 + spec.LW + spec.LINK_COM["hand"]}
x_grip = spec.L1 + spec.L2 + spec.LW + spec.LT
m_arm_moment = sum(spec.LINK_MASS[k] * (x_sh + xs[k]) for k in xs)      # kg*m
M_pitch = 2 * G * (m_arm_moment + sh * (x_sh) + 0.5 * (x_sh + x_grip)) + G * M_CARRIAGE * 0.05   # N*m (xe: tam lech 0,05 m [gia dinh])
A_DEC = 3.0               # m/s^2 phanh khan cap/va cham nhe [gia dinh]
H_COM = 0.45              # m, tam khoi cua xe+tay cao hon san cot tai o thap [gia dinh, tinh o vai 1,05: lay tam khoi so voi chan cot]
Hcar = spec.SHOULDER_Z0 + spec.LIFT_RANGE[1] - spec.BASE_Z1      # chieu cao vai tren chan cot (m), nang toi da
M_dyn = M_TOT * A_DEC * Hcar
E_AL, FY_AL = 70e3, 110.0          # MPa (6063-T5 min ~110) [dien hinh, chua kiem]
I_COL = 7.0e4            # mm^4 (4040 nhe ~ 7 cm4) [gia dinh bao thu, can datasheet ham]
Z_COL = I_COL / 20
sig_s = M_pitch * 1e3 / Z_COL; sig_d = (M_pitch + M_dyn) * 1e3 / Z_COL
theta = M_pitch * 1e3 * Hcar * 1e3 / (E_AL * I_COL)
d_car = M_pitch * 1e3 * (Hcar * 1e3) ** 2 / (2 * E_AL * I_COL)
d_hand = theta * (x_sh + x_grip) * 1e3 + 0
bend = dict(M_pitch_Nm=round(M_pitch, 1), M_dyn_Nm=round(M_dyn, 1), H_vai_m=round(Hcar, 3), sigma_tinh_MPa=round(sig_s, 1), sigma_dong_MPa=round(sig_d, 1),
            SF_tinh=round(FY_AL / sig_s, 1), SF_dong=round(FY_AL / sig_d, 1), goc_nghieng_mrad=round(theta * 1e3, 2), vong_xe_mm=round(d_car, 2),
            sut_dau_tay_mm=round(d_hand, 2))
# khoi ray: 2 ray x 2 khoi cach nhau S doc theo z
S_BLK = 0.100
F_blk = (M_pitch + M_dyn) / S_BLK / 4 + W / 4
blk = dict(S_khoi_mm=100, luc_moi_khoi_N=round(F_blk, 0))
# vit me: euler (pinned-pinned, Droot 6 mm, E 200 GPa, L 0,60 m), nen
Pcr = math.pi ** 2 * 200e3 * (math.pi * 6.0 ** 4 / 64) / 600.0 ** 2
# banh rang in (PETG): Lewis, mod 1,5, b 16, Y~0,30 (z 16..48), Kt 3; tai = mo-men servo gioi han 70% dinh muc, ban kinh banh lon 36 mm
T70 = 0.70 * RATED * KGCM
Ft = T70 / 0.036
sig_g = Ft / (16 * 1.5 * 0.30)
FY_PETG = 40.0   # MPa tu bao-cao-cad.md (PETG, chiu uon z) [dien hinh]
gear = dict(Ft_N=round(Ft, 1), sigma_MPa=round(sig_g, 1), SF_Kt3=round(FY_PETG / (3 * sig_g), 2),
            Ft_ket_tai_stall_N=round(STALL * KGCM / 0.036, 0), sigma_stall_MPa=round(STALL * KGCM / 0.036 / (16 * 1.5 * 0.30), 1),
            SF_stall_Kt3=round(FY_PETG / (3 * STALL * KGCM / 0.036 / (16 * 1.5 * 0.30)), 2))
# lực kẹt khi nut chạm chặn cứng nếu servo ở mô-men stall / 45% / 70% (vit me tu ham nen luc kep lon)
def jam(frac):
    return frac * RATED * KGCM * 0.90 * 0.97 / 16 * 2 * math.pi * screw(2.0, 7.0, 0.15)[2] / 0.002
jamF = {"70%_dm": round(jam(0.70)), "stall": round(STALL * KGCM * 0.90 * 0.97 / 16 * 2 * math.pi * screw(2.0, 7.0, 0.15)[2] / 0.002)}
out = dict(ngay="22/11/2026", tai=dict(m_tay_kg=round(arm, 3), m_vai_kg=round(sh, 3), m_xe_kg=M_CARRIAGE, tai_them_kg=PAYLOAD, M_tong_kg=round(M_TOT, 2), W_N=round(W, 1), F_ma_sat_N=F_FRIC),
           phuong_an=opts, C_nhay_cam_mu=sens, C_ha_tai_servo_Nm=round(t_low, 3), SOR003=sor3, tu_ham=selflock, uon_cot=bend, khoi_ray=blk,
           vitme_Pcr_N=round(Pcr), banh_rang_in=gear, luc_ket_chan_cung_N=jamF)
json.dump(out, open(os.path.join(HERE, "../export/M1-LF-calc.json"), "w"), indent=1, ensure_ascii=False)
print(json.dumps(out, indent=1, ensure_ascii=False))
