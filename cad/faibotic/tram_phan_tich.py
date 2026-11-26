# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""R3 (G1b): nghiên cứu thiết kế trạm hai tay tháo lắp. Chỉ phân tích, không dựng hình, không sửa CAD/SDK.
 Chỉ numpy (+ matplotlib để vẽ).

Dùng động học SDK (sdk/faibotic/kinematics.py, giới hạn khớp và bảng ghép trong spec.py) cho 2 tay AM-ARM200 + kẹp gốc AM-ARM200 (trước: kẹp cũ).
Khung trạm: gốc ở giữa hai trục pan, ngang đáy J1; x ra trước, y sang trái, z lên. Tay trái gốc (0, +s/2, 0), tay phải (0, -s/2, 0).

Chạy: python cad/faibotic/tram_phan_tich.py  -> cad/export/M1-TRAM-r3.json + media/qc/r3/*.png
Mọi kích thước trạm, bán kính vỏ khâu, khối lượng phụ kiện là [giả định] ghi trong GIA_DINH; số tay lấy từ spec.py.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "sdk"))
sys.path.insert(0, HERE)
from faibotic import kinematics as K  # noqa: E402
from faibotic import spec  # noqa: E402

D = math.radians
GIA_DINH = dict(
    h_j1_tren_ban=0.05,        # đáy J1 cao hơn mặt bàn ở chế độ bàn: đế bàn 2040 + ray trạm [giả định]
    cam_tren_ban=0.65,         # yêu cầu của dự án: camera đầu cao 0,65 m trên mặt bàn
    fov_cheo_deg=80.0,         # H65V1: bên bán ghi "FOV 80° tuỳ chọn" (không rõ chéo/ngang) -> coi là chéo, ảnh 4:3 [giả định]
    anh_ty_le=(4, 3),
    r_vai=0.05,                # bán kính bao trụ pan -> trục vai (servo pan + vai) [giả định]
    r_canh=0.035,              # cánh tay (STS3095 khuỷu) [giả định]
    r_cang=0.030,              # cẳng tay [giả định]
    r_kep=0.045,               # cổ tay + kẹp (khe tới 97 mm) + camera cổ tay [giả định]
    z_vat=0.03,                # tâm kẹp trên mặt bàn khi gắp vật nhỏ
    pitches_deg=(-90.0, -60.0, -45.0, -30.0),
    cot_mast_r=0.015,          # cột camera nhôm 2020 (nửa đường chéo ~14 mm) [giả định]
    gimbal_r=0.045,            # gá pan-tilt + camera ở đỉnh cột [giả định]
)
SPACINGS = (0.37452, 0.45, 0.475, 0.50)   # 0,3745 = khoảng hiện tại (URDF AlohaMini2 ±187,26 mm), để so


# ---------------- khung xương tay (vector hoá, khớp với kinematics._arm_point) ----------------
def skeleton(q, side):
    """q: (N,5). Trả dict điểm (N,3) trong khung tay: B (đáy), S (vai), E (khuỷu), W (trục gập), T (TCP), F (đầu ngón)."""
    q = np.atleast_2d(q)
    q1, q2, q3, q4 = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    a1, a12, a123 = -q2, -(q2 + q3), -(q2 + q3 + q4)
    sgn = 1.0 if side == "left" else -1.0
    S = np.stack([np.full_like(q1, spec.SHOULDER_DR), np.full_like(q1, spec.D0)], 1)
    E = S + spec.L1 * np.stack([np.cos(a1), np.sin(a1)], 1)
    W = E + np.stack([spec.L2 * np.cos(a12) - spec.WF_DZ * np.sin(a12), spec.L2 * np.sin(a12) + spec.WF_DZ * np.cos(a12)], 1)
    dz = spec.ROLL_DZ + spec.TCP_DZ
    def along(L):
        return W + np.stack([L * np.cos(a123) - dz * np.sin(a123), L * np.sin(a123) + dz * np.cos(a123)], 1)
    T, F = along(spec.LT), along(spec.LT + spec.TIP_LEN)
    lat = sgn * spec.ROLL_DY + sgn * spec.TCP_DY
    c, s = np.cos(q1), np.sin(q1)
    def to3(P, lat_):
        return np.stack([P[:, 0] * c - lat_ * s, P[:, 0] * s + lat_ * c, P[:, 1]], 1)
    zero = np.zeros_like(q1)
    return dict(B=np.stack([zero, zero, zero], 1), S=to3(S, 0.0), E=to3(E, 0.0), W=to3(W, 0.0), T=to3(T, lat), F=to3(F, lat))


def capsules(sk, base):
    """4 đoạn bao của tay: (P0, P1, r) trong khung trạm."""
    g = GIA_DINH
    b = np.asarray(base)
    return [(sk["B"] + b, sk["S"] + b, g["r_vai"]), (sk["S"] + b, sk["E"] + b, g["r_canh"]),
            (sk["E"] + b, sk["W"] + b, g["r_cang"]), (sk["W"] + b, sk["F"] + b, g["r_kep"])]


def seg_seg_dist(p1, q1, p2, q2):
    """Khoảng cách nhỏ nhất giữa các cặp đoạn (vector hoá, Ericson 5.1.9)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a = np.einsum("ij,ij->i", d1, d1); e = np.einsum("ij,ij->i", d2, d2); f = np.einsum("ij,ij->i", d2, r)
    c = np.einsum("ij,ij->i", d1, r); b = np.einsum("ij,ij->i", d1, d2)
    eps = 1e-12
    denom = a * e - b * b
    s = np.where(denom > eps, np.clip((b * f - c * e) / np.where(denom > eps, denom, 1), 0, 1), 0.0)
    t = (b * s + f) / np.where(e > eps, e, 1)
    t = np.where(e > eps, t, 0.0)
    s = np.where(t < 0, np.clip(-c / np.where(a > eps, a, 1), 0, 1), np.where(t > 1, np.clip((b - c) / np.where(a > eps, a, 1), 0, 1), s))
    t = np.clip(t, 0, 1)
    c1 = p1 + d1 * s[:, None]; c2 = p2 + d2 * t[:, None]
    return np.linalg.norm(c1 - c2, axis=1)


def arm_collide(capA, capB):
    """True nếu bất kỳ cặp đoạn bao nào của hai tay chạm nhau (khoảng cách < tổng bán kính)."""
    hit = np.zeros(len(capA[0][0]), bool)
    for (a0, a1, ra) in capA:
        for (b0, b1, rb) in capB:
            hit |= seg_seg_dist(a0, a1, b0, b1) < ra + rb
    return hit


def base_of(side, s):
    return np.array([0.0, (s / 2) if side == "left" else -(s / 2), 0.0])


# ---------------- 1. vùng làm việc trên bàn ----------------
def reach_map(s, z_obj, step=0.01, xr=(-0.10, 0.75), yr=(-0.75, 0.75)):
    """Lưới (x, y) ở độ cao tâm kẹp z_obj (khung trạm). Trả xs, ys, mask trái, mask phải, q trái, q phải (NaN khi không tới)."""
    xs = np.arange(xr[0], xr[1] + 1e-9, step); ys = np.arange(yr[0], yr[1] + 1e-9, step)
    out = {}
    for side in ("left", "right"):
        b = base_of(side, s)
        m = np.zeros((len(ys), len(xs)), bool); Q = np.full((len(ys), len(xs), 5), np.nan)
        for i, y in enumerate(ys):
            for j, x in enumerate(xs):
                p = np.array([x, y, z_obj]) - b
                if math.hypot(p[0], p[1]) > 0.62:
                    continue
                for pd in GIA_DINH["pitches_deg"]:
                    q = K.arm_ik(p, pitch=D(pd), side=side)
                    if q is not None:
                        m[i, j] = True; Q[i, j] = q; break
        out[side] = (m, Q)
    return xs, ys, out


def band_width(mask_both, ys, xs):
    """Bề rộng lớn nhất theo y của vùng chung (m) và dải x có vùng chung."""
    if not mask_both.any():
        return 0.0, None
    w = mask_both.sum(0) * (ys[1] - ys[0])
    cols = np.where(mask_both.any(0))[0]
    return float(w.max()), (float(xs[cols[0]]), float(xs[cols[-1]]))


# ---------------- 2. va chạm tay-tay (Monte Carlo trên đích đạt được) ----------------
def collision_stats(s, Qs, rng, n=20000):
    """Qs: dict side -> mảng cấu hình đạt được (M,5). Trả xác suất chạm khi hai đích độc lập, và khi mỗi tay ở nửa mình."""
    res = {}
    QL, QR = Qs["left"], Qs["right"]
    iL = rng.integers(0, len(QL["q"]), n); iR = rng.integers(0, len(QR["q"]), n)
    cL = capsules(skeleton(QL["q"][iL], "left"), base_of("left", s))
    cR = capsules(skeleton(QR["q"][iR], "right"), base_of("right", s))
    hit = arm_collide(cL, cR)
    res["p_cham_ngau_nhien"] = float(hit.mean())
    own = (QL["y"][iL] >= 0) & (QR["y"][iR] <= 0)
    res["p_cham_moi_tay_nua_minh"] = float(hit[own].mean()) if own.any() else None
    own2 = (QL["y"][iL] >= 0.05) & (QR["y"][iR] <= -0.05)
    res["p_cham_cach_tam_5cm"] = float(hit[own2].mean()) if own2.any() else None
    return res, hit, iL, iR


def risk_map(s, xs, ys, mL, QL, Qs, rng, nR=300):
    """Với mỗi ô tay trái tới được: tỷ lệ đích tay phải (ngẫu nhiên nR) gây chạm. Trả lưới tỷ lệ (NaN nếu trái không tới)."""
    R = np.full(mL.shape, np.nan)
    iR = rng.integers(0, len(Qs["right"]["q"]), nR)
    cR = capsules(skeleton(Qs["right"]["q"][iR], "right"), base_of("right", s))
    for (i, j) in zip(*np.where(mL)):
        q = np.repeat(QL[i, j][None], nR, 0)
        cL = capsules(skeleton(q, "left"), base_of("left", s))
        R[i, j] = arm_collide(cL, cR).mean()
    return R


# ---------------- 3. vùng quét tay (cột camera, bệ, cột nâng) ----------------
def sweep_configs(step_pan=10, step_sh=6, step_el=6, step_wr=10):
    out = []
    for sh in np.arange(-80, 48.5 + 1e-9, step_sh):
        emax = math.degrees(spec.elbow_max(D(sh)))
        for el in np.arange(-10, emax + 1e-9, step_el):
            wmax = math.degrees(spec.wrist_max(D(sh), D(el)))
            for wr in np.arange(-77.5, wmax + 1e-9, step_wr):
                out.append((sh, el, wr))
    base = np.radians(np.array(out))
    pans = np.radians(np.arange(-70, 70 + 1e-9, step_pan))
    q = np.zeros((len(base) * len(pans), 5))
    q[:, 0] = np.repeat(pans, len(base)); q[:, 1:4] = np.tile(base, (len(pans), 1))
    return q


def sweep_points(q, side, s, n_per_seg=6):
    """Điểm mẫu dọc 4 đoạn bao (trừ đoạn đáy -> vai) kèm bán kính."""
    caps = capsules(skeleton(q, side), base_of(side, s))[1:]
    P, R = [], []
    for (a, b, r) in caps:
        for t in np.linspace(0, 1, n_per_seg):
            P.append(a + (b - a) * t); R.append(np.full(len(a), r))
    return np.concatenate(P), np.concatenate(R)


def mast_clearance(P, R, x_m, z_top):
    """Khe nhỏ nhất (m) giữa vỏ tay và (a) thân cột camera thẳng đứng tại (x_m, 0), z 0..z_top-0,06; (b) cụm gá + camera ở đỉnh
    (cầu tâm x_m+0,03, z_top). Trả (khe thân cột, khe đầu)."""
    g = GIA_DINH
    low = P[:, 2] <= z_top - 0.06
    d_col = (np.hypot(P[low, 0] - x_m, P[low, 1]) - R[low] - g["cot_mast_r"]).min()
    head = np.array([x_m + 0.03, 0.0, z_top])
    d_head = (np.linalg.norm(P - head, axis=1) - R - g["gimbal_r"]).min()
    return round(float(d_col), 4), round(float(d_head), 4)


def head_keepout_ok(q, side, s, x_m, z_top, r_ko=0.12):
    """Số cấu hình còn chạm đầu camera sau khi cấm tâm cổ tay W và TCP vào cầu bán kính r_ko quanh đầu (luật mềm SDK đề xuất)."""
    g = GIA_DINH
    sk = skeleton(q, side); b = base_of(side, s); head = np.array([x_m + 0.03, 0.0, z_top])
    allowed = (np.linalg.norm(sk["T"] + b - head, axis=1) >= r_ko) & (np.linalg.norm(sk["W"] + b - head, axis=1) >= r_ko) \
        & (np.linalg.norm(sk["E"] + b - head, axis=1) >= r_ko)
    hit = np.zeros(len(q), bool)
    for (a, c, r) in capsules(sk, b)[1:]:
        for t in np.linspace(0, 1, 8):
            P = a + (c - a) * t
            hit |= np.linalg.norm(P - head, axis=1) - r - g["gimbal_r"] < 0
    return int(hit.sum()), int((hit & allowed).sum()), int((~allowed).sum())


def plate_hits(q, side, s, x_back=-0.17, x_front=0.03):
    """Cấu hình có vỏ tay xuống dưới mặt ray trong vùng bệ (ngoài trụ đế r 0,07), và còn lại sau luật mềm 'TCP x < 0,10 thì z >= 0,07'."""
    sk = skeleton(q, side); b = base_of(side, s)
    hit = np.zeros(len(q), bool)
    for (a, c, r) in capsules(sk, b)[1:]:
        for t in np.linspace(0, 1, 8):
            P = a + (c - a) * t
            inside = (P[:, 0] > x_back) & (P[:, 0] < x_front) & (np.abs(P[:, 1]) < s / 2 + 0.06)
            far = np.hypot(P[:, 0], P[:, 1] - b[1]) > 0.07
            hit |= inside & far & (P[:, 2] - r < 0.0)
    Tp = sk["T"] + b
    rule = ~((Tp[:, 0] < 0.10) & (Tp[:, 2] < 0.07))
    return int(hit.sum()), int((hit & rule).sum()), int(len(q))


def box_clearance(P, R, lo, hi):
    """Khe nhỏ nhất (m) giữa vỏ tay và hộp trục [lo, hi] (âm = giao)."""
    d = np.maximum(np.maximum(lo - P, P - hi), 0)
    outside = np.linalg.norm(d, axis=1)
    inside = np.all((P > lo) & (P < hi), axis=1)
    return float(np.where(inside, -R, outside - R).min())


# ---------------- 4. FOV camera đầu ----------------
def fov_hv():
    g = GIA_DINH; w, h = g["anh_ty_le"]; dg = math.hypot(w, h)
    t = math.tan(D(g["fov_cheo_deg"]) / 2)
    return 2 * math.degrees(math.atan(t * w / dg)), 2 * math.degrees(math.atan(t * h / dg))


def in_view(P, cam, tilt_deg, H=None, V=None):
    if H is None:
        H, V = fov_hv()
    t = D(tilt_deg)
    f = np.array([math.cos(t), 0, -math.sin(t)]); up = np.array([math.sin(t), 0, math.cos(t)]); rt = np.array([0, -1.0, 0])
    v = P - cam
    zc = v @ f
    ok = zc > 0.05
    return ok & (np.abs(v @ rt) <= zc * math.tan(D(H) / 2)) & (np.abs(v @ up) <= zc * math.tan(D(V) / 2))


def occlusion(cam, cfgL, cfgR, s):
    """Tỷ lệ TCP bị che: đoạn camera -> TCP cắt vỏ tay kia (che chéo) hoặc vỏ vai/cánh/cẳng của chính tay đó (tự che)."""
    out = {}
    skL, skR = skeleton(cfgL, "left"), skeleton(cfgR, "right")
    cL, cR = capsules(skL, base_of("left", s)), capsules(skR, base_of("right", s))
    for name, sk, own, other, side in (("trai", skL, cL, cR, "left"), ("phai", skR, cR, cL, "right")):
        T = sk["T"] + base_of(side, s)
        C = np.repeat(cam[None], len(T), 0)
        # rút đầu đoạn 4 cm trước TCP (thân kẹp quanh TCP không tính là che)
        u = (T - C); L = np.linalg.norm(u, axis=1, keepdims=True); T2 = C + u * (1 - 0.04 / L)
        cross = np.zeros(len(T), bool); selfo = np.zeros(len(T), bool)
        for (a, b, r) in other:
            cross |= seg_seg_dist(C, T2, a, b) < r
        for (a, b, r) in own[:3]:
            selfo |= seg_seg_dist(C, T2, a, b) < r
        out[name] = dict(che_cheo=float(cross.mean()), tu_che=float(selfo.mean()), bat_ky=float((cross | selfo).mean()))
    return out


# ---------------- 5. khối lượng trạm, tải trục nâng, chống lật ----------------
def arm_mass():
    """Khối lượng 1 tay (kg): cận trên SDK (khối đặc URDF + servo + kẹp) và ước tính in (hệ số in PRINT_FACTOR_EST)."""
    solid = sum(spec.LINK_MASS.values())          # LINK_MASS gồm hàm kẹp (jaw)
    est = 0.0
    for b, m in spec.LINK_MASS.items():
        mp = spec.LINK_PRINT[b][0] if b in spec.LINK_PRINT else 0.0
        est += m - mp + mp * spec.PRINT_FACTOR_EST.get(b, 1.0)
    return solid, est


STATION_PARTS = {   # kg, x, y, z trong khung trạm (gốc giữa hai trục pan, đáy J1) [giả định, chờ CAD G2]. Khung chữ U: ray trước + 2 tai sau ôm hai bên cột
    "ray 2040 trước 0,60 m (gá 2 tay, tựa lên khung T giữa)": (0.48, 0.0, 0.0, -0.010),
    "2 tai sau 2040 0,20 m ở y ±0,11 (ôm hai bên cột)": (0.32, -0.10, 0.0, -0.010),
    "ke góc, ốc, con trượt T": (0.15, -0.05, 0.0, -0.010),
    "2 gá đáy tay in (thay khung T trái/phải)": (0.20, 0.0, 0.0, 0.005),
    "cột camera 2020 0,56 m": (0.28, 0.0, 0.0, 0.28),
    "gá pan-tilt in + 2 núm khoá": (0.10, 0.01, 0.0, 0.57),
    "camera đầu H65V1": (0.03, 0.03, 0.0, 0.58),
    "khối ngàm trạm (2 bạc chốt, 2 khoá kéo, đệm tựa)": (0.20, -0.12, 0.0, -0.025),
    "giắc gộp nửa trạm + vỏ (trên tai trái)": (0.06, -0.10, 0.11, -0.035),
    "2 hộp cầu chì tay FB-L/FB-R": (0.16, -0.08, 0.0, 0.01),
    "2 bo bus + 2 cách ly USB + hub + buck 5 V": (0.14, -0.08, 0.0, 0.01),
    "nút E-stop + hộp": (0.12, -0.05, 0.12, 0.03),
    "dây trên trạm (8 AWG ngắn, 16/22 AWG, USB)": (0.25, -0.06, 0.0, 0.0),
    "4 chân đệm cao su": (0.04, -0.08, 0.0, -0.025),
}
CARRIAGE_KEEP = {   # phần xe nâng giữ lại (khung T giữa, servo, bánh răng, chốt, công tắc) + khối ngàm xe [giả định]; khung robot
    "khung T giữa + servo + bánh răng + chốt + công tắc (CAD S1G)": (0.272 + 0.1875 + 0.0127 + 0.06 + 0.0098 + 0.005 + 0.0042 + 0.0452 + 0.0178, 0.045, 0.0, 0.50),
    "khối ngàm xe in + 2 chốt côn thép + nửa giắc": (0.30, 0.0, 0.0, 0.55),
}


def station_mass_com():
    m = sum(v[0] for v in STATION_PARTS.values())
    c = sum(np.array(v[1:]) * v[0] for v in STATION_PARTS.values()) / m
    return m, c


def lift_load(arm_kg, payload_each=0.5):
    """Tải treo trên bánh răng (kg) và lực/mô-men cần ở 0,5 m/s², so giới hạn spec (TORQUE_LIMIT, F_đo giữ chỗ 120 N)."""
    ms, _ = station_mass_com()
    mc = sum(v[0] for v in CARRIAGE_KEEP.values())
    total = ms + mc + 2 * arm_kg + 2 * payload_each
    F = total * (spec.G + 0.5)
    tau_kgcm = F * spec.LIFT_PINION_R / spec.KGCM
    return dict(tram_kg=round(ms, 3), xe_nang_giu_kg=round(mc, 3), tay_kg=round(2 * arm_kg, 3), tai_kg=2 * payload_each,
                tong_kg=round(total, 3), luc_N=round(F, 1), momen_kgcm=round(tau_kgcm, 2),
                pct_dinh_muc=round(100 * tau_kgcm / spec.STS3095.rated_kgcm, 1),
                gioi_han_hien_tai_N=round(spec.LIFT_FORCE_LIMIT_N, 1), du=bool(spec.LIFT_FORCE_LIMIT_N >= F),
                F_rang_can_do_N=round(F / spec.LIFT_TOOTH_SAFETY, 1), cu_LIFT_DESIGN_LOAD_KG=spec.LIFT_DESIGN_LOAD_KG)


def tipover_mobile(s, arm_override=None, cw=None):
    """Chạy lại tipover.pl_min: bỏ 'xe_nang_khung_T' (CAD S1G), thêm phần xe giữ lại + trạm (khung robot, lift 0 -> z gốc tay 0,4137)."""
    import tipover as TP
    M = json.load(open(os.path.join(ROOT, "cad", "export", "M1-PL-masses.json")))
    M = json.loads(json.dumps(M))
    M["co_dinh"].pop("xe_nang_khung_T")
    zb = spec.ARM_BASE_Z0
    for k, v in CARRIAGE_KEEP.items():
        M["co_dinh"]["xe_" + k] = [v[0], v[1], v[2], v[3]]
    ms, c = station_mass_com()
    M["co_dinh"]["tram_khong_tay"] = [ms, spec.SHOULDER_X + c[0], c[1], zb + c[2]]
    M["pan_xy"] = [spec.SHOULDER_X, s / 2]
    res = {}
    if arm_override is None:
        r = TP.pl_min(M, cw=cw); r0 = TP.pl_min(M, cw=cw, tray_load=False)
        res = dict(co_khay_1kg=round(r[0], 3), khong_tai_khay=round(r0[0], 3), pan=(r[1], r[2]))
        res["gia_toc_0_5"] = round(TP.pl_min(M, cw=cw, acc=0.5, z_sh=zb + spec.D0 + 0.45)[0], 3)
        res["doi_trong_can_1_5_kg"] = round(TP.pl_cw_needed(M), 2)
    else:
        # tay nặng hơn (cận trên SDK): thay khối lượng tay trong cấu hình
        cfg, a = TP.pl_cfg(M)
        cfg["arm_m"] = arm_override
        x, y, z = M["khay_goc_xa"]
        worst = min(TP.min_factor(dict(cfg, fixed=list(cfg["fixed"]) + [(1.3, x, sy * y, z)]))[0] for sy in (1, -1))
        res = dict(co_khay_1kg=round(worst, 3), arm_m=arm_override)
    return res


def tipover_bench(arm_kg, arm_r, tcp_r, payload=0.5, foot=(-0.20, 0.08, 0.32), dock_kg=1.0, extra_kg=0.0):
    """Trạm + đế bàn đứng tự do trên bàn (không kẹp): hệ số giữ/lật quanh mép trước và mép bên, tay duỗi ngang, tải ở TCP,
    pan xấu nhất trong ±70. foot = (x sau, x trước, nửa rộng y) của chân đế bàn. Trả hệ số nhỏ nhất và lực kẹp cần."""
    ms, c = station_mass_com()
    xb, xf, yh = foot
    fixed = [(ms, c[0], c[1]), (dock_kg, (xb + xf) / 2 - 0.02, 0.0)]
    if extra_kg:
        fixed.append((extra_kg, xb + 0.03, 0.0))
    worst = (math.inf, None)
    for s in (0.475,):
        for pl in np.arange(-70, 71, 5):
            for pr in np.arange(-70, 71, 5):
                pts = list(fixed)
                for side, p in ((1, pl), (-1, pr)):
                    a = D(p) * -side
                    u = np.array([math.cos(a), math.sin(a)]); b = np.array([0.0, side * s / 2])
                    pts.append((arm_kg, *(b + arm_r * u))); pts.append((payload, *(b + tcp_r * u)))
                for n, d in ((np.array([1, 0]), xf), (np.array([0, 1]), yh), (np.array([0, -1]), yh), (np.array([-1, 0]), -xb)):
                    tip = hold = 0.0
                    for m, x, y in pts:
                        sd = n @ np.array([x, y]) - d
                        if sd > 0:
                            tip += m * sd
                        else:
                            hold += m * -sd
                    f = hold / tip if tip > 0 else math.inf
                    if f < worst[0]:
                        worst = (f, (float(pl), float(pr), n.tolist()), tip, hold)
    return worst


# ---------------- chạy ----------------
def main(plots=True):
    rng = np.random.default_rng(24)
    g = GIA_DINH
    z_tab = -g["h_j1_tren_ban"]
    out = dict(gia_dinh=g, khoang_vai={}, fov={}, cot_camera={}, gioi_han="giới hạn khớp và bảng ghép spec.py (pan ±70, vai -80..48,5, khuỷu theo vai, cổ tay theo vai/khuỷu)")
    # kiểm khung xương khớp SDK
    qt = np.array([[0.3, -0.4, 0.9, 0.5, 0.2]])
    for side in ("left", "right"):
        assert np.allclose(skeleton(qt, side)["T"][0], K.arm_fk(qt[0], side)[0], atol=1e-9)
        assert np.allclose(skeleton(qt, side)["F"][0], K.fingertip_fk(qt[0], side), atol=1e-9)
    maps = {}
    for s in SPACINGS:
        xs, ys, R = reach_map(s, z_tab + g["z_vat"])
        mL, QL = R["left"]; mR, QR = R["right"]
        both = mL & mR
        cell = (xs[1] - xs[0]) * (ys[1] - ys[0])
        w, xr = band_width(both, ys, xs)
        # đích cho va chạm: ô tới được ở 2 độ cao (gắp sát bàn và nhấc 0,12 m)
        xs2, ys2, R2 = reach_map(s, z_tab + 0.15, step=0.02)
        Qs = {}
        for side, (m, Q), (m2, Q2) in (("left", R["left"], R2["left"]), ("right", R["right"], R2["right"])):
            Y1 = np.repeat(ys[:, None], len(xs), 1)[m]; Y2 = np.repeat(ys2[:, None], len(xs2), 1)[m2]
            Qs[side] = dict(q=np.concatenate([Q[m], Q2[m2]]), y=np.concatenate([Y1, Y2]))
        cs, _, _, _ = collision_stats(s, Qs, rng)
        rk = risk_map(s, xs, ys, mL, QL, Qs, rng)
        risk_area = float(np.nansum(rk > 0.05) * cell)
        cross = float((mL & (np.repeat(ys[:, None], len(xs), 1) < 0)).sum() * cell)
        out["khoang_vai"][f"{s:.3f}"] = dict(
            dien_tich_trai_m2=round(float(mL.sum() * cell), 4), dien_tich_hop_m2=round(float((mL | mR).sum() * cell), 4),
            dien_tich_chung_m2=round(float(both.sum() * cell), 4), be_rong_chung_max_m=round(w, 3),
            dai_x_chung_m=xr, be_rong_phu_y_m=round(float((mL | mR).any(1).sum() * (ys[1] - ys[0])), 3),
            tay_trai_qua_tam_m2=round(cross, 4), vung_rui_ro_cham_m2=round(risk_area, 4),
            ty_le_o_rui_ro_trong_vung_chung=round(float(np.nansum((rk > 0.05) & both) / max(both.sum(), 1)), 3), **cs)
        maps[s] = (xs, ys, mL, mR, rk)
        print(f"s={s:.3f}", out["khoang_vai"][f"{s:.3f}"])
    S_CHON = 0.475
    out["khoang_vai_chon_m"] = S_CHON

    # vùng quét: cột camera, bệ, cột nâng (lift 0 và 0,45) ở khoảng chọn
    q = sweep_configs()
    for s in (S_CHON, 0.45, 0.50):
        PL, RL = sweep_points(q, "left", s); PR, RR = sweep_points(q, "right", s)
        P = np.concatenate([PL, PR]); R = np.concatenate([RL, RR])
        z_top = g["cam_tren_ban"] - g["h_j1_tren_ban"]
        cl = {f"{xm:+.2f}": dict(zip(("than_cot", "dau_camera"), mast_clearance(P, R, xm, z_top))) for xm in (-0.06, -0.04, -0.02, 0.0, 0.03)}
        ko = dict(zip(("cau_hinh_cham_dau", "con_cham_sau_luat_cau_0_12", "cau_hinh_bi_cam"), head_keepout_ok(q, "left", s, 0.0, z_top)))
        bed = dict(zip(("cau_hinh_cham_be", "con_lai_sau_luat_TCP", "tong_cau_hinh"), plate_hits(q, "left", s)))
        # cột nâng ở chế độ di động: hộp Post1-2 (spec.COLUMN_*) đổi sang khung trạm; z theo lift
        lo_hi = {}
        for lift in (0.0, 0.45):
            dx = -spec.SHOULDER_X; dz = -(spec.ARM_BASE_Z0 + lift)
            lo = np.array([spec.COLUMN_XR[0] + dx, -spec.COLUMN_YH, spec.BASE_Z1 + dz]); hi = np.array([spec.COLUMN_XR[1] + dx, spec.COLUMN_YH, spec.COLUMN_TOP + dz])
            lo_hi[f"lift_{lift}"] = round(box_clearance(P, R, lo, hi), 4)
        out["cot_camera"][f"{s:.3f}"] = dict(khe_cot_camera_theo_x_m=cl, luat_cam_dau_x0=ko, be_tay_trai=bed, khe_cot_nang=lo_hi,
                                             x_sau_nhat_vo_tay_tren_mat_ray=round(float((P[:, 0] - R).min()), 4))
        print("sweep", s, out["cot_camera"][f"{s:.3f}"])

    # FOV: camera trên cột tại x_m, cao 0,65 m trên bàn
    H, V = fov_hv()
    out["fov"]["H_V_deg"] = (round(H, 1), round(V, 1))
    xs, ys, mL, mR, _ = maps[S_CHON]
    X, Y = np.meshgrid(xs, ys)
    U = (mL | mR)
    Ptab = np.stack([X[U], Y[U], np.full(U.sum(), z_tab + g["z_vat"])], 1)
    Pgrip = Ptab + np.array([0, 0, 0.12])
    for xm in (-0.04, -0.02, 0.0):
        cam = np.array([xm, 0.0, g["cam_tren_ban"] - g["h_j1_tren_ban"]])
        rows = {}
        for tilt in range(25, 71, 5):
            a = in_view(Ptab, cam, tilt).mean(); b = in_view(Pgrip, cam, tilt).mean()
            near = cam[2] - z_tab
            rows[tilt] = dict(phu_mat_ban=round(float(a), 3), phu_kep_cao_12cm=round(float(b), 3),
                              mep_gan_tren_ban_x_m=round(xm + near / math.tan(D(tilt + V / 2)), 3) if tilt + V / 2 < 90 else None,
                              truc_quang_cham_ban_x_m=round(xm + near / math.tan(D(tilt)), 3))
        out["fov"][f"x_m={xm:+.2f}"] = rows
    # vùng thao tác điển hình trước trạm: x 0,15..0,50, |y| <= 0,35 (bàn 0,35 x 0,70 m)
    tx, ty = np.meshgrid(np.arange(0.15, 0.501, 0.01), np.arange(-0.35, 0.351, 0.01))
    Pt = np.stack([tx.ravel(), ty.ravel(), np.full(tx.size, z_tab + g["z_vat"])], 1)
    for xm in (-0.02, 0.0):
        cam = np.array([xm, 0.0, g["cam_tren_ban"] - g["h_j1_tren_ban"]])
        out["fov"][f"vung_thao_tac_x_m={xm:+.2f}"] = {t: dict(mat_ban=round(float(in_view(Pt, cam, t).mean()), 3),
                                                              kep_cao_12cm=round(float(in_view(Pt + [0, 0, 0.12], cam, t).mean()), 3))
                                                       for t in (35, 40, 45, 50, 55, 60)}
    # nhạy với FOV: coi 80° là FOV ngang (rộng hơn)
    Hh = 80.0; Vh = 2 * math.degrees(math.atan(math.tan(D(40)) * 3 / 4))
    cam = np.array([0.0, 0.0, g["cam_tren_ban"] - g["h_j1_tren_ban"]])
    out["fov"]["nhay_80_la_ngang"] = {t: round(float(in_view(Ptab, cam, t, Hh, Vh).mean()), 3) for t in (35, 45, 50, 55)}
    out["fov"]["nhay_fovy70_sim"] = {t: round(float(in_view(Ptab, cam, t, 70 * 4 / 3 * 0.95, 70).mean()), 3) for t in (35, 45, 50, 55)}

    # che khuất: đích ngẫu nhiên hai tay, camera x_m 0 (cột trên đường trục pan)
    QL = maps[S_CHON]
    xs2, ys2, R2 = reach_map(S_CHON, z_tab + g["z_vat"], step=0.02)
    qL = R2["left"][1][R2["left"][0]]; qR = R2["right"][1][R2["right"][0]]
    n = 4000
    cL = qL[rng.integers(0, len(qL), n)]; cR = qR[rng.integers(0, len(qR), n)]
    hit = arm_collide(capsules(skeleton(cL, "left"), base_of("left", S_CHON)), capsules(skeleton(cR, "right"), base_of("right", S_CHON)))
    cL, cR = cL[~hit], cR[~hit]
    out["fov"]["che_khuat"] = {t: occlusion(np.array([0.0, 0, g["cam_tren_ban"] - g["h_j1_tren_ban"]]), cL, cR, S_CHON) for t in (35, 50)}

    # khối lượng, trục nâng, chống lật
    solid, est = arm_mass()
    ms, c = station_mass_com()
    out["khoi_luong"] = dict(tay_can_tren_sdk_kg=round(solid, 3), tay_uoc_in_kg=round(est, 3), tram_khong_tay_kg=round(ms, 3),
                             tram_khong_tay_tam=[round(v, 3) for v in c], tram_ca_tay_uoc_kg=round(ms + 2 * est, 3),
                             tram_ca_tay_can_tren_kg=round(ms + 2 * solid, 3), chi_tiet=STATION_PARTS)
    out["truc_nang"] = dict(uoc_in=lift_load(est), can_tren=lift_load(solid), tai_1kg_moi_tay=lift_load(est, 1.0))
    import tipover as TP
    a = TP.arm_pl()
    out["chong_lat_di_dong"] = {f"{s:.3f}": tipover_mobile(s) for s in (0.37452, 0.45, S_CHON, 0.50)}
    out["chong_lat_di_dong"]["0.475_tay_can_tren_sdk"] = tipover_mobile(S_CHON, arm_override=solid)
    out["chong_lat_di_dong"]["ghi_chu"] = f"tay theo tipover.arm_pl (URDF {a['m']:.2f} kg, tâm {a['r']:.3f} m, TCP {a['tcp']:.3f} m), tải 1 kg/tay ở TCP, khay 1 kg góc xa, đối trọng 8,23 kg"
    for fk, extra in (("tu_do", 0.0), ("them_3kg_sau", 3.0)):
        f, cfgw, tip, hold = tipover_bench(est, a["r"], a["tcp"], payload=0.5, extra_kg=extra)
        f1, _, tip1, _ = tipover_bench(est, a["r"], a["tcp"], payload=1.0, extra_kg=extra)
        out.setdefault("chong_lat_ban", {})[fk] = dict(he_so_tai_0_5=round(f, 3), he_so_tai_1=round(f1, 3), canh=cfgw,
                                                      momen_lat_Nm_tai_1=round(9.81 * tip1, 2))
    # lực kẹp: mô-men lật (tải 1 kg, hệ số động 2) chia cánh tay đòn kẹp 0,25 m (2 kẹp ở mép sau đế bàn)
    Mt = out["chong_lat_ban"]["tu_do"]["momen_lat_Nm_tai_1"] * 2
    out["chong_lat_ban"]["luc_kep_can_moi_kep_N"] = round(Mt / 0.25 / 2, 1)
    # tải khoá ngàm (di động): mô-men quanh mép trước bàn tựa (x = -0,06 khung trạm), khoá ở x = -0,17 (cánh tay đòn 0,11 m)
    ms, c = station_mass_com()
    pts = [(ms, c[0]), (2 * est, a["r"]), (2 * 1.0, a["tcp"])]
    Mf = 9.81 * sum(m * (x - 0.01) for m, x in pts)
    out["ngam"] = dict(momen_quanh_mep_tua_Nm=round(Mf, 2), he_so_dong=2.5, luc_moi_khoa_N=round(2.5 * Mf / 0.19 / 2, 1),
                       ghi_chu="tải 1 kg/tay ở TCP duỗi thẳng trước; mép tựa trước (mặt trước khung T giữa) x +0,01; 2 khoá kéo trên 2 tai ở x -0,18 (cánh tay đòn 0,19 m) [giả định]")
    os.makedirs(os.path.join(ROOT, "cad", "export"), exist_ok=True)
    with open(os.path.join(ROOT, "cad", "export", "M1-TRAM-r3.json"), "w") as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    if plots:
        draw(maps, out)
    return out


def draw(maps, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    d = os.path.join(ROOT, "media", "qc", "r3"); os.makedirs(d, exist_ok=True)
    fig, axs = plt.subplots(1, 4, figsize=(13, 4.2), sharey=True)
    cmap = ListedColormap(["#ffffff", "#9ecae1", "#fdae6b", "#31a354"])
    for ax, s in zip(axs, SPACINGS):
        xs, ys, mL, mR, rk = maps[s]
        img = mL.astype(int) + 2 * mR.astype(int)
        ax.imshow(img, origin="lower", extent=(xs[0], xs[-1], ys[0], ys[-1]), cmap=cmap, vmin=0, vmax=3, interpolation="nearest")
        X, Y = np.meshgrid(xs, ys)
        ax.contour(X, Y, np.nan_to_num(rk, nan=0.0), levels=[0.05], colors="#d62728", linewidths=1.2)
        ax.plot([0, 0], [-s / 2, s / 2], "ko", ms=4)
        k = out["khoang_vai"][f"{s:.3f}"]
        ax.set_title(f"vai {s*100:.1f} cm\nchung {k['dien_tich_chung_m2']*1e4:.0f} cm², chạm {k['p_cham_ngau_nhien']*100:.1f}%", fontsize=9)
        ax.set_xlabel("x trước (m)"); ax.set_aspect("equal"); ax.set_xlim(-0.1, 0.75)
    axs[0].set_ylabel("y trái (m)")
    fig.suptitle("Vùng gắp trên bàn (tâm kẹp 3 cm trên bàn): xanh lam = chỉ tay trái, cam = chỉ tay phải, xanh lá = cả hai; viền đỏ = ô tay trái có >5% đích tay phải gây chạm", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(d, "r3-vung-lam-viec.png"), dpi=80); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    for key, st in (("x_m=-0.02", "-"), ("x_m=+0.00", "--")):
        rows = out["fov"][key]; t = sorted(rows)
        ax[0].plot(t, [rows[k]["phu_mat_ban"] * 100 for k in t], st, label=f"vùng gắp trên bàn, cột {key[4:]}")
        ax[0].plot(t, [rows[k]["phu_kep_cao_12cm"] * 100 for k in t], st, label=f"kẹp nhấc 12 cm, cột {key[4:]}")
    ax[0].axvline(35, color="grey", lw=0.8); ax[0].set_xlabel("góc chúi camera (độ)"); ax[0].set_ylabel("% vùng nằm trong khung hình")
    ax[0].legend(fontsize=7); ax[0].set_title("Camera đầu 0,65 m trên bàn, FOV 80° chéo 4:3 [giả định]", fontsize=9)
    xs, ys, mL, mR, _ = maps[0.475]
    ax[1].imshow((mL | mR).astype(int), origin="lower", extent=(xs[0], xs[-1], ys[0], ys[-1]), cmap=ListedColormap(["#ffffff", "#c7e9c0"]))
    H, V = out["fov"]["H_V_deg"]
    zc = GIA_DINH["cam_tren_ban"] - GIA_DINH["z_vat"]
    for tilt, col in ((35, "#d62728"), (50, "#3182bd")):
        pts = []
        for sv, sh in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
            t = D(tilt); f = np.array([math.cos(t), 0, -math.sin(t)]); up = np.array([math.sin(t), 0, math.cos(t)]); rt = np.array([0, -1.0, 0])
            ray = f + sh * math.tan(D(H) / 2) * rt + sv * math.tan(D(V) / 2) * up
            if ray[2] >= 0:
                ray[2] = -1e-3
            k = zc / -ray[2]; pts.append((0.0 + k * ray[0], k * ray[1]))
        pts.append(pts[0]); P = np.array(pts)
        ax[1].plot(P[:, 0], P[:, 1], color=col, label=f"chúi {tilt}°")
    ax[1].set_xlim(-0.1, 1.2); ax[1].set_ylim(-0.75, 0.75); ax[1].set_aspect("equal"); ax[1].legend(fontsize=7)
    ax[1].set_title("Vết khung hình trên mặt bàn, vai 47,5 cm", fontsize=9); ax[1].set_xlabel("x trước (m)")
    fig.tight_layout(); fig.savefig(os.path.join(d, "r3-fov-camera-dau.png"), dpi=80); plt.close(fig)


if __name__ == "__main__":
    main()
