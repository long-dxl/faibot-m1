# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Hệ số chống lật tĩnh cho đế 3 bánh omni (M1 v0.5). Chỉ numpy, không cần build123d.

Mô hình (như R1b mục 4.1): nền phẳng, tĩnh. Mép lật là 3 cạnh tam giác nối điểm chạm đất của 3 bánh. Với mỗi cạnh, khối nằm ngoài cạnh
gây lật, khối nằm trong cạnh giữ; hệ số = mô-men giữ / mô-men lật. Lấy cạnh xấu nhất, rồi tổ hợp pan xấu nhất của hai tay (mỗi tay độc lập
trong ±PAN). Tay duỗi ngang (xấu nhất về tầm), tải ở đầu kẹp. Có thể thêm gia tốc ngang a (m/s²) đặt lên mọi khối theo độ cao z
(gần đúng nền nghiêng / phanh): lực quán tính m·a hướng ra ngoài cạnh đang xét cộng vào mô-men lật.
Hệ toạ độ như SDK: x trước, y trái, gốc tâm đế, mét, kg.

Chạy: python tipover.py            -> in bảng tái lập R1b + số cấu hình P-L (cad/export/M1-PL-masses.json, sinh bởi assembly_pl.py --masses)
"""
import json, math, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81


def edges(R, angles_deg):
    """Cạnh tam giác bánh: list (n, d) với n pháp tuyến ngoài (2D), d khoảng cách cạnh tới tâm đế. Điểm p ngoài cạnh khi p·n > d."""
    P = [np.array([R * math.cos(math.radians(a)), R * math.sin(math.radians(a))]) for a in angles_deg]
    out = []
    for i in range(len(P)):
        a, b = P[i], P[(i + 1) % len(P)]
        t = b - a; n = np.array([t[1], -t[0]]) / np.linalg.norm(t)
        if n @ (a + b) / 2 < 0: n = -n
        out.append((n, float(n @ a)))
    return out


def factor(masses, edge_list, acc=0.0):
    """masses: list (m, x, y[, z]). Trả (hệ số nhỏ nhất, chỉ số cạnh)."""
    best = (math.inf, -1)
    for k, (n, d) in enumerate(edge_list):
        tip = hold = 0.0
        for m in masses:
            s = n @ np.array(m[1:3]) - d          # >0: ngoài cạnh
            if s > 0: tip += m[0] * s
            else: hold += m[0] * -s
            if acc and len(m) > 3: tip += m[0] * acc / G * m[3]
        f = hold / tip if tip > 1e-12 else math.inf
        if f < best[0]: best = (f, k)
    return best


def arm_points(pan_xy, side, pan_deg, arm_m, arm_r, load_m, load_r, z_sh=0.55):
    """Khối của một tay duỗi ngang (tâm cách trục pan arm_r) + tải ở load_r. pan dương = quay vào trong (về mặt y = 0)."""
    a = math.radians(pan_deg) * -side            # tay trái (side +1) quay vào trong là quay về -y
    u = np.array([math.cos(a), math.sin(a)]); p = np.array([pan_xy[0], side * pan_xy[1]])
    out = [(arm_m, *(p + arm_r * u), z_sh)]
    if load_m: out.append((load_m, *(p + load_r * u), z_sh))
    return out


def min_factor(cfg, pan_max=70.0, step=5.0, acc=0.0, z_sh=0.55):
    """cfg: dict(R, angles, pan_xy, arm_m, arm_r, load_m, load_r, fixed=[(m,x,y[,z])...]). Trả (hệ số nhỏ nhất, pan trái, pan phải, cạnh)."""
    E = edges(cfg["R"], cfg["angles"])
    pans = np.arange(-pan_max, pan_max + 1e-9, step)
    best = (math.inf, None, None, None)
    for pl in pans:
        A = arm_points(cfg["pan_xy"], 1, pl, cfg["arm_m"], cfg["arm_r"], cfg["load_m"], cfg["load_r"], z_sh)
        for pr in pans:
            B = arm_points(cfg["pan_xy"], -1, pr, cfg["arm_m"], cfg["arm_r"], cfg["load_m"], cfg["load_r"], z_sh)
            f, k = factor(cfg["fixed"] + A + B, E, acc)
            if f < best[0]: best = (f, float(pl), float(pr), k)
    return best


# ---------------- bảng cũ (docs M1-so-sanh-...-ky-thuat mục 6): một mép trước, tay duỗi thẳng trước ----------------
def old_table():
    """M1 v0.4: bánh 60/180/300 độ, R 0,17 -> mép trước x = 0,085. Hai tay 3,57 kg tâm cách vai 0,158 m, vai x 0,12; tải 1 kg/tay ở x 0,644;
    xe nâng 1,75 kg ở x -0,05; đế + cột + pin 12,0 kg ở x_b -0,05. Kỳ vọng 1,81 / 1,85 / 1,03."""
    E = edges(0.17, (60, 180, 300))
    m = [(3.57, 0.12 + 0.158, 0.0), (2 * 1.0, 0.644, 0.0), (1.75, -0.05, 0.0), (12.0, -0.05, 0.0)]
    n, d = E[[k for k, (n, _) in enumerate(E) if n[0] > 0.99][0]]
    tip = sum(a * (n @ np.array([x, y]) - d) for a, x, y in m if n @ np.array([x, y]) > d)
    hold = sum(a * (d - n @ np.array([x, y])) for a, x, y in m if n @ np.array([x, y]) <= d)
    return dict(lat=tip, giu=hold, he_so=hold / tip)


# ---------------- mô hình R1b (đế R0,25, đỉnh bánh ra trước) ----------------
R1B_BASES = {"B1": (9.0, -0.02), "B2": (11.0, -0.02), "B3": (12.0, -0.05)}     # kg, x tâm (R1b bảng 4.1)


def r1b_cfg(base="B2", cw=0.0, tray=False, arm_m=1.30, R=0.25, angles=(0, 120, 240), cw_x=-0.10):
    mb, xb = R1B_BASES[base]
    fixed = [(mb, xb, 0.0, 0.25), (1.5, -0.058 + 0.041, 0.0, 0.50)]     # đế; xe nâng + khung T 1,5 kg (trục cột -0,058, tâm lùi về trước 0,041)
    if cw: fixed.append((cw, cw_x, 0.0, 0.08))
    cfg = dict(R=R, angles=angles, pan_xy=(0.060, 0.187), arm_m=arm_m, arm_r=0.182, load_m=1.0, load_r=0.584, fixed=fixed)
    return cfg


def r1b_min(base="B2", cw=0.0, tray=False, arm_m=1.30, **kw):
    cfg = r1b_cfg(base, cw, tray, arm_m, **kw)
    if not tray:
        return min_factor(cfg)[0]
    # khay 0,3 kg + tải 1 kg dồn ở góc xa (0,30; ±0,15) như R1b (bảo thủ: cả khối khay đặt ở góc), lấy phía xấu hơn.
    # Đặt khay ở giữa (0,22; 0) thì hệ số cao hơn 0,04-0,06 (xem tests/test_tipover.py).
    res = []
    for sy in (1, -1):
        c = dict(cfg); c["fixed"] = cfg["fixed"] + [(1.3, 0.30, 0.15 * sy, 0.21)]
        res.append(min_factor(c)[0])
    return min(res)


# ---------------- cấu hình P-L (23/11/2026): 2 tay AM-ARM200 nguyên bản trên khung T AlohaMini2, đế kiểu LeKiwi nối nan R0,25 ----------------
def arm_pl():
    """Một tay AM-ARM200 theo URDF upstream (amarm.py): khối lượng bỏ link camera giữ chỗ, cộng camera cổ tay 0,03 kg [giả định, R1b];
    tâm khối lượng và đầu kẹp tính ở tư thế duỗi ngang ra trước (lift -162,5°, elbow 163,5°, wflex -13,5°), khoảng cách ngang từ trục pan."""
    import amarm as AM
    q = AM.POSE_DEG["straight"]; qr = {k: math.radians(v) for k, v in q.items()}
    m, c = AM.mass_com(qr)
    cam_m, cam_r = 0.03, float(-AM.tcp(qr, 0.0)[1])
    r = (m * float(-c[1]) + cam_m * cam_r) / (m + cam_m)
    return dict(m=m + cam_m, r=r, tcp=float(-AM.tcp(qr)[1]), z_com=float(c[2]))


def pl_cfg(masses, load_m=1.0):
    """masses: dict từ cad/export/M1-PL-masses.json: R, angles, pan_xy (m, hệ robot), co_dinh {tên: [kg, x, y, z]}, khay_goc_xa [x, y, z]."""
    a = arm_pl()
    fixed = [tuple(v) for v in masses["co_dinh"].values()]
    return dict(R=masses["R"], angles=tuple(masses["angles"]), pan_xy=tuple(masses["pan_xy"]), arm_m=a["m"], arm_r=a["r"], load_m=load_m, load_r=a["tcp"], fixed=fixed), a


def pl_min(masses, cw=None, tray_load=True, acc=0.0, z_sh=0.72, pan_max=70.0, arm=None):
    """Hệ số nhỏ nhất (tổ hợp pan ±pan_max, phía khay xấu hơn). cw: thay khối lượng đối trọng trong masses (giữ vị trí).
    arm: (kg, tâm cách trục pan m) thay tay URDF (vd. tay CAD đủ servo, assembly_pl.tay_cad)."""
    cfg, a = pl_cfg(masses)
    if arm is not None: cfg["arm_m"], cfg["arm_r"] = arm
    fixed = list(cfg["fixed"])
    if cw is not None:
        k = list(masses["co_dinh"]).index(masses["doi_trong_key"]); v = list(fixed[k]); v[0] = cw; fixed[k] = tuple(v)
    x, y, z = masses["khay_goc_xa"]
    cands = [[]] if not tray_load else [[(1.3, x, y, z)], [(1.3, x, -y, z)]]      # khay 0,3 + tải 1 dồn ở góc xa (quy ước R1b)
    worst = None
    for c in cands:
        cc = dict(cfg); cc["fixed"] = fixed + c
        r = min_factor(cc, pan_max=pan_max, acc=acc, z_sh=z_sh)
        if worst is None or r[0] < worst[0]: worst = r
    return worst


def pl_cw_needed(masses, target=1.5, **kw):
    """Đối trọng (kg) nhỏ nhất ở vị trí đã đặt để hệ số >= target (chia đôi 0..20 kg). None nếu 20 kg chưa đủ hoặc đối trọng làm lật sau."""
    f = lambda w: pl_min(masses, cw=w, **kw)[0]
    if f(0.0) >= target: return 0.0
    if f(20.0) < target: return None
    lo, hi = 0.0, 20.0
    for _ in range(30):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if f(mid) < target else (lo, mid)
    return hi


if __name__ == "__main__":
    print("bảng cũ:", {k: round(v, 3) for k, v in old_table().items()})
    for arm in (1.30, 1.75):
        for tray in (False, True):
            for cw in (0, 3, 5):
                print(f"tay {arm} khay {tray} đối trọng {cw}:", [round(r1b_min(b, cw, tray, arm), 2) for b in ("B1", "B2", "B3")])
    print("tay AM-ARM200 (URDF):", {k: round(v, 4) for k, v in arm_pl().items()})
    p = os.path.join(HERE, "../export/M1-PL-masses.json")
    if os.path.exists(p):
        M = json.load(open(p))
        for cw in (0.0, M["co_dinh"][M["doi_trong_key"]][0]):
            print(f"P-L đối trọng {cw:.2f} kg: có khay 1 kg {pl_min(M, cw)[0]:.3f}; không tải khay {pl_min(M, cw, tray_load=False)[0]:.3f}")
        print("đối trọng cần cho 1,5 (khay 1 kg góc xa):", pl_cw_needed(M))
