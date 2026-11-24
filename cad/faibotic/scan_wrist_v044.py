# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""v0.4.4: quét giao cắt cụm cổ tay (giá J5, chốt J5, vỏ/đáy/nắp J56, khoá yaw M1-AR-025, kẹp J7, ngón, camera cổ tay) khi gập quanh trục wrist_flex URDF.
 Hình học lấy từ assembly_full.py (cùng phép đặt, không chép số). Thay scan_wrist_v043.py (cổ tay quay quanh chốt J5, hình cũ).
Vật cản như v0.4.3 (giá chặn dưới M1-SH-017 + đệm M1-SH-018, thanh chặn trên M1-SH-016 + đệm M1-SH-015, gá vai M1-SH-003, link M1-SH-004, chốt cân bằng, hai lò xo)
+ đệm/tab trên link (M1-AR-022/023/024) + cẳng tay của chính tay đó (gá khuỷu M1-AR-020 + 2 servo khuỷu, ống M1-AR-021, ngàm M1-AR-026, servo gập): tự va chạm.
Cuộn: kẹp, ngón, camera quay quanh trục wrist_roll URDF (khung cẳng tay: x dọc, qua (y -3,81; z -26,48)).
Tự va chạm với cẳng tay không phụ thuộc vai/khuỷu: tính một lần theo (cổ tay, cuộn) ở chế độ --self, ghép vào từng dòng ra.
Chạy: python scan_wrist_v044.py LUOI.json RA.json   (LUOI.json = {"shoulders":[..],"elbows":[..],"wrists":[..],"rolls":[..]} hoặc {"cells":[[qs,qe,[qw..]],..],"rolls":[..]});
dòng ra [qs, qe, qw, cuộn, {chi tiết|vật cản: mm3}]. Biến M1_PROCS = số tiến trình (mặc định 2).
22/11 (a, quét theo cuộn): lọc hộp bao bằng 8 góc hộp bao cục bộ nhân ma trận đặt (không dời hình khi hộp không chạm: cùng kết quả, nhanh hơn ~10 lần);
chế độ quét cuộn: python scan_wrist_v044.py --sweep RA.json [BIEN_DEG]  (lưới vai x khuỷu của bảng, cuộn ROLLS, cổ tay thô COARSE rồi tinh 2,5 quanh mức giao đầu tiên;
bảng = nhỏ nhất qua mọi cuộn của mức sạch lớn nhất (liền từ 0) trừ biên, trần giới hạn gập WF_MAX)."""
import sys, os, json, time, itertools
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from multiprocessing import Pool
from build123d import Pos, Rot
import assembly_full as A
A.GRIPPER = "am"   # G2B: công cụ này dựng cho kẹp J7 AM-ARM200 (v0.4.x); kẹp cũ v0.5 chưa vào sim/web (C1, C2)
import balance as b, failsafe as f, failsafe_v042 as g2, failsafe_v042_low as L, failsafe_v043 as g3
from params import LINK_LEN
G = {}

def vol(a, c):
    """Thể tích giao (mm3), lọc hộp bao trước; không tính khoảng cách (nhanh hơn balance.inter)."""
    ba, bc = a.bounding_box(), c.bounding_box()
    if ba.min.X > bc.max.X or ba.max.X < bc.min.X or ba.min.Y > bc.max.Y or ba.max.Y < bc.min.Y or ba.min.Z > bc.max.Z or ba.max.Z < bc.min.Z:
        return 0.0
    try:
        r = a & c; return 0.0 if r is None else float(r.volume)
    except Exception:
        return float("nan")

def _mat(loc):
    T = loc.wrapped.Transformation()
    return np.array([[T.Value(i, j) for j in range(1, 5)] for i in range(1, 4)])

def _bb(s):
    b = s.bounding_box(); return np.array([[b.min.X, b.min.Y, b.min.Z], [b.max.X, b.max.Y, b.max.Z]])

def _bbw(bb, M):
    """Hộp bao thế giới (bảo thủ) của hộp cục bộ bb sau phép đặt M (3x4)."""
    C = np.array(list(itertools.product(*bb.T))); P = C @ M[:, :3].T + M[:, 3]
    return np.array([P.min(0), P.max(0)])

def _hit_bb(a, c):
    return not ((a[0] > c[1]).any() or (a[1] < c[0]).any())

def init():
    U = A.upstream()["p"]; S = A.step_to_fore()
    w = {n: S * s for n, s in U.items()}
    w["OB_F_J7_Gripper"] = S * (Pos(0, -A.GRIP_SLIDE, A.GRIP_UP) * U["OB_F_J7_Gripper"]); w["OB_F_J7_Jaw"] = S * (Pos(A.JAW_X, -A.GRIP_SLIDE, A.JAW_UP) * U["OB_F_J7_Jaw"])
    w["M1-AR-025_yaw_lock"] = S * A.yaw_lock_025(); w["M1-CM-001_wrist_cam"] = S * A.cam_wrist()
    w["M1-AR-009_wrist_cam_bracket"] = S * A.cam_bracket_009()      # v0.4.5 (DK-W4)
    G["wrist"] = w
    G["roll_parts"] = ("OB_F_J7_Gripper", "OB_F_J7_Jaw", "M1-CM-001_wrist_cam", "M1-AR-009_wrist_cam_bracket")
    R = A.wrist_ref(); r = R["fore"](R["roll"]); G["roll_yz"] = (float(r[1]), float(r[2]))
    G["flex"] = A.flex_axis()
    ea = b.elbow_assembly()
    G["fore"] = {"cang_tay": A.forearm_021(), "ngam_026": A.seat_026(), "servo_gap": A.flex_servo_box(), "ga_khuyu": ea["e_cradle"], "servo_khuyu_t": ea["e_servoL"], "servo_khuyu_p": ea["e_servoR"]}
    pad, br, _ = L.shapes(45.0, 50.0); xt = g2.find_top_xf()
    G["obs"] = {"chan_duoi": br, "dem_duoi": pad, "thanh_tren": g2.top_bar(xt), "dem_tren": g2.top_pad(xt), "gia_vai": f.cradle_v4()}
    tab, rp = g2.elbow_rev()
    G["link_parts0"] = {"tab_023": tab, **{f"dem_023{chr(97 + i)}": r for i, r in enumerate(rp)}, "dem_022": g3.pad_pu(), "de_024": g3.pad_base()}
    G["cache"] = {}
    G["wbb"] = {n: _bb(p) for n, p in w.items()}
    G["fore_bb"] = {k: _bb(v) for k, v in G["fore"].items()}

def obstacles(qs):
    c = G["cache"]
    if qs not in c:
        o = dict(G["obs"])
        o["link"] = b.link_v2(qs); o["chot"] = b.pin(qs); o["lo_xo_t"] = b.spring(qs, -1); o["lo_xo_p"] = b.spring(qs, 1)
        for k, s in G["link_parts0"].items(): o[k] = Rot(Y=qs) * s
        c.clear(); c[qs] = (o, {k: _bb(v) for k, v in o.items()})
    return c[qs]

def one(t):
    qs, qe, qw, roll = t
    obs, obb = obstacles(qs) if qs is not None else (G["fore"], G["fore_bb"])     # qs None: tự va chạm (khung cẳng tay)
    fx, fz = G["flex"]; ry, rz = G["roll_yz"]
    Fm = Rot(Y=qs or 0) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe or 0) if qs is not None else Pos(0, 0, 0)
    W = Pos(fx, 0, fz) * Rot(Y=qw) * Pos(-fx, 0, -fz)
    Rr = Pos(0, ry, rz) * Rot(X=roll) * Pos(0, -ry, -rz)
    hit = {}
    for n, p in G["wrist"].items():
        L_ = Fm * W * Rr if (roll and n in G["roll_parts"]) else Fm * W
        wb = _bbw(G["wbb"][n], _mat(L_)); q = None
        for on, o in obs.items():
            if not _hit_bb(wb, obb[on]):
                continue
            if q is None: q = L_ * p
            v = vol(q, o)
            if v: hit[f"{n}|{on}"] = round(v, 1)
    return [qs, qe, qw, roll, hit]

ROLLS = (-160.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 160.0)   # spec wrist_roll ±160; ít nhất -160, -90, -45, 0, 45, 90, 160 (a) + ±135
COARSE = (-77.5, -70.0, -60.0, -50.0, -40.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 87.5, 90.5)
WF_MAX = 87.5      # giới hạn gập đề xuất (tự va chạm, mục 1 bao-cao-cad v0.4.4); sạch tới 90,5 (= 87,5 + 3) thì ô lấy trần 87,5

def run(pool, tasks, self_hit, out, t0, tag):
    selfk = sorted({(qw, r) for _, _, qw, r in tasks} - set(self_hit))
    for (_, _, qw, r, h) in pool.map(one, [(None, None, qw, r) for qw, r in selfk]):
        self_hit[(qw, r)] = h
    for i, r in enumerate(pool.imap(one, tasks, chunksize=4)):
        r[4].update(self_hit[(r[2], r[3])]); out.append(r)
        if i % 200 == 0:
            print(tag, i, len(tasks), round(time.time() - t0), flush=True)

def sweep_main(dst, margin):
    from faibotic import spec
    S, E = spec.WRIST_COUPLE_SHOULDER_DEG, spec.WRIST_COUPLE_ELBOW_DEG
    t0 = time.time(); rows = []; self_hit = {}
    with Pool(int(os.environ.get("M1_PROCS", "2")), initializer=init) as p:
        run(p, [(qs, qe, qw, r) for qs in S for qe in E for r in ROLLS for qw in COARSE], self_hit, rows, t0, "tho")
        H = {(qs, qe, qw, r): h for qs, qe, qw, r, h in rows}
        fine = []
        for qs in S:
            for qe in E:
                for r in ROLLS:
                    up = [w for w in COARSE if w >= 0]
                    hit = next((w for w in up if H[(qs, qe, w, r)]), None)
                    if hit is not None and hit > 0:
                        lo = max(w for w in up if w < hit)
                        fine += [(qs, qe, w, r) for w in np.arange(lo + 2.5, hit - 1e-6, 2.5).tolist()]
        run(p, fine, self_hit, rows, t0, "tinh")
    H = {(qs, qe, qw, r): h for qs, qe, qw, r, h in rows}
    cell = {}; table = []
    for qs in S:
        row = []
        for qe in E:
            per = {}
            for r in ROLLS:
                ws = sorted(w for (a, b_, w, rr) in H if a == qs and b_ == qe and rr == r)
                clean = None; first = None
                for w in [w for w in ws if w >= 0]:
                    if H[(qs, qe, w, r)]:
                        first = w; break
                    clean = w
                neg = next((w for w in sorted([w for w in ws if w < 0], reverse=True) if H[(qs, qe, w, r)]), None)
                per[r] = dict(sach_lon_nhat=clean, cham_dau=first, vat=(H[(qs, qe, first, r)] if first is not None else {}), cham_am=neg)
            worst = min(ROLLS, key=lambda r: (1e9 if per[r]["cham_dau"] is None else (per[r]["sach_lon_nhat"] if per[r]["sach_lon_nhat"] is not None else -999.0)))
            mc = min((COARSE[-1] if per[r]["cham_dau"] is None else (per[r]["sach_lon_nhat"] if per[r]["sach_lon_nhat"] is not None else -999.0)) for r in ROLLS)
            val = WF_MAX if mc >= WF_MAX + margin else mc - margin
            row.append(round(val, 1))
            cell[f"{qs}|{qe}"] = dict(bang=round(val, 1), sach_nho_nhat_qua_cuon=mc, cuon_xau_nhat=worst, theo_cuon={str(r): per[r] for r in ROLLS})
        table.append(row)
    json.dump(dict(hinh="v0.4.4 (assembly_full.py), quét theo cuộn", cuon=ROLLS, co_tay_tho=COARSE, bien_deg=margin, tran=WF_MAX,
                   WRIST_COUPLE_SHOULDER_DEG=S, WRIST_COUPLE_ELBOW_DEG=E, WRIST_COUPLE_MAX_DEG=table, o=cell,
                   tu_va_cham=[[qw, r, h] for (qw, r), h in sorted(self_hit.items()) if h], rows=rows, giay=round(time.time() - t0)), open(dst, "w"))
    print("done", round(time.time() - t0), json.dumps(table))

if __name__ == "__main__":
    if sys.argv[1] == "--sweep":
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../sdk"))
        sweep_main(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 3.0); sys.exit()
    grid = json.load(open(sys.argv[1]))
    if "cells" in grid:
        tasks = [(qs, qe, qw, r) for qs, qe, ws in grid["cells"] for qw in ws for r in grid.get("rolls", [0])]
    else:
        tasks = [(qs, qe, qw, r) for qs in grid["shoulders"] for qe in grid["elbows"] for qw in grid["wrists"] for r in grid.get("rolls", [0])]
    selfk = sorted({(qw, r) for _, _, qw, r in tasks})
    t0 = time.time(); out = []
    with Pool(int(os.environ.get("M1_PROCS", "2")), initializer=init) as p:
        self_hit = {(qw, r): h for _, _, qw, r, h in p.map(one, [(None, None, qw, r) for qw, r in selfk])}
        print("tu va cham xong", round(time.time() - t0), flush=True)
        for i, r in enumerate(p.imap(one, tasks, chunksize=2)):
            r[4].update(self_hit[(r[2], r[3])]); out.append(r)
            if i % 50 == 0:
                print(i, len(tasks), round(time.time() - t0), flush=True)
    json.dump({"hinh": "v0.4.4 (assembly_full.py)", "tu_va_cham": [[qw, r, self_hit[(qw, r)]] for qw, r in selfk], "rows": out}, open(sys.argv[2], "w"))
    print("done", round(time.time() - t0), "hits", sum(1 for r in out if r[4]), "of", len(out))
