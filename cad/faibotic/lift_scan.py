# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Quét va chạm chuỗi tay (vai 140 + cẳng 434 duỗi, bán kính 30 mm [giả định]) với cột, ray, khối, xe, đầu, tay đỡ bên kia (hộp bao).
  lift_scan.py derive   -> tìm khuỷu lớn nhất không chạm theo vai (pan ±70, lift 0 và 0,50 m), ghi ../export/M1-LF-scan.json
  lift_scan.py verify   -> quét với giới hạn trong spec (ARM_LIMITS + spec.elbow_max_by_shoulder), mong 0 chạm
  lift_scan.py derive_mesh [biên mm] [bước cổ tay] -> v0.4.4: thay chuỗi thẳng bán kính 30 bằng đám điểm đỉnh mesh thật của link, gá khuỷu, cẳng tay, cụm cổ tay, kẹp, camera
                           (assembly_full.py, lưới 5 mm), quét thêm góc cổ tay -90..120 (bước mặc định 30); hộp vật cản nới theo biên (mặc định 10 mm); ghi ../export/M1-LF-scan-derive_mesh.json
Chạy: nice -n 15 python lift_scan.py verify. Chưa quét cổ tay/kẹp/cáp (Chưa có số liệu)."""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
import lift as L
from faibotic import spec
R = 30.0; PAN_STEP = 5; Q_STEP = 3.0

def static_boxes(lift, own):
    z_ax = L.Z_AX0 + lift
    car = L.carriage(z_ax)
    st = {"cot": L.column(), "de": L.flange(), "dau": Pos_head(), "goi_tren": L.top_block()}
    for i, b in enumerate(L.blocks(z_ax)): st[f"khoi{i}"] = b
    for i, r in enumerate(L.rails()): st[f"ray{i}"] = r
    for k, v in car.items():
        if k[-1] == own and k[:3] in ("pad", "out"): continue      # giá chính của tay này (link thật hở 4,8 mm: lift.py link_vs_pad)
        st["xe_" + k] = v
    names = list(st)
    return names, np.array([L._aabb(st[k], R) for k in names])

def Pos_head():
    from build123d import Pos, Box
    return Pos(L.CX + 10, 0, spec.COLUMN_TOP * 1000 - 30) * Box(70, 140, 60)

def chain(qs, qe):
    a1 = math.radians(qs); a2 = math.radians(qs + qe)
    t1 = np.arange(0, 140, 10.0); t2 = np.arange(0, 434.1, 10.0)
    p1 = np.array([140 * math.cos(a1), -140 * math.sin(a1)])
    P = np.vstack([np.c_[t1 * math.cos(a1), -t1 * math.sin(a1)], p1 + np.c_[t2 * math.cos(a2), -t2 * math.sin(a2)]])
    return P[P[:, 0] ** 2 + P[:, 1] ** 2 > 3600]

def hit(P, pan, side, own_boxes, z_ax):
    cp, sp = math.cos(math.radians(pan)), math.sin(math.radians(pan))
    X = L.SX + P[:, 0] * cp; Y = side * L.SY + side * P[:, 0] * sp; Z = z_ax + P[:, 1]   # pan dương quay vào trong cho cả hai tay (đối xứng)
    names, B = own_boxes
    ins = ((X[:, None] > B[:, 0]) & (X[:, None] < B[:, 3]) & (Y[:, None] > B[:, 1]) & (Y[:, None] < B[:, 4]) & (Z[:, None] > B[:, 2]) & (Z[:, None] < B[:, 5]))
    return [names[i] for i in np.where(ins.any(0))[0]]

def run(mode):
    pans = range(-70 if mode == "derive" else int(round(math.degrees(spec.ARM_LIMITS["pan"][0]))),
                 (70 if mode == "derive" else int(round(math.degrees(spec.ARM_LIMITS["pan"][1])))) + 1, PAN_STEP)
    nodes = [-80, -64, -48, -32, -16, 0, 16, 24, 32, 40, 48.5]
    res = {"mode": mode, "pan": [min(pans), max(pans)], "R_mm": R, "q_step": Q_STEP}
    worst = {n: 127.0 for n in nodes}; nposes = 0; hits = {}
    for lift in (0.0, L.STROKE):
        for side in (-1, 1):
            own = "L" if side > 0 else "R"
            B = static_boxes(lift, own)
            for pan in pans:
                if mode == "derive":
                    for qs in nodes:
                        for qe in np.arange(-10, 127.01, Q_STEP):
                            nposes += 1
                            if hit(chain(qs, qe), pan, side, B, L.Z_AX0 + lift):
                                worst[qs] = min(worst[qs], float(qe) - Q_STEP); break
                else:
                    for qs in np.arange(-80, 48.51, 4.0):
                        lim = math.degrees(spec.ARM_LIMITS["elbow"][1]); emax = math.degrees(spec.elbow_max(math.radians(qs)))
                        for qe in np.arange(-10, min(lim, emax) + 0.01, Q_STEP):
                            nposes += 1
                            h = hit(chain(qs, qe), pan, side, B, L.Z_AX0 + lift)
                            if h:
                                for k in h: hits[k] = hits.get(k, 0) + 1
    res["so_tu_the"] = nposes
    if mode == "derive":
        res["khuyu_max_theo_vai_deg"] = {str(k): v for k, v in worst.items()}
    else:
        res["so_cham"] = sum(hits.values()); res["theo_chi_tiet"] = hits
    json.dump(res, open(os.path.join(HERE, f"../export/M1-LF-scan-{mode}.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))

def _cloud(shapes, step=5.0):
    """Đỉnh tessellate (dung sai 1 mm) gộp lưới step mm -> mảng N x 3."""
    from export_sim_meshes import tess      # tessellate mịn dần / từng mặt khi solid upstream lỗi
    P = []
    for s in shapes:
        v, _ = tess(s, 1.0, 0.5); P += v.tolist()
        for e in s.edges():
            try:
                for t in np.linspace(0, 1, max(2, int(e.length / step) + 1)):
                    q = e.position_at(t); P.append([q.X, q.Y, q.Z])
            except Exception:
                pass
    P = np.array(P)
    k = np.round(P / step).astype(int)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[idx]

def mesh_groups():
    """Đám điểm theo khâu: link (khung vai, vai 0), cẳng tay (khung khuỷu), cổ tay (khung cẳng tay, gập 0). Lò xo, gá vai không đi theo khâu: bỏ như chuỗi cũ."""
    import assembly_full as A, balance as b, failsafe_v042 as g2, failsafe_v043 as g3
    from build123d import Pos
    tab, rp = g2.elbow_rev()
    link = _cloud([b.link_v2(0.0), b.pin(0.0), tab, *rp, g3.pad_pu(), g3.pad_base()])
    ea = b.elbow_assembly()
    fore = _cloud([ea["e_cradle"], ea["e_servoL"], ea["e_servoR"], A.forearm_021(), A.seat_026(), A.flex_servo_box()])
    U = A.upstream()["p"]; S = A.step_to_fore()
    ws = [S * s for n, s in U.items() if not n.startswith("OB_F_J7")]
    ws += [S * (Pos(0, -A.GRIP_SLIDE, A.GRIP_UP) * U["OB_F_J7_Gripper"]), S * (Pos(A.JAW_X, -A.GRIP_SLIDE, A.JAW_UP) * U["OB_F_J7_Jaw"]), S * A.yaw_lock_025(), S * A.cam_wrist(), S * A.cam_bracket_009()]
    wrist = _cloud(ws)
    return link, fore, wrist, A.flex_axis()

def _ry(P, deg, cx=0.0, cz=0.0):
    a = math.radians(deg); c, s_ = math.cos(a), math.sin(a)
    x, z = P[:, 0] - cx, P[:, 2] - cz
    return np.c_[cx + x * c + z * s_, P[:, 1], cz - x * s_ + z * c]   # như build123d Rot(Y=deg): dương = gập xuống

def arm_points(G, qs, qe, qw):
    link, fore, wrist, (fx, fz) = G
    w = _ry(wrist, qw, fx, fz)
    fe = _ry(np.vstack([fore, w]), qe); fe[:, 0] += 140.0
    P = _ry(np.vstack([link, fe]), qs)
    return P[P[:, 0] ** 2 + P[:, 2] ** 2 > 3600]

def hit3(P, pan, side, own_boxes, z_ax):
    a = math.radians(pan); cp, sp = math.cos(a), math.sin(a)
    x, y = P[:, 0], P[:, 1]          # hình tay trái; tay phải = gương qua y = 0 cùng góc pan (như hit)
    X = L.SX + x * cp - y * sp; Y = side * (L.SY + x * sp + y * cp); Z = z_ax + P[:, 2]
    names, B = own_boxes
    ins = ((X[:, None] > B[:, 0]) & (X[:, None] < B[:, 3]) & (Y[:, None] > B[:, 1]) & (Y[:, None] < B[:, 4]) & (Z[:, None] > B[:, 2]) & (Z[:, None] < B[:, 5]))
    return [names[i] for i in np.where(ins.any(0))[0]]

def run_mesh(margin=10.0, wstep=30):
    import time, assembly_full as A
    global R
    R = margin; t0 = time.time()
    G = mesh_groups(); print("diem", [len(g) for g in G[:3]], round(time.time() - t0), flush=True)
    nodes = [-80, -64, -48, -32, -16, 0, 8, 16, 24, 32, 36, 40, 44, 48.5]; wrists = list(range(-90, 121, wstep))
    res = {"mode": "derive_mesh", "bien_mm": margin, "pan": [-70, 70], "q_step": Q_STEP, "co_tay_deg": wrists, "luoi_diem_mm": 5.0,
           "so_diem": dict(link=len(G[0]), cang_tay=len(G[1]), co_tay=len(G[2]))}
    worst = {w: {n: 127.0 for n in nodes} for w in wrists}; who = {}
    for lift in (0.0, L.STROKE):
        for side in (-1, 1):
            own = "L" if side > 0 else "R"
            names, B = static_boxes(lift, own)
            for n_, _, s_ in A.cam_head(): names.append(n_); B = np.vstack([B, L._aabb(s_, R)])
            for pan in range(-70, 71, PAN_STEP):
                for w in wrists:
                    for qs in nodes:
                        for qe in np.arange(-10, 127.01, Q_STEP):
                            if qe >= worst[w][qs]: break
                            h = hit3(arm_points(G, qs, qe, w), pan, side, (names, B), L.Z_AX0 + lift)
                            if h:
                                worst[w][qs] = float(qe) - Q_STEP; who[f"{w}|{qs}"] = dict(cham=h, pan=pan, lift=lift, khuyu=float(qe)); break
        print("lift", lift, round(time.time() - t0), flush=True)
    res["khuyu_max_theo_vai_deg"] = {str(w): {str(k): v for k, v in worst[w].items()} for w in wrists}
    res["khuyu_max_moi_co_tay"] = {str(k): min(worst[w][k] for w in wrists) for k in nodes}
    res["cham_dau_tien"] = who; res["giay"] = round(time.time() - t0)
    json.dump(res, open(os.path.join(HERE, "../export/M1-LF-scan-derive_mesh.json"), "w"), indent=1, ensure_ascii=False)
    print(json.dumps(res["khuyu_max_moi_co_tay"]))

if __name__ == "__main__":
    m = sys.argv[1] if len(sys.argv) > 1 else "verify"
    if m == "derive_mesh":
        run_mesh(float(sys.argv[2]) if len(sys.argv) > 2 else 10.0, int(sys.argv[3]) if len(sys.argv) > 3 else 30)
    else:
        run(m)
