# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""(24/11/2026): kẹp gốc AM-ARM200 làm kẹp mặc định của M1 P-L. Thân kẹp OB_F_J7_Gripper (M1-AR-007), hàm động OB_F_J7_Jaw (M1-AR-008),
giá camera cổ tay OB_F_Camera (M1-AR-027): STEP follower upstream (commit 5b03af1, Apache-2.0), đặt vào khung URDF bằng am_j7_register.py
(cad/export/AM_J7_registration.json). Servo kẹp STS3215 theo khớp left_gripper URDF (như assembly_full.gripper_servo_pose, không dời).
Camera H65V1 (M1-CM-001) đặt theo khung left_camera URDF [vị trí đề xuất].

Khung tay (hand, mm): gốc khớp cuộn (gốc link left_Fixed_Jaw), x dọc trục cuộn ra ngón, y trái, z lên ở tư thế 0 SDK, cuộn 0, tay trái
(trùng body <bên>_hand của sim/m1.xml). Góc kẹp: góc URDF left_gripper (rad); SDK/sim dùng góc mở tính từ lúc khép (GRIP_CLOSE_URDF).

Chạy (venv build123d): python gripper_am.py --scan -> ../export/AM_gripper_PL.json (góc khép/mở, khe đầu ngón, TCP, hộp va chạm, khối lượng, quét)
                        python gripper_am.py --roll-scan -> ../export/AM_gripper_roll_scan.json (giá camera chạm đáy J56 khi cuộn -147..-87 độ)
                        python gripper_am.py --yaw-scan [--procs 3] -> ../export/AM_wrist_yaw_scan.json (DK-N66: yaw x gập x cuộn x kẹp)"""
import json, math, os, sys, argparse
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(REPO, "sdk")); sys.path.insert(0, os.path.join(REPO, "sim", "tools"))
import amarm as AM

EXP = os.path.join(HERE, "..", "export")
REG = os.path.join(EXP, "AM_J7_registration.json")
OUT = os.path.join(EXP, "AM_gripper_PL.json")
PARTS = {   # STEP -> (mã, vật liệu, link URDF)
    "OB_F_J7_Gripper": ("M1-AR-007", "ivory", "left_Fixed_Jaw"), "OB_F_Camera": ("M1-AR-027", "navy", "left_Fixed_Jaw"),
    "OB_F_J7_Jaw": ("M1-AR-008", "navy", "left_Moving_Jaw")}
FINGER_X0 = 70.0       # mm khung tay: phần ngón (đầu hàm) bắt đầu từ đây; gần trục kẹp là gốc hàm tì thân kẹp ở mọi góc (mô hình upstream)
HINGE_R = 22.0         # mm: vùng gốc hàm quanh trục kẹp, bỏ khi tìm giới hạn mở (gốc hàm và thân kẹp chồng nhau sẵn trong CAD upstream)
TIP_LEN = 10.0         # mm: tâm kẹp lùi khỏi mút ngón dọc trục cuộn (như spec.TIP_LEN)
OPEN_SOFT_DEG = 64.0   # giới hạn mở mềm (độ từ chạm khép) = tư thế 0 URDF upstream (hàm mở 64,6 độ, đầu ngón cách 98 mm) [giả định].
                       # Quét CAD: ngoài vùng gốc hàm, hàm không chạm thân kẹp, giá camera, servo, camera tới 180 độ, nên không có chặn cứng để lấy;
                       # tư thế 0 URDF là tư thế mở rộng nhất upstream có công bố. Chạm khép đo bằng CAD (scan).


def _reg():
    return json.load(open(REG))["chi_tiet"]


def _loc(M):
    from build123d import Location, Plane
    return Location(Plane(origin=tuple(M[:3, 3]), x_dir=tuple(M[:3, 0]), z_dir=tuple(M[:3, 2])))


def M4(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M


_S = {}
def step_parts_base():
    """{tên STEP: (link, shape)} trong khung left_Base URDF (mm) ở tư thế 0 URDF (góc kẹp 0)."""
    if not _S:
        from build123d import import_step
        R = _reg()
        for n, (_, _, link) in PARTS.items():
            _S[n] = (link, _loc(np.array(R[n]["M"])) * import_step(os.path.join(AM.STP, n + ".stp")))
    return _S


def base_to_hand():
    """4x4 (mm): khung left_Base URDF ở tư thế 0 URDF -> khung tay (hand) ở tư thế 0 SDK, cho điểm gắn link left_Fixed_Jaw / left_Moving_Jaw (góc kẹp 0).
    Cùng phép với export_sim_meshes.pl_urdf_to_fore (vai -162,25, khuỷu +162,25 độ URDF) trừ gốc body hand."""
    import am_arm200_pl as U
    from faibotic import spec
    a = math.radians(spec.UPPER_URDF_DEG); T0 = U.fk({}); T1 = U.fk({"left_shoulder_lift": -a, "left_elbow_flex": a})
    el1 = U.R_SDK @ T1["left_Lower_Arm"][1] * 1000
    hand = np.array([spec.L2 + spec.LW, spec.ROLL_DY, spec.WF_DZ + spec.ROLL_DZ]) * 1000
    R0, p0 = T0["left_Fixed_Jaw"]; R1, p1 = T1["left_Fixed_Jaw"]
    R = U.R_SDK @ R1 @ R0.T
    t = U.R_SDK @ p1 * 1000 - R @ (p0 * 1000) - el1 - hand
    return M4(R, t)


def jaw_axis_base():
    """(gốc, trục đơn vị) khớp left_gripper trong khung left_Base, mm, tư thế 0 URDF."""
    T = AM.fk({})["left_Moving_Jaw"]; return T[:3, 3] * 1000, T[:3, 2]


def grip_servo_pose_base():
    """Servo kẹp STS3215: trục ra theo khớp left_gripper, mặt tựa còi 16,0 mm dọc trục từ gốc khớp (URDF), thân hướng về cổ tay (như assembly_full)."""
    import assembly_full as A, servos as SV
    R = A.wrist_ref(); E, t = R["elbow"], R["t"]
    fore = np.array(A.GRIP_SV, float)                                    # khung cẳng tay của assembly_full (URDF, không dời GRIP_SV_D)
    P_step = np.array([fore[1] + E[0], E[1] - fore[0], fore[2] + E[2]])
    return SV.pose("sts3215", P_step - t, (0, 0, 1), (1, 0, 0))


def cam_module_base():
    """Camera H65V1 (M1-CM-001, bo 36 x 36 + ống kính) tại khung left_camera URDF, trục quang +z khung đó [vị trí đề xuất]."""
    import assembly_full as A
    T = AM.fk({})["left_camera"]; M = T.copy(); M[:3, 3] *= 1000
    return _loc(M) * A.camera_module()


def parts_base(grip_urdf=None):
    """List (tên, vật liệu, link, shape) khung left_Base ở tư thế 0 URDF; hàm động quay tới grip_urdf (rad URDF; None = 0)."""
    from build123d import Axis
    out = []
    for n, (link, s) in step_parts_base().items():
        code, mat, _ = PARTS[n]
        if link == "left_Moving_Jaw" and grip_urdf:
            o, a = jaw_axis_base(); s = s.rotate(Axis(tuple(o), tuple(a)), math.degrees(grip_urdf))
        out.append((f"{code}_{n}", mat, link, s))
    out.append(("M1-CM-001_wrist_cam", "servo", "left_Fixed_Jaw", cam_module_base()))
    return out


def _pts(s, tol=0.1):
    v, _ = s.tessellate(tol, 0.2); return np.array([[p.X, p.Y, p.Z] for p in v])


def _vol(a, b):
    try:
        r = a & b; return 0.0 if r is None else float(r.volume)
    except Exception:
        return float("nan")


def scan(step=2.0):
    """Quét góc kẹp trong khung tay: chiều khép, góc chạm khép, giới hạn mở, TCP, khe đầu ngón, hộp va chạm, khối lượng."""
    from build123d import Axis, Box, Pos, Align, Cylinder, Location, Plane
    import servos as SV
    H = base_to_hand(); L = _loc(H)
    P = {n: (link, L * s) for n, (link, s) in step_parts_base().items()}
    body, cam_mount, jaw0 = P["OB_F_J7_Gripper"][1], P["OB_F_Camera"][1], P["OB_F_J7_Jaw"][1]
    o, a = jaw_axis_base(); o = H[:3, :3] @ o + H[:3, 3]; a = H[:3, :3] @ a
    ax = Axis(tuple(o), tuple(a))
    servo = L * SV.solid("sts3215", grip_servo_pose_base())
    cam = L * cam_module_base()
    big = 400.0
    fin_box = Pos(FINGER_X0, -big / 2, -big / 2) * Box(big, big, big, align=(Align.MIN,) * 3)
    BF = body & fin_box
    # vùng gốc hàm: trụ bán kính HINGE_R quanh trục kẹp
    hinge = Location(Plane(origin=tuple(o - a * 60), z_dir=tuple(a))) * Cylinder(HINGE_R, 120, align=(Align.CENTER, Align.CENTER, Align.MIN))
    jaw = lambda d: jaw0.rotate(ax, d)
    tip = lambda Q: (Q[Q[:, 0] > Q[:, 0].max() - 3.0].mean(0), Q[:, 0].max())
    # chiều khép: đầu hàm động tiến về ngón cố định
    Pb = _pts(BF, 0.2); tb, xb = tip(Pb)
    d0 = np.linalg.norm(tip(_pts(jaw(0.0), 0.2))[0][1:] - tb[1:]); d5 = np.linalg.norm(tip(_pts(jaw(5.0), 0.2))[0][1:] - tb[1:])
    sgn = 1.0 if d5 < d0 else -1.0          # +sgn độ = khép
    rows = []
    def close_iv(d): return _vol(BF, jaw(sgn * d) & fin_box)
    lo, hi = None, None
    for d in np.arange(-60.0, 120.0, step):
        if close_iv(d) > 0.05: hi = d; break
        lo = d
    for _ in range(14):
        m = (lo + hi) / 2; lo, hi = (lo, m) if close_iv(m) > 0.05 else (m, hi)
    th_close = sgn * lo                                     # độ URDF tại chạm khép
    # mở: hàm (ngoài vùng gốc) với thân kẹp, giá camera, servo kẹp, camera
    obst = [("than_kep", body - hinge), ("gia_camera", cam_mount), ("servo_kep", servo), ("camera", cam)]
    def open_hits(d):
        J = jaw(th_close - sgn * d) - hinge
        return {k: round(_vol(J, s), 2) for k, s in obst}
    open_lim, scan_open = None, []
    for d in np.arange(step, 181.0, step):
        h = open_hits(d); scan_open.append(dict(mo_deg=float(d), giao_mm3=h))
        if any(v > 0.05 for v in h.values()): open_lim = float(d); break
    scan_open = [r for r in scan_open if r["mo_deg"] % 10 < 1e-6 or any(v > 0.05 for v in r["giao_mm3"].values())]
    if open_lim is None: open_lim = 180.0
    lo2, hi2 = open_lim - step, open_lim
    for _ in range(10):
        m = (lo2 + hi2) / 2; lo2, hi2 = (lo2, m) if any(v > 0.05 for v in open_hits(m).values()) else (m, hi2)
    open_contact = lo2
    open_soft = min(OPEN_SOFT_DEG, open_contact - 3.0)
    # đầu ngón, khe, TCP: đầu hàm là điểm cố định trên hàm (lấy lúc khép), quay theo góc mở
    print("khep", th_close, "cham mo", open_contact, flush=True)
    Q = _pts(jaw(th_close) & fin_box); tj_c, xj_c = tip(Q)
    def rot(p, deg):
        k = a / np.linalg.norm(a); r = math.radians(deg); v = p - o
        return o + v * math.cos(r) + np.cross(k, v) * math.sin(r) + k * np.dot(k, v) * (1 - math.cos(r))
    tj_o = rot(tj_c, -sgn * open_soft); tj_u = rot(tj_c, -th_close)
    tcp = (tb + tj_c) / 2; tcp[0] = (xb + xj_c) / 2
    grasp = tcp - np.array([TIP_LEN, 0, 0])
    gap = lambda tj: float(np.linalg.norm(tj - tb))
    # hộp va chạm (khung tay): ngón cố định = phần thân kẹp x >= FINGER_X0; hàm động khép (khung tay) phần x >= FINGER_X0
    def bb(s):
        b = s.bounding_box(); return [round(v, 2) for v in (b.min.X, b.min.Y, b.min.Z, b.max.X, b.max.Y, b.max.Z)]
    jc = jaw(th_close)
    # hộp va chạm ngón hàm động, xoay theo hướng hàm (gốc trục kẹp -> đầu hàm) quanh z khung tay; phần ngón = u >= 0,55 u_max
    d2 = (tj_c - o)[:2]; beta = math.atan2(d2[1], d2[0]); dvec = np.array([math.cos(beta), math.sin(beta), 0.0]); nvec = np.array([-dvec[1], dvec[0], 0.0])
    PJ = _pts(jc, 0.1) - o; u, w, zz = PJ @ dvec, PJ @ nvec, PJ[:, 2]; keep = u >= 0.55 * u.max()
    jb = dict(u=[float(u[keep].min()), float(u.max())], w=[float(w[keep].min()), float(w[keep].max())], z=[float(zz[keep].min()), float(zz[keep].max())])
    fb = BF.bounding_box()
    # mặt trong hộp hàm (w max) không được lấn mặt trong hộp ngón cố định (y min) khi khép: đo góc trong của hộp hàm trong dải x của ngón cố định
    def inner_y(wmax):
        ys = []
        for uu in (jb["u"][0], jb["u"][1]):
            pnt = o + uu * dvec + wmax * nvec
            if fb.min.X - 1 <= pnt[0] <= fb.max.X + 1: ys.append(pnt[1])
        return max(ys) if ys else -1e9
    wmax = jb["w"][1]; pen = inner_y(wmax) - fb.min.Y
    if pen > -0.3: wmax -= pen + 0.3
    jb["w"][1] = wmax; jb["beta_deg"] = round(math.degrees(beta), 3); jb["cat_mat_trong_mm"] = round(max(pen + 0.3, 0.0), 3)
    jb = {k: ([round(x, 3) for x in v] if isinstance(v, list) else v) for k, v in jb.items()}
    palm = body - fin_box
    vol = {n: round(float(s.volume) / 1000, 3) for n, s in (("than_kep", body), ("gia_camera", cam_mount), ("ham", jaw0))}
    out = dict(
        ghi_chu="khung tay mm (gốc khớp cuộn, x dọc trục cuộn, y trái, z lên; tư thế 0 SDK, cuộn 0, tay trái). Góc URDF left_gripper; mở = góc tính từ chạm khép.",
        chieu_khep_urdf=sgn, goc_khep_urdf_deg=round(th_close, 3), mo_cham_deg=round(open_contact, 2), mo_mem_deg=open_soft,
        truc_kep_goc=np.round(o, 3).tolist(), truc_kep_huong=np.round(a, 5).tolist(),
        dau_ngon_co_dinh=np.round(tb, 2).tolist(), mut_ngon_co_dinh_x=round(float(xb), 2), dau_ham_khep=np.round(tj_c, 2).tolist(),
        khe_dau_ngon_khep_mm=round(gap(tj_c), 2), khe_dau_ngon_mo_mem_mm=round(gap(tj_o), 2), khe_dau_ngon_urdf0_mm=round(gap(tj_u), 2),
        tcp_khep=np.round(tcp, 2).tolist(), tam_kep_lui_tip=np.round(grasp, 2).tolist(), tip_len_mm=TIP_LEN,
        hop_bao=dict(than_kep=bb(body), than_kep_tru_ngon=bb(palm), ngon_co_dinh=bb(BF), ham_khep=bb(jc), ham_khep_ngon=bb(jc & fin_box),
                     gia_camera=bb(cam_mount), servo_kep=bb(servo), camera=bb(cam)),
        hop_ham_xoay=jb, the_tich_cm3=vol, servo_pose_tay=(H @ grip_servo_pose_base()).round(3).tolist(),
        cam_khung_tay=(H @ _cam_M()).round(4).tolist(), quet_mo=scan_open, finger_x0_mm=FINGER_X0, hinge_r_mm=HINGE_R)
    return out


def roll_scan(degs=range(-160, 161, 10), grips=(0.0, 100.0)):
    """Kẹp (thân, giá camera, camera, hàm ở độ mở %) theo góc cuộn URDF với cụm cổ tay (giá J5, chốt, vỏ/đáy/nắp J56, khoá yaw). Giao mm3 > 0,5."""
    import assembly_pl as P, assembly_full as A
    from build123d import Pos
    wrist = {f: s for f, (link, s) in P.arm_parts_local().items() if link in ("left_Wrist_Pitch_Roll", "left_wrist_yaw")}
    wrist["M1-AR-025_yaw_lock"] = Pos(*(-P._t_step())) * A.yaw_lock_025()
    T0 = AM.fk({}); out = {}
    for g in grips:
        parts = {n: (link, s) for n, m, link, s in parts_base(P.grip_urdf(g))}
        for d in degs:
            Tq = AM.fk({"roll": math.radians(d), "grip": P.grip_urdf(g)}); hits = {}
            for n, (link, s) in parts.items():
                D = Tq[link] @ np.linalg.inv(T0[link]); D[:3, 3] *= 1000; sh = _loc(D) * s
                for w, ws in wrist.items():
                    v = _vol(sh, ws)
                    if v > 0.5 or v != v: hits[f"{n.split('_', 1)[0]}x{w}"] = round(v, 1)
            if hits: out[f"g{int(g)}_r{d}"] = hits
    return out


YAW_MOVING_LINKS = ("left_wrist_yaw", "left_Fixed_Jaw", "left_Moving_Jaw")      # đi theo khớp yaw
YAW_FIXED_LINKS = ("left_Wrist_Pitch_Roll", "left_Lower_Arm")                   # giá J5 + chốt (theo gập), cẳng tay J4


def _yaw_sets(grips):
    """(moving {grip: {tên: (link, shape)}}, fixed {tên: (link, shape)}) khung left_Base mm, tư thế 0 URDF."""
    import assembly_pl as P, servos as SV
    from build123d import Pos
    arm = P.arm_parts_local()
    fixed = {f: (l, s) for f, (l, s) in arm.items() if l in YAW_FIXED_LINKS}
    base = {f: (l, s) for f, (l, s) in arm.items() if l == "left_wrist_yaw"}
    Ty = np.array(AM.fk({})["left_wrist_yaw"], float).copy(); Ty[:3, 3] *= 1000
    base["servo_yaw"] = ("left_wrist_yaw", SV.solid("sts3215", Ty @ P.yaw_servo_pose_link()))
    base["servo_kep"] = ("left_Fixed_Jaw", SV.solid("sts3215", grip_servo_pose_base()))
    mov = {}
    for g in grips:
        m = dict(base)
        for n, _, link, s in parts_base(P.grip_urdf(g)):
            m[n] = (link, s)
        mov[g] = m
    return mov, fixed


_YS = {}


def _bbo(a, b, tol=0.0):
    return not (a.max.X < b.min.X - tol or b.max.X < a.min.X - tol or a.max.Y < b.min.Y - tol or b.max.Y < a.min.Y - tol
                or a.max.Z < b.min.Z - tol or b.max.Z < a.min.Z - tol)


def _ys_init():
    if not _YS:
        _YS["mov"], _YS["fixed"] = _yaw_sets((0.0, 100.0)); _YS["T0"] = AM.fk({})


def yaw_pose_hits(args):
    """Giao mm3 (cặp chi tiết đi theo yaw x giá J5/chốt/cẳng tay) ở một tư thế (gập, yaw, cuộn: độ URDF; kẹp %)."""
    wf, yw, rl, g = args
    _ys_init()
    mov, fixed = _YS["mov"][g], _YS["fixed"]
    T0 = _YS["T0"]; Tq = AM.fk({"wflex": math.radians(wf), "yaw": math.radians(yw), "roll": math.radians(rl), "grip": __import__("assembly_pl").grip_urdf(g)})
    def place(link, s):
        D = Tq[link] @ np.linalg.inv(T0[link]); D[:3, 3] *= 1000; return _loc(D) * s
    F = {n: place(l, s) for n, (l, s) in fixed.items()}
    Fb = {n: s.bounding_box() for n, s in F.items()}
    hits = {}
    for n, (l, s) in mov.items():
        sh = place(l, s); b = sh.bounding_box()
        for w, ws in F.items():
            if not _bbo(b, Fb[w]): continue
            v = _vol(sh, ws)
            if v > 0.05 or v != v: hits[f"{n}x{w}"] = round(v, 1)
    return (wf, yw, rl, g), hits


def yaw_scan(wflex_deg, yaw_deg, roll_deg, grips=(0.0,), procs=3, base=None):
    """Quét yaw (độ URDF) x gập x cuộn x kẹp. Trả {khoá tư thế: {cặp: mm3 vượt nền}}; nền = giao ở yaw 0 cùng gập/cuộn/kẹp (cặp lắp sẵn, chốt J5)."""
    import multiprocessing as mp
    poses = [(wf, yw, rl, g) for g in grips for wf in wflex_deg for rl in roll_deg for yw in sorted(set(list(yaw_deg) + [0.0]))]
    with mp.get_context("spawn").Pool(procs, initializer=_ys_init) as pool:      # spawn: fork sau khi nạp OCC treo worker
        R = dict(pool.map(yaw_pose_hits, poses, chunksize=8))
    out = {}
    for (wf, yw, rl, g), h in R.items():
        b0 = R[(wf, 0.0, rl, g)]
        ex = {k: round(v - b0.get(k, 0.0), 1) for k, v in h.items() if v != v or v - b0.get(k, 0.0) > 0.5}
        if ex: out[f"g{int(g)}_f{wf:g}_r{rl:g}_y{yw:g}"] = ex
    nen = {f"g{int(g)}_f{wf:g}_r{rl:g}": R[(wf, 0.0, rl, g)] for g in grips for wf in wflex_deg for rl in roll_deg if R[(wf, 0.0, rl, g)]}
    return out, nen


def _cam_M():
    T = AM.fk({})["left_camera"]; M = T.copy(); M[:3, 3] *= 1000; return M


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--scan", action="store_true"); ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--roll-scan", action="store_true", help="quét cuộn cổ tay -> ../export/AM_gripper_roll_scan.json")
    ap.add_argument("--yaw-scan", action="store_true", help="DK-N66: quét yaw cổ tay x gập x cuộn -> ../export/AM_wrist_yaw_scan.json")
    ap.add_argument("--procs", type=int, default=3); a = ap.parse_args()
    if a.yaw_scan:
        import time; t0 = time.time()
        _ys_init()
        nen0 = yaw_pose_hits((0.0, 0.0, 0.0, 0.0))[1]
        def extra(wf, yw, rl=0.0, g=0.0):
            b = yaw_pose_hits((wf, 0.0, rl, g))[1]; h = yaw_pose_hits((wf, yw, rl, g))[1]
            return {k: round(v - b.get(k, 0.0), 1) for k, v in h.items() if v != v or v - b.get(k, 0.0) > 0.5}
        # (1) chạm đầu tiên theo yaw ở gập 0, cuộn 0, kẹp khép (vỏ J56 với giá J5 chỉ phụ thuộc yaw): thô 5 độ rồi chia đôi tới 0,1 độ
        cham, quet_tho = {}, {}
        for sgn in (1, -1):
            lo = 0.0
            for y in np.arange(5.0, 180.1, 5.0):
                h = extra(0.0, sgn * y); quet_tho[f"y{sgn * y:g}"] = h
                if h: hi = y; break
                lo = y
            for _ in range(6):
                m = (lo + hi) / 2; lo, hi = (lo, m) if extra(0.0, sgn * m) else (m, hi)
            cham["duong" if sgn > 0 else "am"] = dict(urdf_deg=round(sgn * hi, 2), cap=extra(0.0, sgn * (hi + 0.5)))
        BIEN = 3.0
        lim_urdf = (-math.floor(abs(cham["am"]["urdf_deg"]) - BIEN), math.floor(cham["duong"]["urdf_deg"] - BIEN))
        # (2) kiểm lưới trong tầm đã chọn: gập x cuộn x kẹp x yaw; vượt nền yaw 0 phải = 0
        WF = (-77.5, -60.0, -40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 87.5); RL = (-160.0, -120.0, -80.0, -40.0, 0.0, 40.0, 80.0, 120.0, 160.0)
        YY = sorted(set([float(lim_urdf[0]), float(lim_urdf[1])] + [float(v) for v in np.arange(-80.0, 80.1, 20.0) if lim_urdf[0] < v < lim_urdf[1]]))
        luoi, nen = yaw_scan(WF, YY, RL, (0.0, 100.0), procs=a.procs)
        # (3) gập lớn nhất còn sạch ở yaw 0 (cẳng tay J4 với đáy J56), độ URDF = SDK
        lo, hi = 0.0, None
        for f in np.arange(60.0, 100.1, 2.5):
            if any(k.endswith("J4_Elbow") for k in yaw_pose_hits((f, 0.0, 0.0, 0.0))[1]): hi = f; break
            lo = f
        if hi is not None:
            for _ in range(6):
                m = (lo + hi) / 2; lo, hi = (lo, m) if any(k.endswith("J4_Elbow") for k in yaw_pose_hits((m, 0.0, 0.0, 0.0))[1]) else (m, hi)
        r = dict(ghi_chu=("DK-N66: chi tiết đi theo khớp yaw (vỏ/đáy/nắp J56, servo yaw, thân kẹp J7, hàm, giá camera, camera, servo kẹp) x giá J5 + chốt "
                          "(theo gập) + cẳng tay J4. Góc độ URDF (yaw SDK = -URDF; gập, cuộn URDF = SDK). Giao mm3 vượt nền (cùng gập/cuộn/kẹp ở yaw 0) > 0,5; "
                          "g0/g100 = độ mở kẹp %. STEP follower AM-ARM200 + khối bao servo STS3215 (servos.py) + camera bao giả định."),
                 nen_lap_san=nen0, cham_yaw=cham, quet_tho_5deg=quet_tho, bien_deg=BIEN,
                 gioi_han_urdf_deg=list(lim_urdf), gioi_han_sdk_deg=[-lim_urdf[1], -lim_urdf[0]],
                 luoi=dict(gap_deg=list(WF), cuon_deg=list(RL), kep_pct=[0, 100], yaw_urdf_deg=YY, vuot_nen=luoi),
                 nen_yaw0_theo_tu_the=nen, gap_cham_cang_tay_yaw0_deg=hi, thoi_gian_s=round(time.time() - t0, 1))
        json.dump(r, open(os.path.join(EXP, "AM_wrist_yaw_scan.json"), "w"), ensure_ascii=False, indent=1)
        print(json.dumps({k: v for k, v in r.items() if k not in ("nen_yaw0_theo_tu_the",)}, ensure_ascii=False))
    if a.roll_scan:
        r = dict(ghi_chu="kẹp AM-ARM200 theo góc cuộn URDF (= góc SDK) với cụm cổ tay; giao mm3 > 0,5; g0/g100 = độ mở kẹp %",
                 buoc_10=roll_scan(), mep_buoc_2=roll_scan(list(range(-150, -139, 2)) + list(range(-92, -79, 2)), (0.0,)),
                 vung_cham_deg=[-147, -87], gioi_han_sdk_deg=[-160, 160], ghi_chu_gioi_han="SDK chưa thu hẹp, DK-N54 mở")
        json.dump(r, open(os.path.join(EXP, "AM_gripper_roll_scan.json"), "w"), ensure_ascii=False, indent=1); print(json.dumps(r["mep_buoc_2"]))
    if a.scan:
        r = scan(a.step); json.dump(r, open(OUT, "w"), ensure_ascii=False, indent=1)
        print(json.dumps({k: v for k, v in r.items() if k not in ("quet_mo",)}, ensure_ascii=False))
