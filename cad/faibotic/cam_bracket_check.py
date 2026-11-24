# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""DK-W4 (22/11/2026): kiểm giá camera cổ tay M1-AR-009 + vị trí camera mới (assembly_full.cam_wrist, cam_bracket_009). build123d.
(1) ràng buộc đặt: camera + giá trước mặt đầu vỏ J56 (x >= 289,5), trên mặt hàm (z >= -2); (2) cuộn -160..160 (bước 5): camera + giá với chi tiết cổ tay
không cuộn (vỏ/đáy/nắp J56, giá J5, chốt, khoá yaw), cả vị trí camera cũ để so; (3) hàm động quay quanh trục left_gripper URDF -40..18,56 (khép): với camera + giá;
(4) tầm nhìn: tia Ø1 từ tâm ống kính tới đỉnh hai đầu ngón khi khép và TCP khép (dừng 1,5 mm trước điểm) không cắt thân kẹp/hàm, góc lệch trục quang so FOV dọc 75° của sim;
(5) xuất STEP M1-AR-009 (khung cẳng tay, cổ tay 0, cuộn 0). Chạy: python cam_bracket_check.py -> ../export/cam_bracket_v045.json"""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
import assembly_full as A
A.GRIPPER = "am"   # G2B: công cụ này dựng cho kẹp J7 AM-ARM200 (v0.4.x); kẹp cũ v0.5 chưa vào sim/web (C1, C2)
import tcp_v045 as T
from build123d import Pos, Rot, Axis, Cylinder, Box, Align, Location, Plane, export_step, Compound

def vol(a, c):
    r = a & c
    return 0.0 if r is None else round(float(r.volume), 2)

def main():
    U = A.upstream()["p"]; S = A.step_to_fore(); R = A.wrist_ref(); F = R["fore"]
    cam = S * A.cam_wrist(); brk = S * A.cam_bracket_009()
    WD = (A.WCAM_DX, A.WCAM_DY, A.WCAM_DZ)
    A.WCAM_DX, A.WCAM_DY, A.WCAM_DZ = 0.0, 0.0, 0.0; cam_old = S * A.cam_wrist(); A.WCAM_DX, A.WCAM_DY, A.WCAM_DZ = WD
    out = {}
    bbs = lambda s: [round(v, 2) for v in (s.bounding_box().min.X, s.bounding_box().min.Y, s.bounding_box().min.Z, s.bounding_box().max.X, s.bounding_box().max.Y, s.bounding_box().max.Z)]
    grip = S * (Pos(0, -A.GRIP_SLIDE, A.GRIP_UP) * U["OB_F_J7_Gripper"]); jaw0 = S * (Pos(A.JAW_X, -A.GRIP_SLIDE, A.JAW_UP) * U["OB_F_J7_Jaw"])
    still = {n: S * U[n] for n in ("OB_F_J56_Wrist_Housing", "OB_F_J56_Bottom", "OB_F_J56_Top", "OB_F_J5_Elbow_Wrist_Bracket", "OB_F_J5_Pin_Bearing")}
    still["M1-AR-025_yaw_lock"] = S * A.yaw_lock_025()
    out["hop_bao"] = dict(camera_moi=bbs(cam), gia_009=bbs(brk), camera_cu=bbs(cam_old), vo_J56=bbs(still["OB_F_J56_Wrist_Housing"]))
    out["dat"] = dict(WCAM_DX=A.WCAM_DX, WCAM_DY=A.WCAM_DY, WCAM_DZ=A.WCAM_DZ, x_min=round(min(cam.bounding_box().min.X, brk.bounding_box().min.X), 2),
                      z_min_camera=round(cam.bounding_box().min.Z, 2), the_tich_gia_cm3=round(brk.volume / 1000, 2),
                      gia_giao_than_kep=vol(brk, grip), gia_giao_camera=vol(brk, cam), camera_giao_than_kep=vol(cam, grip),
                      khe_camera_than_kep=round(cam.distance_to(grip), 2), khe_camera_vo_J56=round(cam.distance_to(still["OB_F_J56_Wrist_Housing"]), 2),
                      khe_gia_vo_J56=round(brk.distance_to(still["OB_F_J56_Wrist_Housing"]), 2))
    r = F(R["roll"]); ry, rz = float(r[1]), float(r[2])
    def rollrows(objs):
        rows = []
        for roll in range(-160, 161, 5):
            Rr = Pos(0, ry, rz) * Rot(X=roll) * Pos(0, -ry, -rz); h = {}
            for on, o in objs.items():
                q = Rr * o
                for n, s in still.items():
                    v = vol(q, s)
                    if v: h[f"{on}|{n}"] = v
            if h: rows.append([roll, h])
        return rows
    out["cuon_moi"] = rollrows({"camera": cam, "gia_009": brk})
    out["cuon_cu"] = rollrows({"camera_cu": cam_old})
    org, ax = T.urdf_frames(); fdir = lambda d: np.array([-d[1], d[0], d[2]])
    piv = F(org["left_gripper"] + R["t"]); pax = fdir(ax["left_gripper"]); axis = Axis(tuple(piv), tuple(pax))
    jr = []
    for d in list(np.arange(-40.0, 18.0, 2.0)) + [18.56]:
        j = jaw0.rotate(axis, float(d)); h = {k: vol(j, o) for k, o in (("camera", cam), ("gia_009", brk))}
        jr.append([round(float(d), 2), h, round(min(j.distance_to(cam), j.distance_to(brk)), 2)])
    out["ham_dong"] = dict(giao=[x for x in jr if any(x[1].values())], khe_nho_nhat=min(x[2] for x in jr))
    # tầm nhìn
    tj = json.load(open(os.path.join(A.EXP, "tcp_v045.json")))
    Rc, O = R["cam"]; O2 = O + Rc[:, 2] * A.WCAM_K + np.array([A.WCAM_DY, -A.WCAM_DX, A.WCAM_DZ])
    lens = F(O2 + Rc[:, 2] * (A.CAM[2] + A.LENS[1])); opt = fdir(Rc[:, 2])
    jc = jaw0.rotate(axis, 18.56)
    vis = {}
    PGc, PJc = T.pts(grip, 0.1), T.pts(jc, 0.1)
    def top(P):
        t = P[P[:, 0] > P[:, 0].max() - 3]; return t[np.argmax(t[:, 2])]
    for name, p in (("dinh_dau_ngon_co_dinh", top(PGc[PGc[:, 1] > 0])), ("dinh_dau_ham_dong_khep", top(PJc)), ("TCP_khep", np.array(tj["TCP_khep_fore"]))):
        p = np.array(p, float); d = p - lens; L = float(np.linalg.norm(d)); u = d / L
        ray = Location(Plane(origin=tuple(lens), z_dir=tuple(u))) * Pos(0, 0, (L - 1.5) / 2) * Cylinder(0.5, L - 1.5)
        ang = math.degrees(math.acos(float(np.clip(u @ opt, -1, 1))))
        vis[name] = dict(diem=[round(float(v), 2) for v in p], goc_lech_truc_quang_deg=round(ang, 1), trong_FOV_doc_75=ang <= 37.5, che_than_kep_mm3=vol(ray, grip), che_ham_mm3=vol(ray, jc))
    out["tam_nhin"] = dict(tam_ong_kinh=[round(float(v), 2) for v in lens], truc_quang=[round(float(v), 3) for v in opt], tia=vis)
    export_step(brk, os.path.join(A.EXP, "M1-AR-009_wrist_cam_bracket_v045.step"))
    export_step(Compound([brk, cam]), os.path.join(A.EXP, "M1-AR-009_wrist_cam_bracket_v045_with_cam.step"))
    print(json.dumps(out, ensure_ascii=False))
    json.dump(out, open(os.path.join(A.EXP, "cam_bracket_v045.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
