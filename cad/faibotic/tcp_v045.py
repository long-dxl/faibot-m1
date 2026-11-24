# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""(22/11/2026): đo TCP kẹp thật trên CAD v0.4.4 (assembly_full: kẹp J7 + hàm động đặt như lắp ráp), so với spec.py.
Hàm động quay quanh trục khớp left_gripper của URDF upstream (am-arm200.urdf, quy về hệ STEP bằng wrist_ref, như cụm cổ tay) tới khi chạm hàm cố định
(giao thể tích > 0,05 mm3, chia đôi góc). Đầu ngón mỗi hàm: ngang/đứng = trọng tâm đỉnh lưới trong 3 mm cuối theo trục cuộn, dọc = mút ngón. TCP khép = trung điểm hai đầu ngón khi khép;
tâm kẹp kiểu SDK = TCP khép lùi TIP_LEN dọc trục cuộn. Mọi toạ độ trong khung cẳng tay (gốc trục khuỷu, x dọc cẳng tay, z lên), cổ tay 0, cuộn 0, tay trái;
và trong khung khớp cuộn (gốc wrist_roll URDF, x dọc trục cuộn, y/z như khung cẳng tay khi cuộn 0) để so spec.TCP_DY/TCP_DZ.
Chạy: python tcp_v045.py -> ../export/tcp_v045.json"""
import json, os, sys
import numpy as np
import xml.etree.ElementTree as ET
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
import assembly_full as A
A.GRIPPER = "am"   # G2B: công cụ này dựng cho kẹp J7 AM-ARM200 (v0.4.x); kẹp cũ v0.5 chưa vào sim/web (C1, C2)
from build123d import Pos, Axis
from faibotic import spec

def urdf_frames():
    T = {"left_Base": (np.eye(3), np.zeros(3))}; org, ax = {}, {}
    for j in ET.parse(A.URDF).getroot().findall("joint"):
        o = j.find("origin"); Rp, p = T[j.find("parent").get("link")]
        Rj = Rp @ A._rpy(*map(float, o.get("rpy").split())); pj = p + Rp @ np.array(list(map(float, o.get("xyz").split()))) * 1000
        org[j.get("name")] = pj; T[j.find("child").get("link")] = (Rj, pj)
        a = j.find("axis")
        if a is not None: ax[j.get("name")] = Rj @ np.array(list(map(float, a.get("xyz").split())))
    return org, ax

def pts(s, tol=0.05):
    v, _ = s.tessellate(tol, 0.1); return np.array([[p.X, p.Y, p.Z] for p in v])

def tip(P, n=3.0):
    t = P[P[:, 0] > P[:, 0].max() - n]; return t.mean(0), P[:, 0].max()

def main():
    R = A.wrist_ref(); F = R["fore"]; org, ax = urdf_frames()
    fdir = lambda d: np.array([-d[1], d[0], d[2]])
    piv = F(org["left_gripper"] + R["t"]); pax = fdir(ax["left_gripper"]); roll0 = F(org["left_wrist_roll"] + R["t"]); rax = fdir(ax["left_wrist_roll"])
    U = A.upstream()["p"]; S = A.step_to_fore()
    G = S * (Pos(0, -A.GRIP_SLIDE, A.GRIP_UP) * U["OB_F_J7_Gripper"]); J0 = S * (Pos(A.JAW_X, -A.GRIP_SLIDE, A.JAW_UP) * U["OB_F_J7_Jaw"])
    axis = Axis(tuple(piv), tuple(pax))
    from build123d import Box, Align
    # chỉ xét phần ngón (x >= 360): ở tư thế trưng bày hàm động đã tì vào thân kẹp quanh trục (khe 0, mô hình upstream), quay 1 độ đã giao vùng gốc hàm
    GF = G & Pos(360, -200, -200) * Box(100, 400, 400, align=(Align.MIN, Align.MIN, Align.MIN))
    def jaw(deg): return J0.rotate(axis, deg)
    def iv(deg):
        r = GF & jaw(deg); return 0.0 if r is None else float(r.volume)
    # chiều khép: chiều làm đầu hàm động tiến về hàm cố định (y tăng)
    sgn = 1.0 if tip(pts(jaw(5.0), 0.2))[0][1] > tip(pts(J0, 0.2))[0][1] else -1.0
    lo, hi = 0.0, 0.0
    for d in np.arange(1.0, 60.0, 1.0):
        if iv(sgn * d) > 0.05: hi = d; break
        lo = d
    for _ in range(14):
        m = (lo + hi) / 2; (lo, hi) = (lo, m) if iv(sgn * m) > 0.05 else (m, hi)
    th = sgn * lo
    PG, PJ0, PJc = pts(G), pts(J0), pts(jaw(th))
    fin = PG[PG[:, 1] > 0]                       # ngón cố định nằm phía +y của khe (đo ở tư thế trưng bày: y 6,3..16 ở đầu)
    tg, xg = tip(fin); tj0, xj0 = tip(PJ0); tjc, xjc = tip(PJc)
    tcp_closed = (tg + tjc) / 2; tcp_open = (tg + tj0) / 2
    tcp_closed[0] = (xg + xjc) / 2; tcp_open[0] = (xg + xj0) / 2      # dọc trục cuộn: mút đầu ngón (như test_wrist_upstream: x lớn nhất), ngang/đứng: trọng tâm 3 mm cuối
    grasp_closed = tcp_closed - rax * spec.TIP_LEN * 1000
    sdk_tcp = np.array([spec.L2 + spec.LT, spec.ROLL_DY + spec.TCP_DY, spec.WF_DZ + spec.ROLL_DZ + spec.TCP_DZ]) * 1000
    sdk_tip = sdk_tcp + rax * spec.TIP_LEN * 1000
    def inroll(p): d = p - roll0; return [round(float(d @ rax), 2), round(float(d[1]), 2), round(float(d[2]), 2)]
    flex = np.array(A.flex_axis())
    out = dict(
        ghi_chu="khung cẳng tay mm, cổ tay 0, cuộn 0, tay trái; khung cuộn: gốc wrist_roll URDF, [dọc trục cuộn, y, z]",
        truc_kep_URDF_fore=[round(float(v), 2) for v in piv], huong_truc_kep=[round(float(v), 3) for v in pax],
        goc_khep_deg=round(th, 2), giao_goc_ham_vs_than_kep_1_do_mm3=round(float((G & jaw(sgn * 1.0)).volume), 2), khe_dau_ngon_trung_bay_mm=round(float(np.linalg.norm(tg - tj0)), 2), khe_dau_ngon_khep_mm=round(float(np.linalg.norm(tg - tjc)), 2),
        dau_ngon_co_dinh=[round(float(v), 2) for v in tg], dau_ham_dong_trung_bay=[round(float(v), 2) for v in tj0], dau_ham_dong_khep=[round(float(v), 2) for v in tjc],
        TCP_khep_fore=[round(float(v), 2) for v in tcp_closed], TCP_khep_cuon=inroll(tcp_closed),
        tam_kep_khep_lui_TIP_LEN_cuon=inroll(grasp_closed), TCP_trung_bay_cuon=inroll(tcp_open),
        SDK_TCP_fore=[round(float(v), 2) for v in sdk_tcp], SDK_TCP_cuon=inroll(sdk_tcp), SDK_dau_ngon_cuon=inroll(sdk_tip),
        lech_tam_kep_khep_vs_SDK_TCP_mm=round(float(np.linalg.norm(grasp_closed - sdk_tcp)), 2),
        lech_dau_ngon_khep_vs_SDK_dau_ngon_mm=round(float(np.linalg.norm(tcp_closed - sdk_tip)), 2),
        thanh_phan_lech_cuon=[round(a - b, 2) for a, b in zip(inroll(grasp_closed), inroll(sdk_tcp))],
        dau_ngon_xa_nhat_x=dict(co_dinh=round(float(xg), 2), ham_dong_trung_bay=round(float(xj0), 2), ham_dong_khep=round(float(xjc), 2)),
        truc_gap_den_dau_ngon_khep_mm=round(float(tcp_closed[0] - flex[0]), 2), SDK_truc_gap_den_dau_ngon_mm=round((spec.LT + spec.TIP_LEN) * 1000, 2),
        truc_cuon_thap_hon_truc_gap_mm=round(float(flex[1] - roll0[2]), 2), tam_kep_khep_thap_hon_truc_gap_mm=round(float(flex[1] - grasp_closed[2]), 2),
    )
    print(json.dumps(out, ensure_ascii=False, indent=1))
    json.dump(out, open(os.path.join(A.EXP, "tcp_v045.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
