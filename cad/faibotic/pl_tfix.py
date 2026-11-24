# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""P-L: đặt khung T AlohaMini2 theo khớp URDF thay cho đăng ký ICP (DK-N39).
Ghi ma trận 'post_W' (hệ robot, mm) vào ../export/AlohaMini2_placement.json; không đổi hình học upstream. Chạy lại được (lần hai dời ~0).

Vì sao: URDF AlohaMini2 gộp cả khung T vào một mesh vertical_link, không có khớp riêng cho từng chi tiết. Chi tiết có khớp URDF là đế tay:
left/right_base_joint (vertical_link -> left/right_Base, ±187,26; -59,9; 477,5 mm). Đế J1 AM-ARM200 (O_F_J1_Base) đã đặt theo khung left_Base
(đăng ký vào mesh URDF left_Base, sai 1,3 mm), tay phải dùng cùng phép đặt. Khung T trái/phải bắt vít lên đáy J1 nên vị trí của chúng suy
ra từ khớp URDF đó, không cần ICP:
  (1) T trái/phải: theo y, tâm 4 lỗ vít Φ3,2 (bước 60 x 80, khoét Φ6,5 từ dưới, khớp 4 trụ vít Φ5 dưới đáy J1) = y khớp URDF (±187,26);
      mặt trên khung T = z khớp URDF (573,7 ở lift 0; đáy J1 đặt bằng ICP ở 573,8, tâm vít J1 lệch y -1,4 mm, ghi lại). Theo x: rãnh thanh 2040 đồng trục lỗ thanh khung T giữa (tâm vít J1 lệch
      rãnh ~4 mm theo x: ghi 'lech_x_vit_J1_mm', điều kiện mở). Chỉ tịnh tiến, giữ hướng của bản bố trí.
  (2) Thanh ngang 2040: trục trùng lỗ thanh của khung T giữa (mặt cắt y = 0, lỗ 20,4 x 40,7; khung T giữa đăng ký sai 0,6 mm),
      giữa thanh ở y = 0 (hai đế tay URDF đối xứng).
Kiểm chéo: z suy từ khớp URDF cho rãnh thanh T trái/phải trùng lỗ khung T giữa trong 'lech_z_ranh_vs_URDF_mm' (dưới 0,5 mm).
Nhóm khung T giữa dời y để đối xứng qua y 0 (bước 0).
Post3-Post4: mối ghép chồng 15 mm như Post1-2, 2-3 (cùng phép đặt bản bố trí); boolean OCC lỗi trên cặp này nên ghi ở assembly_pl.INTENDED."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
from build123d import Plane, GeomType
import assembly_pl as P


def M4t(t):
    M = np.eye(4); M[:3, 3] = t; return M


def screw_center(conn):
    """Tâm 4 lỗ vít Φ3,2 xuyên mặt trên khung T trái/phải (mặt trụ r 1,6 chạm mặt trên) và z mặt trên."""
    top = conn.bounding_box().max.Z
    cs = [f.center() for f in conn.faces() if f.geom_type == GeomType.CYLINDER and abs(f.radius - 1.6) < 0.05 and f.bounding_box().max.Z > top - 0.2]
    assert len(cs) >= 4, len(cs)
    return np.array([np.mean([c.X for c in cs]), np.mean([c.Y for c in cs]), top]), len(cs)


def j1_center(j1):
    """Tâm 4 trụ vít Φ5 dưới đáy J1 (mặt trụ r 2,5 chạm đáy) và z đáy."""
    z0 = j1.bounding_box().min.Z
    cs = [f.center() for f in j1.faces() if f.geom_type == GeomType.CYLINDER and abs(f.radius - 2.5) < 0.1 and f.bounding_box().min.Z < z0 + 0.2]
    assert len(cs) >= 4, len(cs)
    return np.array([np.mean([c.X for c in cs]), np.mean([c.Y for c in cs]), z0]), len(cs)


def bar_hole_center(mid):
    """Tâm lỗ thanh 2040 trong khung T giữa: mặt cắt y = 0, lỗ trong có diện tích lớn nhất ở nửa trên."""
    best = None
    for f in mid.intersect(Plane.XZ).faces():
        for w in f.inner_wires():
            b = w.bounding_box(); a = (b.max.X - b.min.X) * (b.max.Z - b.min.Z)
            if best is None or (b.min.Z, a) > (best.min.Z, (best.max.X - best.min.X) * (best.max.Z - best.min.Z)): best = b
    return np.array([(best.min.X + best.max.X) / 2, (best.min.Z + best.max.Z) / 2]), (round(best.max.X - best.min.X, 2), round(best.max.Z - best.min.Z, 2))


def main():
    PL = json.load(open(os.path.join(P.EXP, "AlohaMini2_placement.json")))
    # (0) nhóm khung T giữa (khung, giá camera ngực, chốt; bánh răng theo gear_center_U) dời theo y để khung T giữa đối xứng qua y 0 như hai khớp URDF
    #     đế tay (ICP lệch -0,55 mm; không dời thì mặt dán khung T phải chồng 0,2 mm trên cả mặt, 168 mm3)
    mb = P.up_shape("OB_T_Connector_Middle").bounding_box(); dy0 = -(mb.min.Y + mb.max.Y) / 2
    if abs(dy0) > 0.01:
        for n in ("OB_T_Connector_Middle", "O_T_Connector_Dowel_Pin_12_25", "OB_T_Camera_Mount"):
            PL[n]["post_W"] = (M4t((0, dy0, 0)) @ np.array(PL[n].get("post_W", np.eye(4).tolist()))).tolist()
        PL["gear_center_U"][0] = round(PL["gear_center_U"][0] + dy0, 3); PL["rack_center_yW"] = PL.get("rack_center_yW")
        PL["nhom_T_giua_doi_y_mm"] = round(PL.get("nhom_T_giua_doi_y_mm", 0.0) + dy0, 3)
        json.dump(PL, open(os.path.join(P.EXP, "AlohaMini2_placement.json"), "w"), indent=1, ensure_ascii=False)
        P._UP.clear(); P.PLACE.clear(); P.PLACE.update(PL)
    print("nhom T giua doi y", round(dy0, 3), flush=True)
    q = P.AM.pose("ready", 0.0); rep = {}
    hc, hsz = bar_hole_center(P.up_shape("OB_T_Connector_Middle"))
    for side, name, ys in (("L", "OB_T_Connector_Left", 1), ("R", "OB_T_Connector_Right", -1)):
        j1 = [s for n, _, s in P.arm(side, q, 0.0) if "O_F_J1_Base" in n][0]
        conn = P.up_shape(name); cj, nj = j1_center(j1); cc, nc = screw_center(conn)
        cu = P.U2W(P.ARM_BASE_U[side])                     # khớp URDF left/right_base_joint (lift 0): tâm vít y, mặt trên z
        d = np.array([cj[0], cu[1], cu[2]]) - cc
        lech_j1 = np.round(cj - np.array([cj[0], cu[1], cu[2]]), 2).tolist()   # tâm trụ vít J1 (đặt bằng đăng ký ICP) so khớp URDF
        # x: rãnh thanh của khung T trái/phải (mặt cắt giữa đoạn thanh cắm vào) đồng trục lỗ thanh khung T giữa. Rãnh = lỗ 20,4 phía +x + hốc
        # đai ốc phía -x (z 537..554), nên tâm rãnh = mép +x - 20,4/2. Lệch so với tâm vít J1 (dx) ghi lại: điều kiện mở.
        y_mid = ys * 95.0 - d[1]
        inner = [w.bounding_box() for f in conn.intersect(Plane.XZ.offset(-y_mid)).faces() for w in f.inner_wires()]
        ib = max(inner, key=lambda b: (b.max.X - b.min.X) * (b.max.Z - b.min.Z))
        dx_rail = hc[0] - (ib.max.X - hsz[0] / 2); dz_rail = hc[1] - (ib.min.Z + ib.max.Z) / 2
        dx_j1 = d[0]; d[0] = dx_rail
        old = np.array(PL[name].get("post_W", np.eye(4).tolist()))
        PL[name]["post_W"] = (M4t(d) @ old).tolist()
        PL[name]["chinh"] = dict(cach="y, z theo khớp URDF left/right_base_joint (tâm lỗ vít y = y khớp, mặt trên = z khớp = đáy J1); x theo rãnh thanh đồng trục lỗ khung T giữa",
                                 lech_x_vit_J1_mm=round(float(dx_j1 - dx_rail), 2), lech_z_ranh_vs_URDF_mm=round(float(dz_rail - d[2]), 2), lech_vit_J1_vs_URDF_yz_mm=lech_j1[1:],
                                 tam_vit_J1_mm=np.round(cj, 2).tolist(), tam_vit_T_truoc_mm=np.round(cc, 2).tolist(), doi_them_mm=np.round(d, 2).tolist(),
                                 tong_doi_so_ICP_mm=np.round((M4t(d) @ old)[:3, 3], 2).tolist(), so_mat_tru=[nj, nc])
        rep[name] = PL[name]["chinh"]; print(name, rep[name], flush=True)
    bar = P.up_shape("O_T_Connector_Cross_Bar"); bb = bar.bounding_box()
    c = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2])
    d = np.array([hc[0], 0.0, hc[1]]) - c
    old = np.array(PL["O_T_Connector_Cross_Bar"].get("post_W", np.eye(4).tolist()))
    PL["O_T_Connector_Cross_Bar"]["post_W"] = (M4t(d) @ old).tolist()
    PL["O_T_Connector_Cross_Bar"]["chinh"] = dict(cach="trục thanh = tâm lỗ thanh khung T giữa (mặt cắt y 0), giữa thanh ở y 0", lo_thanh_mm=hsz,
                                                  tam_lo_xz=np.round(hc, 2).tolist(), doi_them_mm=np.round(d, 2).tolist(),
                                                  tong_doi_so_ICP_mm=np.round((M4t(d) @ old)[:3, 3], 2).tolist())
    print("bar", PL["O_T_Connector_Cross_Bar"]["chinh"], flush=True)
    PL.pop("post3_post4_diem_trong", None)
    PL["khung_T_ghi_chu"] = "DK-N39 (S1G): T trái/phải và thanh ngang đặt theo khớp URDF (pl_tfix.py), không còn chỉnh nhỏ theo giao cắt"
    json.dump(PL, open(os.path.join(P.EXP, "AlohaMini2_placement.json"), "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
