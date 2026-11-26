# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Khối lượng + trọng tâm tay M1 từ hình học AM-ARM200 (STEP, Apache-2.0) + gá vai Faibotic.
Chạy: python (venv có build123d) cad/faibotic/arm_mass.py  -> cad/export/arm_mass.json
 Mọi giả định có cờ; xem ../../archive/v0.4/cad/ASSUMPTIONS.md mục v0.2.

Hệ toạ độ STEP upstream (follower): tay nằm dọc -y, mặt đối xứng x = -18. Khuỷu: tâm cung đầu
OB_F_J4_Elbow; cổ tay J5: trục chốt OB_F_J5_Pin_*. Cả hai suy ra từ bbox [tự suy ra, STEP là bố cục
trưng bày, không có ràng buộc lắp]. s = khoảng cách dọc tay tính từ trục khớp gần của đoạn.
"""
import glob, json, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from build123d import import_step

HERE = os.path.dirname(os.path.abspath(__file__))
STP = os.path.join(HERE, "../upstream/AM-ARM/am-arm200/stp/follower")
OUT = os.path.join(HERE, "../export/arm_mass.json")

RHO = {"PETG": 1.27, "PLA": 1.24}   # g/cm3 [điển hình bảng dữ liệu Prusament, CHƯA kiểm nguồn lần này]
NOZZLE = 0.4                        # mm [giả định]
CASES = {"thap(4 thanh,15%)": (4, 0.15), "giua(6 thanh,20%)": (6, 0.20), "cao(6 thanh,40%)": (6, 0.40)}
# upstream 3d-print.md: follower 4-6 thành, infill 15-20%; "6 thành được khuyến nghị mạnh"; mức 40% là kịch bản xấu
M_STS3215 = 70.5   # g [số hãng tự báo]
M_STS3095 = 187.5  # g [số hãng tự báo, trang hãng; một nguồn 194,5]
M_SCREW = 1.1      # g M3x10 thép [tự suy ra: ~142 mm3 x 7,85 g/cm3]
M_CABLE = 4.0      # g mỗi cáp nối dài 26 cm [giả định]

Y_ELBOW = None; Y_WRIST = None


def load(name):
    return import_step(os.path.join(STP, name + ".stp"))


def metrics(s):
    c = s.center()
    return {"V_cm3": s.volume / 1000, "A_cm2": sum(f.area for f in s.faces()) / 100, "com": (c.X, c.Y, c.Z)}


def printed_mass_g(m, mat, walls, infill):
    V, A = m["V_cm3"], m["A_cm2"]
    shell = min(A * walls * NOZZLE / 10, V)  # cm3; A*t cận trên (bỏ trừ góc), kẹp bởi V
    return RHO[mat] * (shell + infill * (V - shell)), shell / V


def main():
    # trục khớp suy từ bbox
    j4 = load("OB_F_J4_Elbow").bounding_box()
    y_elbow = j4.max.Y - (j4.max.Z - j4.min.Z) / 2          # tâm cung đầu khuỷu
    z_elbow = (j4.max.Z + j4.min.Z) / 2
    pin = load("OB_F_J5_Pin_No_Bearing").bounding_box()
    y_wrist = (pin.min.Y + pin.max.Y) / 2
    gr = load("OB_F_J7_Gripper").bounding_box(); jw = load("OB_F_J7_Jaw").bounding_box()
    y_tip = max(-gr.min.Y, -jw.min.Y)                        # đầu ngón xa nhất (|y|)
    y_tip_fixed = -gr.min.Y
    L2 = y_elbow - y_wrist
    LT_tip = y_tip_fixed + y_wrist                           # wrist -> đầu ngón cố định (mm)
    # chi tiết Faibotic (hình học tham số)
    from params import LINK_LEN
    import shoulder
    link = shoulder.link(0.0); cradle = shoulder.cradle()
    parts = {}
    def addp(key, seg, s_ref, name=None, shape=None, axis_from="y", mat="PETG", origin="upstream"):
        shp = shape if shape is not None else load(name)
        m = metrics(shp)
        if origin == "upstream":
            s = s_ref - m["com"][1]                           # dọc -y từ trục s_ref (y_f của trục)
        else:
            s = m["com"][0]                                   # link Faibotic: x dọc tay từ trục vai
        parts[key] = dict(seg=seg, origin=origin, file=(name + ".stp") if name else "faibotic/shoulder.py", mat=mat,
                          V_cm3=round(m["V_cm3"], 2), A_cm2=round(m["A_cm2"], 1), s_com_mm=round(s, 1), **{"com_stp": [round(c, 1) for c in m["com"]]})
    addp("link_vai_khuyu(Faibotic)", "upper", 0, shape=link, origin="faibotic")
    addp("J4_Elbow", "fore", y_elbow, "OB_F_J4_Elbow")
    addp("J5_Elbow_Wrist_Bracket", "fore", y_elbow, "OB_F_J5_Elbow_Wrist_Bracket")
    for n in ("OB_F_J56_Wrist_Housing", "OB_F_J56_Bottom", "OB_F_J56_Top", "OB_F_J5_Pin_No_Bearing"):
        addp(n, "wrist", y_wrist, n)
    for n in ("OB_F_J7_Gripper", "OB_F_J7_Jaw", "OB_F_Camera"):
        addp(n, "hand", y_wrist, n)
    # đối chiếu, không tính vào tay
    extra = {}
    for n in ("OB_F_J3_Shoulder", "O_F_J1_Base", "O_F_J2_Rotation", "OB_F_WaveShare_Mounting_Plate", "OB_F_J5_Pin_Bearing"):
        m = metrics(load(n)); extra[n] = {"V_cm3": round(m["V_cm3"], 2), "A_cm2": round(m["A_cm2"], 1)}
    cm = metrics(cradle)
    extra["cradle(Faibotic)"] = {"V_cm3": round(cm["V_cm3"], 2), "A_cm2": round(cm["A_cm2"], 1)}

    LW = 50.0   # mm: gốc thân "hand" (khớp roll) cách trục gập cổ tay; giữ spec cũ
    seg_len = {"upper": 0.0, "fore": 0.0, "wrist": 0.0, "hand": LW}
    # linh kiện mua sẵn (s từ trục gần của đoạn)
    purchased = [
        ("STS3095_khuyu", "fore", M_STS3095, 0.0, "tại trục khuỷu [giả định]"),
        ("STS3215_gap_co_tay", "fore", M_STS3215, L2, "tại trục cổ tay [giả định]"),
        ("STS3215_xoay_co_tay", "wrist", M_STS3215, None, "tại trọng tâm vỏ J56 [giả định]"),
        ("STS3215_kep", "hand", M_STS3215, None, "tại trọng tâm J7 [giả định]"),
    ]
    screws = {"upper": 6, "fore": 22, "wrist": 24, "hand": 22}   # tổng 74 [bom.md]; phân bổ là giả định
    cables = {"upper": 1, "fore": 2, "wrist": 2, "hand": 1}
    segs = {}
    for case, (w, inf) in CASES.items():
        out = {}
        for seg in ("upper", "fore", "wrist", "hand"):
            items = []
            for k, p in parts.items():
                if p["seg"] != seg: continue
                g, sf = printed_mass_g(dict(V_cm3=p["V_cm3"], A_cm2=p["A_cm2"]), p["mat"], w, inf)
                s = p["s_com_mm"] - (LW if seg == "hand" else 0.0)
                items.append((k, g, s))
            segparts = [p["s_com_mm"] for p in parts.values() if p["seg"] == seg]
            sm = sum(i[1] for i in items)
            s_print = sum(i[1] * i[2] for i in items) / sm if sm else 0
            for name, sg, g, s, note in [(a, b, c, d, e) for a, b, c, d, e in purchased if b == seg]:
                if s is None: s = s_print                       # đặt tại trọng tâm phần in
                else: s = s - (LW if seg == "hand" else 0.0)
                items.append((name, g, s))
            sm_int = sum(i[1] for i in items)
            s_mid = sum(i[1] * i[2] for i in items) / sm_int
            items.append(("oc_vit", screws[seg] * M_SCREW, s_mid)); items.append(("cap", cables[seg] * M_CABLE, s_mid))
            m = sum(i[1] for i in items)
            sc = sum(i[1] * i[2] for i in items) / m
            out[seg] = {"mass_g": round(m, 1), "com_from_joint_mm": round(sc, 1)}
        segs[case] = out
    # thân xoay (pan) khung "shoulder": cradle + 2 STS3095 + J2_Rotation (không vào mô-men vai)
    pan_body = {}
    for case, (w, inf) in CASES.items():
        g_c, _ = printed_mass_g(cm, "PETG", w, inf)
        g_j2, _ = printed_mass_g(dict(V_cm3=extra["O_F_J2_Rotation"]["V_cm3"], A_cm2=extra["O_F_J2_Rotation"]["A_cm2"]), "PETG", w, inf)
        pan_body[case] = round(g_c + g_j2 + 2 * M_STS3095, 1)
    tot = {c: round(sum(v["mass_g"] for v in s.values()), 1) for c, s in segs.items()}
    res = {
        "ngay": "21/11/2026",
        "gia_dinh": {"density_g_cm3": RHO, "nozzle_mm": NOZZLE, "cases(thanh,infill)": CASES,
                     "vo_in": "thể tích vỏ = min(diện tích x thanh x 0,4 mm, V) (cận trên); nội thất = infill x phần còn lại",
                     "cờ": "mật độ [điển hình, chưa kiểm nguồn]; thành/infill từ 3d-print.md upstream (4-6 thanh, 15-20%)",
                     "STS3215_g": M_STS3215, "STS3095_g": M_STS3095, "oc_g": M_SCREW, "cap_g": M_CABLE,
                     "vi_tri_servo": "đặt tại trục khớp/trọng tâm vỏ [giả định, thân servo thật lệch ~20 mm]"},
        "truc_khop_suy_tu_bbox_mm": {"y_elbow_stp": round(y_elbow, 1), "z_elbow_stp": round(z_elbow, 1), "y_wrist_stp": round(y_wrist, 1),
                                     "y_dau_ngon_co_dinh": round(-y_tip_fixed, 1), "y_dau_ngon_xa_nhat": round(-y_tip, 1)},
        "chieu_dai_mm": {"L1_link_Faibotic": LINK_LEN, "L2_khuyu_co_tay_CAD": round(L2, 1), "co_tay_den_dau_ngon_CAD": round(LT_tip, 1),
                         "tam_voi_den_dau_ngon": round(LINK_LEN + L2 + LT_tip, 1)},
        "chi_tiet": parts, "doi_chieu_khong_tinh": extra,
        "doan_tay": segs, "tong_tay_g": tot, "than_pan_g": pan_body,
        "ghi_chu": "chưa gồm module camera (chỉ giá đỡ), WaveShare, dây nguồn/bus lên cột; trục khớp suy từ bbox",
    }
    json.dump(res, open(OUT, "w"), ensure_ascii=False, indent=1)
    print(json.dumps({k: res[k] for k in ("chieu_dai_mm", "doan_tay", "tong_tay_g", "than_pan_g", "truc_khop_suy_tu_bbox_mm")}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
