# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""S1G: kích thước và khối lượng thân P-L cho hình va chạm sim (sdk/faibotic/spec.py) lấy từ CAD assembly_pl.py, ghi ../export/M1-PL-sim-dims.json.
tests/test_pl_sim_dims.py so spec.py với tệp này (sai <= 0,5 mm, 0,01 kg), để sim và CAD cùng một nguồn.
Hệ robot (m): base = sàn tại tâm tam giác bánh; carriage = gốc tay (đáy J1) ở lift SDK 0, tức lift CAD spec.LIFT_CAD_OFFSET.
Chạy (venv build123d, sau assembly_pl.py --masses): python pl_sim_dims.py"""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
import numpy as np
from build123d import Compound
import assembly_pl as P
from faibotic import spec


def bb(parts, names):
    s = [so for n, _, x in parts if n.startswith(names) for so in x.solids()]      # chỉ solid: STEP Post1 có thực thể lạc ở gốc bố trí (y_W 3171)
    b = Compound(s).bounding_box(); return np.array([b.min.X, b.min.Y, b.min.Z, b.max.X, b.max.Y, b.max.Z]) / 1000


def main():
    L0 = spec.LIFT_CAD_OFFSET * 1000; zc = P.U2W(P.ARM_BASE_U["L"])[2] + L0          # gốc xe nâng (mm)
    base = P.base_parts() + P.tray_parts() + P.column(); car = P.T_frame(L0) + P.dock(L0) + P.station(L0)
    h = P.base_heights(); r = lambda v: [round(float(x), 5) for x in v]
    cw = bb(base, ("M1-BS-032", "M1-BS-039")); tr = bb(base, ("M1-ST-012",)); col = bb(base, ("M1-LF-051", "M1-LF-052"))
    foot = bb(base, ("M1-LF-054", "M1-LF-055")); deck = bb(base, ("M1-BS-037",))
    tmid = bb(car, ("M1-LF-057",)); rail = bb(car, ("M1-TR-001",)); ear = bb(car, ("M1-TR-002_tai_2040_L",)); mast = bb(car, ("M1-TR-006",))
    gimb = bb(car, ("M1-TR-007", "M1-CM-001_head")); dock = bb(car, ("M1-TR-020",))
    for b in (tmid, rail, ear, mast, gimb, dock): b[[2, 5]] -= zc / 1000
    # camera đầu (trạm): gốc camera sim = đầu ống kính, trục nhìn chúi TR.CAM_TILT; khung xe nâng (x như robot, z từ đáy J1)
    import tram_hai_tay as TR
    o = np.array([P.U2W(P.ARM_BASE_U["L"])[0], 0.0, 0.0])
    look = TR.cam_look(); lens = (o + TR.cam_lens_point()) / 1000; axis = (o + TR.cam_axis_point()) / 1000
    zc_ = -look; xc = np.array([0.0, -1.0, 0.0]); yc = np.cross(zc_, xc)
    M = json.load(open(os.path.join(P.EXP, "M1-PL-masses.json"))); T = M["chi_tiet"]
    kg = lambda pre: sum(v["kg"] for k, v in T.items() if k.startswith(pre))
    spokes = kg("M1-BS-031") / 3; column = kg(("M1-LF-051", "M1-LF-052", "M1-LF-053", "M1-LF-054", "M1-LF-055"))
    cwm = M["co_dinh"][M["doi_trong_key"]][0]; tray = kg(("M1-ST-011", "M1-ST-012", "M1-BS-006_2040_do_khay"))      # khay đế 4 mảnh + cột đỡ + thanh 2040 đỡ khay
    base_rest = M["de_kg"] - cwm - 3 * spokes - tray - kg("M1-BS-035")    # sàn, giá bánh, khoang điện tử, servo bánh (bánh: body bánh trong sim)
    out = dict(nguon="cad/faibotic/assembly_pl.py + cad/export/M1-PL-masses.json (pl_sim_dims.py)", lift_cad_mm_o_lift_sdk_0=L0, goc_xe_nang_z_m=round(zc / 1000, 6),
               BASE_RADIUS=round(float((deck[3] - deck[0]) / 2), 5), BASE_Z=r([h["spoke_bot"] / 1000, h["deck_top"] / 1000]),
               SPOKE_R=[P.SPOKE_R0 / 1000, P.SPOKE_R1 / 1000], SPOKE_Z=r([h["spoke_bot"] / 1000, h["spoke_top"] / 1000]),
               COUNTERWEIGHT_HALF=r((cw[3:] - cw[:3]) / 2), COUNTERWEIGHT_POS=r((cw[3:] + cw[:3]) / 2), TRAY_HALF=r((tr[3:] - tr[:3]) / 2), TRAY_POS=r((tr[3:] + tr[:3]) / 2),
               COLUMN_XR=r([col[0], col[3]]), COLUMN_YH=round(float(max(-col[1], col[4])), 5), COLUMN_TOP=round(float(col[5]), 5),
               COLUMN_FOOT_XR=r([foot[0], foot[3]]), COLUMN_FOOT_YH=round(float(max(-foot[1], foot[4])), 5), COLUMN_FOOT_Z=r([foot[2], foot[5]]),
               TMID_XR=r([tmid[0], tmid[3]]), TMID_YH=round(float(max(-tmid[1], tmid[4])), 5), TMID_Z=r([tmid[2], tmid[5]]),
               STATION_RAIL_XR=r([rail[0], rail[3]]), STATION_RAIL_YH=round(float(max(-rail[1], rail[4])), 5), STATION_RAIL_Z=r([rail[2], rail[5]]),
               STATION_EAR_XR=r([ear[0], ear[3]]), STATION_EAR_Y=r([ear[1], ear[4]]), STATION_EAR_Z=r([ear[2], ear[5]]),
               HEAD_MAST_XR=r([mast[0], mast[3]]), HEAD_MAST_YH=round(float(max(-mast[1], mast[4])), 5), HEAD_MAST_Z=r([mast[2], mast[5]]),
               HEAD_GIMBAL_XR=r([gimb[0], gimb[3]]), HEAD_GIMBAL_YH=round(float(max(-gimb[1], gimb[4])), 5), HEAD_GIMBAL_Z=r([gimb[2], gimb[5]]),
               DOCK_XR=r([dock[0], dock[3]]), DOCK_YH=round(float(max(-dock[1], dock[4])), 5), DOCK_Z=r([dock[2], dock[5]]),
               SHOULDER_Y=round(P.ARM_BASE_U["L"][0] / 1000, 5), H_J1_ABOVE_TABLE=TR.H_J1_TREN_BAN / 1000, HEAD_CAM_TILT_DEG=TR.CAM_TILT,
               HEAD_CAM_AXIS=r(axis), HEAD_CAM_POS=r(lens), HEAD_CAM_XYAXES=r(list(xc) + list(yc)), WHEEL_RADIUS=P.WR / 1000,
               kg=dict(COUNTERWEIGHT_MASS=round(cwm, 3), SPOKE_MASS=round(spokes, 3), COLUMN_MASS=round(column, 3), TRAY_MASS=round(tray, 3),
                       BASE_MASS=round(base_rest, 3), CARRIAGE_MASS=round(M["xe_nang_kg"], 3), STATION_MASS=round(M["tram_kg"], 3), omni_moi_banh=P.OMNI_KG),
               STATION_COM=M["tram_tam_khung_xe_m"])
    json.dump(out, open(os.path.join(P.EXP, "M1-PL-sim-dims.json"), "w"), indent=1, ensure_ascii=False)
    print(json.dumps(out, ensure_ascii=False))


if __name__ == "__main__":
    main()
