# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""(22/11/2026): bằng chứng số cho ba lệch cũ, đo trên CAD hiện hành (assembly_full v0.4.5). build123d.
(1) Đế xoay M1-LF-024 và tấm giao diện M1-LF-023b đồng trục pan dưới trục vai như SDK (spec.SHOULDER_X/Y); trước R0 lùi 83 mm.
(2) Chặn dưới M1-SH-017 + đệm M1-SH-018 với tay đỡ M1-LF-023 + tấm giao diện: thể tích giao và khe nhỏ nhất theo vai -80..62 và pan -70..70 (bước 10),
    hệ vai (xe nâng tĩnh, chặn dưới gắn gá vai quay theo pan); trước R0 giao ~1.385 mm3 mỗi tay.
(3) Đầu ngón: CAD (khép, cad/export/tcp_v045.json) so SDK (spec) tính từ trục gập và từ chốt J5 cũ.
Chạy: python dk15_check.py -> ../export/dk15_v045.json"""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from build123d import Pos, Rot
import assembly_full as A, failsafe_v042_low as L, lift
from faibotic import spec

def main():
    out = {}
    hub = A.pan_hub_024().bounding_box()
    za = lift.Z_AX0 + 200.0
    car = lift.carriage(za); Tm = Pos(-lift.SX, -lift.SY, -za)        # xe nâng -> hệ vai trái
    pad = (Tm * car["padL"]).bounding_box()
    out["(1)_truc_pan"] = dict(de_xoay_024_tam_xy_he_vai_mm=[round((hub.min.X + hub.max.X) / 2, 3), round((hub.min.Y + hub.max.Y) / 2, 3)],
                               tam_giao_dien_023b_xy_he_vai_mm=[round((pad.min.X + pad.max.X) / 2, 3), round((pad.min.Y + pad.max.Y) / 2, 3)],
                               truc_vai_CAD_the_gioi_mm=[lift.SX, lift.SY], SDK_SHOULDER_XY_mm=[spec.SHOULDER_X * 1000, spec.SHOULDER_Y * 1000],
                               lech_mm=round(math.hypot((hub.min.X + hub.max.X) / 2, (hub.min.Y + hub.max.Y) / 2) + math.hypot(lift.SX - spec.SHOULDER_X * 1000, lift.SY - spec.SHOULDER_Y * 1000), 3))
    lp, br, _ = L.shapes(45.0, 50.0)
    obs = {"tay_do_023": Tm * car["outL"], "giao_dien_023b": Tm * car["padL"]}
    rows = []; vmax = 0.0; dmin = 1e9
    for pan in range(-70, 71, 10):
        P = Rot(Z=pan)
        for qs in (-80, -40, 0, 30, 48.5, 55, 62):    # chặn dưới gắn gá vai: không phụ thuộc vai; giữ vòng lặp để ghi rõ
            for n, s in (("chan_duoi_017", br), ("dem_018", lp)):
                for on, o in obs.items():
                    r = (P * s) & o; v = 0.0 if r is None else float(r.volume); d = float((P * s).distance_to(o))
                    vmax = max(vmax, v); dmin = min(dmin, d)
                    if v > 0: rows.append([pan, qs, n, on, round(v, 2)])
            break
    out["(2)_chan_duoi_vs_tay_do"] = dict(giao=rows, giao_lon_nhat_mm3=round(vmax, 2), khe_nho_nhat_mm=round(dmin, 2), pan=[-70, 70, 10],
                                           ghi_chu="chặn dưới + đệm gắn gá vai (quay theo pan, không theo vai); tay đỡ/tấm giao diện tĩnh theo xe nâng")
    tj = json.load(open(os.path.join(A.EXP, "tcp_v045.json")))
    fx = A.flex_axis()[0]; R = A.wrist_ref(); pin_x = float(R["fore"](R["yaw"])[0])
    tip = tj["TCP_khep_fore"][0]; sdk_tip = (spec.L2 + spec.LT + spec.TIP_LEN) * 1000
    out["(3)_dau_ngon"] = dict(CAD_khep_tu_truc_gap_mm=round(tip - fx, 2), SDK_tu_truc_gap_mm=round(sdk_tip - spec.L2 * 1000, 2),
                               CAD_khep_tu_chot_J5_mm=round(tip - pin_x, 2), SDK_tu_chot_J5_mm=round(sdk_tip - pin_x, 2), lech_mm=round(tip - sdk_tip, 2),
                               ghi_chu="số cũ 171,7 (CAD) / 165 (SDK) đo trước từ trục cổ tay giả định (chốt J5); nay cùng chuỗi khớp URDF, đo dọc trục cuộn ở gập 0")
    print(json.dumps(out, ensure_ascii=False))
    json.dump(out, open(os.path.join(A.EXP, "dk15_v045.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
