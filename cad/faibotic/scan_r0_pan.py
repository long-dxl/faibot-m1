# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Quét R0 (23/11/2026): đế xoay M1-LF-024 + cụm vai quay pan so với xe nâng; đế xoay so với chi tiết quay theo vai; đệm khuỷu M1-AR-022 so với tay đỡ.
build123d. Chạy: nice -n 10 python scan_r0_pan.py -> ../export/M1-LF-R0-scan.json
(1) đế xoay với gá vai, giá chặn M1-SH-017, đệm M1-SH-018, servo vai (tĩnh) và link, chốt, lò xo ở vai -80..62 (bước 2).
(2) cụm quay pan (đế xoay, giá chặn, đệm, gá vai, link + đệm khuỷu AR-022 ở vai 0/30/48,5/50/55/62) quay pan -70..70 (bước 10), lift 0, so với mọi chi tiết xe nâng (tay trái).
Không gồm cẳng tay, cổ tay, kẹp, tay bên kia (va chạm tay-tay: xem 08)."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from build123d import Pos, Rot
import assembly_full as A, balance as b, failsafe as f, failsafe_v042_low as L, failsafe_v043 as g3, shoulder as sh, lift

def v(a, c):
    return round(b.inter(a, c)[0] or 0.0, 1)

def main():
    hub = A.pan_hub_024(); pad, br, _ = L.shapes(45.0, 50.0); cr = f.cradle_v4()
    out = {"tinh": {}, "quay_theo_vai": [], "pan": [], "tieu_chi_mm3": 0.5}
    for k, s in (("cradle", cr), ("SH-017", br), ("SH-018", pad), ("servoL", sh.servo(-1)), ("servoR", sh.servo(1))):
        out["tinh"][k] = v(hub, s)
    for q in range(-80, 63, 2):
        for k, s in (("link", b.link_v2(q)), ("pin", b.pin(q)), ("spring0", b.spring(q, -1)), ("spring1", b.spring(q, 1))):
            x = v(hub, s)
            if x > out["tieu_chi_mm3"]: out["quay_theo_vai"].append((q, k, x))
    car = lift.carriage(lift.Z_AX0)
    for qs in (0.0, 30.0, 48.5, 50.0, 55.0, 62.0):
        lt = Rot(Y=qs)
        rot = {"LF-024": hub, "SH-017": br, "SH-018": pad, "SH-003": cr, "SH-004_link": b.link_v2(qs), "AR-022_elbow_pad": lt * g3.pad_pu(), "AR-024": lt * g3.pad_base()}
        for pan in range(-70, 71, 10):
            T = Pos(lift.SX, lift.SY, lift.Z_AX0) * Rot(Z=pan)
            for n, s in rot.items():
                for k, fx in car.items():
                    if k.endswith("R"): continue
                    x = v(T * s, fx)
                    if x > out["tieu_chi_mm3"]: out["pan"].append(dict(vai=qs, pan=pan, chi_tiet=n, xe=k, mm3=x))
    out["ket_luan"] = dict(tinh_0=all(x <= 0.5 for x in out["tinh"].values()), quay_theo_vai_0=not out["quay_theo_vai"], pan_so_cham=len(out["pan"]))
    json.dump(out, open(os.path.join(HERE, "../export/M1-LF-R0-scan.json"), "w"), ensure_ascii=False, indent=1)
    print(json.dumps(out["ket_luan"]), json.dumps(out["pan"][:30], ensure_ascii=False))

if __name__ == "__main__":
    main()
