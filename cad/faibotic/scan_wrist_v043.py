# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""v0.4.3: quét giao cắt kẹp J7 + vỏ cổ tay J56 (AM-ARM200) với giá chặn và gá (vai x khuỷu x cổ tay). build123d.
Vật cản: giá chặn dưới M1-SH-017 v3 + đệm M1-SH-018 v3, thanh chặn trên M1-SH-016 + đệm M1-SH-015, gá vai M1-SH-003 (cradle_v4).
Trục cổ tay suy từ chốt J5 [giả định như arm_fit]; cuộn cổ tay 0 (quét theo cuộn: Chưa có số liệu).
Vòng 2 (QC2 v0.4.3): thêm link M1-SH-004, chốt cân bằng M6 và hai lò xo (gắn link, theo vai) và cuộn cổ tay (quay kẹp J7 quanh trục song song trục cẳng tay qua tâm vỏ J56 [giả định]).
Chạy: python scan_wrist_v043.py LUOI.json RA.json   (LUOI.json = {"shoulders":[..],"elbows":[..],"wrists":[..],"rolls":[..]} hoặc {"cells":[[qs,qe,[qw..]],..],"rolls":[..]}); dòng ra [qs,qe,qw,cuon,{vật:mm3}]"""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(__file__))
from multiprocessing import Pool
from build123d import import_step, Pos, Rot
import balance as b, failsafe as f, failsafe_v042 as g2, failsafe_v042_low as L
from params import LINK_LEN

STP = os.path.join(os.path.dirname(__file__), "..", "upstream", "AM-ARM", "am-arm200", "stp", "follower") + os.sep
NAMES = ["OB_F_J56_Bottom", "OB_F_J56_Top", "OB_F_J56_Wrist_Housing", "OB_F_J7_Gripper", "OB_F_J7_Jaw"]
G = {}

def init():
    j4 = import_step(STP + "OB_F_J4_Elbow.stp"); bb = j4.bounding_box()
    xc, ye, ze = (bb.min.X + bb.max.X) / 2, bb.max.Y - (bb.max.Z - bb.min.Z) / 2, (bb.min.Z + bb.max.Z) / 2
    pin = import_step(STP + "OB_F_J5_Pin_Bearing.stp").bounding_box(); yw = (pin.min.Y + pin.max.Y) / 2
    G["L2"] = ye - yw; G["c"] = (xc, ye, ze)
    G["parts"] = {n: import_step(STP + n + ".stp") for n in NAMES}
    hb = G["parts"]["OB_F_J56_Wrist_Housing"].bounding_box()
    G["roll_c"] = ((hb.min.X + hb.max.X) / 2, (hb.min.Z + hb.max.Z) / 2)   # trục cuộn // trục y của STEP qua tâm vỏ J56 [giả định]
    G["cache"] = {}
    pad, br, _ = L.shapes(45.0, 50.0)
    xt = g2.find_top_xf()
    G["obs"] = {"chan_duoi": br, "dem_duoi": pad, "thanh_tren": g2.top_bar(xt), "dem_tren": g2.top_pad(xt), "gia_vai": f.cradle_v4()}

def obstacles(qs):
    c = G["cache"]
    if qs not in c:
        o = dict(G["obs"])
        o["link"] = b.link_v2(qs); o["chot"] = b.pin(qs); o["lo_xo_t"] = b.spring(qs, -1); o["lo_xo_p"] = b.spring(qs, 1)
        c.clear(); c[qs] = o
    return c[qs]

def one(t):
    qs, qe, qw, roll = t
    obs = obstacles(qs)
    L2 = G["L2"]; xc, ye, ze = G["c"]
    loc = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe) * Pos(L2, 0, 0) * Rot(Y=qw) * Pos(-L2, 0, 0)
    fr = loc * Rot(Z=90) * Pos(-xc, -ye, -ze)
    hit = {}
    xr, zr = G["roll_c"]
    R = Pos(xr, 0, zr) * Rot(Y=roll) * Pos(-xr, 0, -zr)
    for n, p in G["parts"].items():
        q = fr * (R * p if (roll and n.startswith("OB_F_J7")) else p)
        for on, o in obs.items():
            v = b.inter(q, o)[0] or 0.0
            if v:
                hit[f"{n}|{on}"] = round(v, 1)
    return [qs, qe, qw, roll, hit]

if __name__ == "__main__":
    grid = json.load(open(sys.argv[1]))
    if "cells" in grid:
        tasks = [(qs, qe, qw, r) for qs, qe, ws in grid["cells"] for qw in ws for r in grid.get("rolls", [0])]
    else:
        tasks = [(qs, qe, qw, r) for qs in grid["shoulders"] for qe in grid["elbows"] for qw in grid["wrists"] for r in grid.get("rolls", [0])]
    t0 = time.time(); out = []
    with Pool(int(os.environ.get("M1_PROCS", "2")), initializer=init) as p:
        for i, r in enumerate(p.imap(one, tasks, chunksize=4)):
            out.append(r)
            if i % 200 == 0:
                print(i, len(tasks), round(time.time() - t0), flush=True)
    json.dump({"obstacles": ["chan_duoi", "dem_duoi", "thanh_tren", "dem_tren", "gia_vai", "link", "chot", "lo_xo_t", "lo_xo_p"], "rows": out}, open(sys.argv[2], "w"))
    print("done", round(time.time() - t0), "hits", sum(1 for r in out if r[4]), "of", len(out))
