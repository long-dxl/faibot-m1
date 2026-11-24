# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Gộp kết quả đăng ký AlohaMini2 (am2_register.py -> ../export/AlohaMini2_registration.json) thành phép đặt dùng cho lắp ráp P-L
(../export/AlohaMini2_placement.json) và các số dẫn xuất (răng trụ, bánh răng, khung T, chân trụ). Chỉ numpy.

Phép đặt: p_U = R p_tệp + t (mm), hệ URDF base_link của AlohaMini2 ở vertical_move = 0. Nhóm cứng dùng chung t trung bình để chi tiết
cùng nhóm không lệch nhau vì sai số ICP: trụ 1-3 + giá camera đỉnh; khung T giữa + bánh răng + chốt + giá camera ngực.
Số răng trụ đo trên STL bố trí (mặt cắt, 23/11/2026): đỉnh x 1161,83, chân 1157,93, dải y -3192,76..-3164,76, răng z 265,3..863,8."""
import json, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); EXP = os.path.join(HERE, "../export")
RZ = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
POSTS = ["OB_Main_Assembly_Post1", "OB_Main_Assembly_Post2", "OB_Main_Assembly_Post3", "OB_Top_Camera_Mount"]
TGRP = ["OB_T_Connector_Middle", "OB_Z_Axis_Servo_Gear", "O_T_Connector_Dowel_Pin_12_25", "OB_T_Camera_Mount"]
OWN = ["O_Main_Assembly_Post4", "O_POST4_Connector_Base", "OB_T_Connector_Left", "OB_T_Connector_Right", "O_T_Connector_Cross_Bar",
       "O_F_J1_Base", "O_F_J2_Rotation"]


def main():
    reg = json.load(open(os.path.join(EXP, "AlohaMini2_registration.json")))
    tp = np.mean([reg[n]["t"] for n in POSTS], 0); tt = np.mean([reg[n]["t"] for n in TGRP], 0)
    P = {"_ghi_chu": __doc__.strip().splitlines()[0], "nhom_tru_t": tp.round(2).tolist(), "nhom_T_t": tt.round(2).tolist(),
         "khop_kem_nhat": {n: dict(mean=round(reg[n]["mean"], 2)) for n in ("OB_T_Connector_Left", "OB_T_Connector_Right", "O_T_Connector_Cross_Bar")}}
    for n in POSTS: P[n] = dict(R=RZ, t=tp.tolist(), mean=reg[n]["mean"])
    for n in TGRP: P[n] = dict(R=RZ, t=tt.tolist(), mean=reg[n]["mean"])
    P["OB_T_Connector_Middle"]["step_offset"] = [888.8, -217.9, 0.0]       # STEP khung T giữa lệch STL bố trí (so hộp bao, thể tích 535,9/541,4 cm3)
    for n in OWN: P[n] = dict(R=reg[n]["R"], t=reg[n]["t"], mean=reg[n]["mean"])
    R = np.array(RZ, float)
    P["gear_center_U"] = (R @ np.array([1167.21, -3179.99, 450.19]) + tt).round(2).tolist()     # tâm hộp bao bánh răng STL bố trí
    U = lambda p: R @ np.array(p, float) + tp
    P["rack_tip_xW"] = round(float(-U([1161.83, 0, 0])[1]), 2); P["rack_root_xW"] = round(float(-U([1157.93, 0, 0])[1]), 2)
    P["rack_center_yW"] = round(float((U([0, -3192.76, 0])[0] + U([0, -3164.76, 0])[0]) / 2), 2)
    P["rack_z_W"] = [round(265.3 + tp[2], 1), round(863.8 + tp[2], 1)]
    P["Tmid_top_zW"] = round(523.5 + tt[2], 2); P["Tmid_bot_zW"] = round(379.9 + tt[2], 2)
    P["post_foot_zW"] = round(-4.7 + reg["O_POST4_Connector_Base"]["t"][2], 2)
    json.dump(P, open(os.path.join(EXP, "AlohaMini2_placement.json"), "w"), indent=1, ensure_ascii=False)
    return P


if __name__ == "__main__":
    P = main(); print({k: v for k, v in P.items() if not isinstance(v, dict)})
