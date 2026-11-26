# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Vị trí 18 servo của robot P-L 6+1 để đánh dấu trên ảnh servo-id (web-assets/lap-rap).

Chạy (cần build123d, máy dựng CAD): python servo_markers.py OUT.json [--pose ready] [--lift 0]
Ra: {tên khớp SDK: {"id": ID servo, "xyz_mm": [x, y, z]}} hoặc {"id": ..., "mesh": tên chi tiết} (hệ W: x trước, y trái, z lên, mm).
- Servo có thân trong CAD (bánh 13-15, lift 16, yaw 21/22, kẹp 6/12): lấy tâm hộp bao của mesh (render_scene.py --markers đọc "mesh").
- Servo pan, vai, khuỷu, gập, cuộn (ID 1-5, 7-11): CAD không dựng thân servo, đặt tại gốc khớp theo FK của tay (như tay_cad() và spec.ARM_POINT_MASSES).
ID lấy từ sdk/faibotic/spec.py SERVO_IDS."""
import json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "..", "sdk"))
import assembly_pl as P
from faibotic import spec

LINK = {"pan": "left_Base", "shoulder": "left_Rotation_Pitch", "elbow": "left_Lower_Arm", "wrist_flex": "left_Wrist_Pitch_Roll", "wrist_roll": "left_Fixed_Jaw"}
MESH = {"wheel_0": "M1-SV-001_wheel0", "wheel_1": "M1-SV-001_wheel1", "wheel_2": "M1-SV-001_wheel2", "lift": "M1-LF-lift_servo",
        "left_gripper": "M1-SV-001_gripper_L", "right_gripper": "M1-SV-001_gripper_R", "left_wrist_yaw": "M1-SV-001_wrist_yaw_L", "right_wrist_yaw": "M1-SV-001_wrist_yaw_R"}


def main():
    out_path = sys.argv[1]
    pose = sys.argv[sys.argv.index("--pose") + 1] if "--pose" in sys.argv else "ready"
    lift = float(sys.argv[sys.argv.index("--lift") + 1]) if "--lift" in sys.argv else 0.0
    Tq = P.AM.fk(P._q(pose, 0.0, 0.0, 0.0))
    res = {}
    for side, pre in (("L", "left"), ("R", "right")):
        B = P.M4(P.RW) @ P.M4(np.eye(3), np.array(P.ARM_BASE_U[side]) + (0, 0, lift))
        for j, link in LINK.items():
            p = B @ np.append(np.array(Tq[link][:3, 3], float) * 1000.0, 1.0)
            res[f"{pre}_{j}"] = {"id": spec.SERVO_IDS[f"{pre}_{j}"], "xyz_mm": [round(float(v), 2) for v in p[:3]]}
    for k, m in MESH.items():
        res[k] = {"id": spec.SERVO_IDS[k], "mesh": m}
    assert len(res) == 18 and sorted(v["id"] for v in res.values()) == sorted(spec.SERVO_IDS.values())
    json.dump(res, open(out_path, "w"), indent=1, ensure_ascii=False)
    print("18 servo ->", out_path)


if __name__ == "__main__":
    main()
