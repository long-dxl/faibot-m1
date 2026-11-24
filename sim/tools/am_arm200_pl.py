"""Suy số tay AM-ARM200 gốc (P-L) cho spec.py từ URDF upstream: kích thước khâu, góc lệch quy ước, khối lượng/trọng tâm/quán tính từng body sim.

URDF: cad/upstream/AM-ARM/am-arm200/urdf/am-arm200.urdf (SolidWorks exporter, commit 5b03af1e). Khối lượng URDF = thể tích chi tiết in x 1,0 g/cm³
(đối chiếu: OB_F_J7_Jaw 28,18 cm³ ~ left_Moving_Jaw 28,2 g; J7_Gripper 61,73 + Camera 12,63 = left_Fixed_Jaw 74,4 g), KHÔNG có servo.
Quy ước SDK (khung tay trái, gốc ở đáy J1 trên trục pan, x trước, y trái, z lên): tư thế 0 = cánh tay vai-khuỷu nằm ngang ra trước, cẳng tay thẳng hàng,
trục cuộn song song cẳng tay; vai/khuỷu/gập dương = gập xuống. Đổi góc: q_sdk = dấu x q_urdf + lệch (URDF_TO_SDK).

Chạy: python sim/tools/am_arm200_pl.py   (in JSON; tests/test_pl_arm_urdf.py so với spec.py)
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "cad" / "upstream" / "AM-ARM" / "am-arm200" / "urdf" / "am-arm200.urdf"
# hệ SDK theo hệ thế giới URDF: x_sdk = -y_urdf, y_sdk = x_urdf, z_sdk = z_urdf
R_SDK = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
# link URDF -> body sim (6+1: wrist_yaw là body riêng sau khớp yaw như upstream; trước khoá cứng vào wrist; left_camera URDF có khối 1,27 kg, tâm cách kẹp 0,17 m: lỗi xuất, bỏ).
# kẹp gốc AM-ARM200 là kẹp mặc định: left_Fixed_Jaw (thân kẹp + giá camera) -> hand, left_Moving_Jaw -> jaw (ở góc khép,
# cad/export/AM_gripper_PL.json goc_khep_urdf_deg; gốc body jaw = trục khớp left_gripper).
LINK_BODY = {"left_Base": "mount", "left_Rotation_Pitch": "shoulder", "left_Upper_Arm": "upper", "left_Lower_Arm": "fore",
             "left_Wrist_Pitch_Roll": "wrist", "left_wrist_yaw": "yaw", "left_Fixed_Jaw": "hand", "left_Moving_Jaw": "jaw"}
GRIP_JSON = ROOT / "cad" / "export" / "AM_gripper_PL.json"


def grip_close_urdf() -> float:
    """Góc URDF left_gripper (rad) khi hai ngón chạm (CAD, cad/faibotic/gripper_am.py --scan)."""
    return math.radians(json.loads(GRIP_JSON.read_text())["goc_khep_urdf_deg"])


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axang(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def parse():
    root = ET.parse(URDF).getroot()
    joints, links = {}, {}
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        joints[j.get("name")] = dict(parent=j.find("parent").get("link"), child=j.find("child").get("link"), type=j.get("type"),
                                     xyz=np.array(list(map(float, o.get("xyz").split()))), rpy=list(map(float, o.get("rpy").split())),
                                     axis=np.array(list(map(float, a.get("xyz").split()))) if a is not None else np.zeros(3))
    for l in root.findall("link"):
        i = l.find("inertial")
        it = i.find("inertia").attrib
        I = np.array([[float(it["ixx"]), float(it["ixy"]), float(it["ixz"])], [float(it["ixy"]), float(it["iyy"]), float(it["iyz"])],
                      [float(it["ixz"]), float(it["iyz"]), float(it["izz"])]])
        links[l.get("name")] = dict(m=float(i.find("mass").get("value")), com=np.array(list(map(float, i.find("origin").get("xyz").split()))), I=I)
    return joints, links


def fk(q: dict):
    """Khung từng link (R, p; m, hệ thế giới URDF) với góc URDF q."""
    joints, _ = parse()
    T = {"left_Base": (np.eye(3), np.zeros(3))}
    pending = dict(joints)
    while pending:
        for n, j in list(pending.items()):
            if j["parent"] not in T:
                continue
            R, p = T[j["parent"]]
            Rj = R @ _rpy(*j["rpy"])
            pj = p + R @ j["xyz"]
            if j["type"] == "revolute":
                Rj = Rj @ _axang(j["axis"], q.get(n, 0.0))
            T[j["child"]] = (Rj, pj)
            del pending[n]
    return T


def derive() -> dict:
    joints, links = parse()
    T0 = fk({})
    P = {n: R_SDK @ T0[j["child"]][1] for n, j in joints.items()}   # gốc khớp ở tư thế 0 URDF, hệ SDK
    sh, el, wf, wr = P["left_shoulder_lift"], P["left_elbow_flex"], P["left_wrist_flex"], P["left_wrist_roll"]
    wy = P["left_wrist_yaw_joint"]
    up = el - sh
    off = math.degrees(math.atan2(up[2], up[0]))               # góc ngẩng cánh tay ở tư thế 0 URDF (~162,3: ngả ra sau, lên)
    fo = wf - el
    geo = dict(D0=sh[2], SHOULDER_DR=sh[0], L1=float(np.hypot(up[0], up[2])), UPPER_URDF_DEG=off,
               L2=fo[0], WF_DZ=fo[2], LW=wr[0] - wf[0], ROLL_DY=wr[1], ROLL_DZ=wr[2] - wf[2], PAN_Z=P["left_shoulder_pan"][2],
               YAW_DX=wy[0] - wf[0], YAW_DY=wy[1], YAW_DZ=wy[2] - wf[2])
    # góc URDF ứng với q_sdk = 0: vai q_sdk = -off - th_s, khuỷu q_sdk = off - th_e (trục URDF -x của hệ URDF = -y SDK)
    a = math.radians(off)
    q0 = {"left_shoulder_lift": -a, "left_elbow_flex": a, "left_gripper": grip_close_urdf()}
    T = fk(q0)
    origin = {"mount": np.zeros(3), "shoulder": np.zeros(3), "upper": np.array([geo["SHOULDER_DR"], 0, geo["D0"]])}
    origin["fore"] = origin["upper"] + [geo["L1"], 0, 0]
    origin["wrist"] = origin["fore"] + [geo["L2"], 0, geo["WF_DZ"]]
    origin["yaw"] = origin["wrist"] + [geo["YAW_DX"], geo["YAW_DY"], geo["YAW_DZ"]]
    origin["hand"] = origin["wrist"] + [geo["LW"], geo["ROLL_DY"], geo["ROLL_DZ"]]
    origin["jaw"] = R_SDK @ T["left_Moving_Jaw"][1]
    geo.update({f"JAW_PIVOT_{k}": float(v) for k, v in zip("XYZ", origin["jaw"] - origin["hand"])})
    geo["GRIP_CLOSE_URDF"] = grip_close_urdf()
    # kiểm: gốc khớp ở q_sdk = 0 phải trùng gốc body theo tham số
    chk = {"left_elbow_flex": "fore", "left_wrist_flex": "wrist", "left_wrist_yaw_joint": "yaw", "left_wrist_roll": "hand"}
    # servo yaw (chồng STS3215 y < 20 mm trong mesh link wrist_yaw, cad/upstream/.../derived/derived.json bo_servo_yaw): tâm hộp bao, khung body yaw
    dj = json.loads((URDF.parent / "derived" / "derived.json").read_text())["link"]["left_wrist_yaw"]["bo_servo_yaw"]["hop_bao_m"]
    Ry, py = T["left_wrist_yaw"]
    geo.update({f"YAW_SERVO_{k}": float(v) for k, v in zip("XYZ", R_SDK @ (py + Ry @ np.mean(dj, 0)) - origin["yaw"])})
    err = max(float(np.linalg.norm(R_SDK @ T[joints[j]["child"]][1] - origin[b])) for j, b in chk.items())
    bodies = {}
    for ln, b in LINK_BODY.items():
        R, p = T[ln]
        L = links[ln]
        Rs = R_SDK @ R
        c = R_SDK @ (p + R @ L["com"]) - origin[b]
        I = Rs @ L["I"] @ Rs.T
        acc = bodies.setdefault(b, [])
        acc.append((L["m"], c, I))
    out = {}
    for b, items in bodies.items():   # gộp nhiều link vào một body (định lý trục song song)
        m = sum(x[0] for x in items)
        c = sum(x[0] * x[1] for x in items) / m
        I = np.zeros((3, 3))
        for mi, ci, Ii in items:
            d = ci - c
            I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        out[b] = dict(mass=round(m, 5), com=[round(float(v), 5) for v in c],
                      inertia=[round(float(v), 9) for v in (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])])
    return dict(geometry={k: round(float(v), 5) for k, v in geo.items()}, check_err_m=err, bodies=out)


if __name__ == "__main__":
    print(json.dumps(derive(), indent=1))
