# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Tay AM-ARM200 follower nguyên bản (cấu hình P-L, 23/11/2026): động học thuận theo URDF upstream (cad/upstream/AM-ARM/am-arm200/urdf/am-arm200.urdf,
commit 5b03af1, Apache-2.0, chỉ đọc), khối lượng/tâm khối lượng link theo URDF, và gán 14 STEP follower vào link để dựng tư thế.
 Chỉ numpy (+ build123d khi dựng hình).

Hệ của tay = khung left_Base URDF (mét; trục pan dọc -z của khung, gốc ở mặt lắp đế tay). Tay phải AlohaMini2 dùng đúng bộ chi tiết như tay trái
(URDF AlohaMini2: right_* cùng mesh, cùng khớp, chỉ dời x), nên tay phải = tay trái dời, không lật gương.
Khớp (rad, giá trị URDF): pan, lift, elbow, wflex, yaw (khoá 0 ở 5+1), roll, grip."""
import math, os, re
import xml.etree.ElementTree as ET
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
UPA = os.path.join(HERE, "..", "upstream", "AM-ARM", "am-arm200")
URDF = os.path.join(UPA, "urdf", "am-arm200.urdf")
STP = os.path.join(UPA, "stp", "follower")
JOINTS = ("left_shoulder_pan", "left_shoulder_lift", "left_elbow_flex", "left_wrist_flex", "left_wrist_yaw_joint", "left_wrist_roll", "left_gripper")
SHORT = ("pan", "lift", "elbow", "wflex", "yaw", "roll", "grip")
IGNORE_MASS = ("left_camera",)          # khối lượng link camera trong URDF (1,27 kg) là số giữ chỗ (R1b mục 2.2)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axang(ax, a):
    ax = np.asarray(ax, float); ax = ax / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


_U = {}
def urdf():
    if not _U:
        r = ET.parse(URDF).getroot()
        J = []
        for j in r.findall("joint"):
            o = j.find("origin"); a = j.find("axis")
            J.append(dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                          xyz=np.array(list(map(float, o.get("xyz").split()))), rpy=np.array(list(map(float, o.get("rpy").split()))),
                          axis=np.array(list(map(float, a.get("xyz").split()))) if a is not None else None))
        M = {}
        for l in r.findall("link"):
            i = l.find("inertial")
            if i is None or i.find("mass") is None: continue
            M[l.get("name")] = (float(i.find("mass").get("value")), np.array(list(map(float, i.find("origin").get("xyz").split()))))
        _U.update(J=J, M=M)
    return _U


def fk(q=None):
    """q: dict tên ngắn -> rad. Trả {link: ma trận 4x4 (mét) trong khung left_Base}."""
    q = dict(q or {}); T = {"left_Base": np.eye(4)}
    for j in urdf()["J"]:
        A = np.eye(4); A[:3, :3] = _rpy(*j["rpy"]); A[:3, 3] = j["xyz"]
        if j["type"] == "revolute":
            k = SHORT[JOINTS.index(j["name"])] if j["name"] in JOINTS else None
            R = np.eye(4); R[:3, :3] = _axang(j["axis"], q.get(k, 0.0)); A = A @ R
        T[j["child"]] = T[j["parent"]] @ A
    return T


def mass_com(q=None):
    """Khối lượng (kg) và tâm khối lượng (m, khung left_Base) của một tay, bỏ link camera giữ chỗ."""
    T = fk(q); m = 0.0; c = np.zeros(3)
    for n, (mi, ci) in urdf()["M"].items():
        if n in IGNORE_MASS: continue
        m += mi; c += mi * (T[n][:3, :3] @ ci + T[n][:3, 3])
    return m, c / m


def tcp(q=None, d=0.10):
    """Điểm đầu kẹp gần đúng (m, khung left_Base): gốc khung left_Fixed_Jaw + d dọc trục cuộn (+z khung đó, hướng ngón) [giả định d 0,10 m;
    URDF chưa có khung TCP; tâm kẹp đo trên CAD ở gripper_am.py]."""
    T = fk(q)["left_Fixed_Jaw"]
    return T[:3, 3] + T[:3, :3] @ np.array([0.0, 0.0, d])


# Tư thế (độ, giá trị khớp URDF). 0 = tay gập (vai ngửa ra sau, cẳng tay quay lại trước), là tư thế nghỉ của AM-ARM200.
# straight: cánh tay + cẳng tay + kẹp nằm ngang ra trước (tìm số: mỗi đoạn lệch z <= 2 mm). ready/down/high: tư thế làm việc để kiểm va chạm.
POSE_DEG = {
    "rest": dict(),                                              # gập: đầu kẹp 0,22 m trước, 0,26 m trên gốc tay
    "straight": dict(lift=-162.5, elbow=163.5, wflex=1.0),       # duỗi ngang: trục cuộn ngang, đầu kẹp 0,580 m trước trục pan (R1b 0,584)
    "ready": dict(lift=-120.0, elbow=120.0, wflex=40.0),         # đầu kẹp 0,49 m trước, chúi 50°
    "down": dict(lift=-150.0, elbow=100.0, wflex=30.0),          # với xuống: đầu kẹp 0,33 m trước, 0,17 m dưới gốc tay
    "up": dict(lift=-60.0, elbow=60.0, wflex=0.0),               # nâng cao: khuỷu 0,28 m trên gốc tay
}


def pose(name, pan=0.0):
    q = {k: math.radians(v) for k, v in POSE_DEG[name].items()}; q["pan"] = math.radians(pan); return q


# ---------------- STEP follower -> link ----------------
# Quan hệ hệ STEP follower và URDF: STEP = URDF(mm) + t (chỉ tịnh tiến; cùng cách assembly_full.wrist_ref, tests/test_wrist_upstream.py).
PART_LINK = {   # gán theo vị trí trong tư thế 0 (kiểm bằng link_of() trong part_links()); WaveShare và chốt không ổ bi không lắp
    "O_F_J1_Base": "left_Base", "O_F_J2_Rotation": "left_Rotation_Pitch", "OB_F_J3_Shoulder": "left_Upper_Arm", "OB_F_J4_Elbow": "left_Lower_Arm",
    "OB_F_J5_Elbow_Wrist_Bracket": "left_Wrist_Pitch_Roll", "OB_F_J5_Pin_Bearing": "left_Wrist_Pitch_Roll", "OB_F_J56_Wrist_Housing": "left_wrist_yaw",
    "OB_F_J56_Bottom": "left_wrist_yaw", "OB_F_J56_Top": "left_wrist_yaw", "OB_F_J7_Gripper": "left_Fixed_Jaw", "OB_F_J7_Jaw": "left_Moving_Jaw",
    "OB_F_Camera": "left_Fixed_Jaw",
}
