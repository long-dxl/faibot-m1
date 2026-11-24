"""Demo phòng lab: gắp đặt k = 3 vật vào vùng đích (thảm navy): 2 vật trên bàn, 1 lon lấy ra từ tủ lạnh. Chỉ dùng API Robot (IK, kẹp, đế).

    ./run.sh --lab-pick                      # không cửa sổ, in JSON, mã 0 = hoàn thành
    .venv/bin/python demos/lab_pick_place.py --headless --seed 3
    .venv/bin/python demos/lab_pick_place.py --viewer       # macOS: mjpython

Vật trên bàn: tay trái, kẹp chĩa xuống. Lon trong tủ lạnh: tay trái mở cửa (như lab_open_close), tay phải với vào bằng dụng cụ nằm ngang
(nóc tủ chặn hướng từ trên xuống), ngón đóng theo phương ngang, đặt lon đứng trên thảm.
Kiểm hoàn thành (mỗi vật): tâm trong vùng đích, đáy trên mặt bàn ±1,5 cm, |v| <= 0,02 sau 1 s chờ.
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import lab_common as LC
from lab_common import L

LIFT_FRIDGE = 0.338  # S1G: giữ trục vai 0,90 m như S1C (0,35 với trục vai 0,55 + lift)
LEVEL_X = 0.54          # giữ như S1C (kẹp AM-ARM200 ngắn hơn kẹp cũ 31,7 mm vẫn trong tầm). S1C P-L (kẹp cũ dài hơn 29 mm, vai lệch trước 58 mm): tầm dụng cụ nằm ngang ở lift 0,35, z 0,78: 0,53-0,71 m. Trước: 0,49. TCP (khung robot) khi kẹp lon; tầm với dụng cụ nằm ngang ở lift 0,40, cuộn 0, z 0,78: 0,475-0,615 m (giới hạn gập 87,5: 0,40-0,615, LEVEL_X 0,405; trước: 0,375); cẳng tay phải ở ngoài nóc tủ
LEVEL_PRE = 0.37


def slot(i):
    (zx, zy), (dx, dy) = L.ZONE["pos"], L.ZONE_SLOTS[i]
    return (zx + dx, zy + dy)


def level_grasp_dy(robot, side, half_w, margin=0.002) -> float:
    """dụng cụ nằm ngang, cuộn 0: độ lệch y (khung robot) của TCP so với tâm vật để vật tựa ngón cố định (kinematics.fixed_side_dir)."""
    from faibotic import kinematics as K, spec
    q = robot.solve_ik(side, [LEVEL_X, -LC.ARM_Y if side == "right" else LC.ARM_Y, LC.CLEAR_Z], pitch=0.0, roll=0.0)
    d = K.fixed_side_dir(q, side) if q is not None else [0.0, 1.0 if side == "left" else -1.0, 0.0]
    return float(d[1]) * (spec.TCP_DY - spec.FIXED_FACE_Y + half_w + margin)


def fetch_from_fridge(robot, oid, target_xy, snap, log) -> None:
    import lab_open_close as OC
    from faibotic import spec
    log("mở cửa tủ lạnh")
    OC.fridge_open(robot, snap, log)
    robot.home(duration=1.0)
    p = LC.obj_pos(robot, oid)
    log(f"lấy {oid} trong tủ lạnh tại {[round(float(v), 3) for v in p]}")
    side = "right"
    dy = level_grasp_dy(robot, side, LC.obj_half_w(oid))     # lon tựa ngón cố định (TCP lệch ngang khỏi tâm lon)
    bx, by, _ = robot.backend.base_pose()
    LC.goto_base(robot, bx, p[1] + LC.ARM_Y + dy)
    LC.goto_base(robot, p[0] - LEVEL_X - 0.30, p[1] + LC.ARM_Y + dy)   # đứng lùi 30 cm, đặt tay nằm ngang ngoài tủ
    robot.move_joints({"lift": LIFT_FRIDGE}, duration=1.0)
    z = p[2]
    robot.move_joints({"right_gripper": 100}, duration=0.4)
    LC.move_level_joint(robot, side, [LEVEL_X, -LC.ARM_Y, z + 0.012], 2.0)   # thân kẹp AM-ARM200 thấp hơn trục cuộn 28 mm: +12 mm để tránh kệ
    LC.goto_base(robot, p[0] - LEVEL_X, p[1] + LC.ARM_Y + dy, speed=0.12)   # đế tiến, tay đưa vào tủ ôm lon
    robot.wait(0.1)
    robot.move_joints({"right_gripper": 0}, duration=0.7)
    robot.wait(0.3)
    LC.move_tcp_level(robot, side, [LEVEL_X, -LC.ARM_Y, z + 0.025], 0.6)
    bx, by, _ = robot.backend.base_pose()
    LC.goto_base(robot, bx - 0.12, by)          # rút lon ra khỏi tủ bằng đế (tay giữ nguyên)
    snap("2b-lay-tu-lanh", "lab_fridge_in")
    bx, by, _ = robot.backend.base_pose()
    tx, ty = target_xy
    LC.goto_base(robot, bx, ty + LC.ARM_Y + dy)
    LC.goto_base(robot, tx - LEVEL_X, ty + LC.ARM_Y + dy)
    zt = L.TABLE_H + LC.obj_half_h(oid)
    LC.move_tcp_level(robot, side, [LEVEL_X, -LC.ARM_Y, zt + 0.015], 0.8)
    LC.move_tcp_level(robot, side, [LEVEL_X, -LC.ARM_Y, zt + 0.004], 0.8)
    robot.wait(0.1)
    robot.move_joints({"right_gripper": 100}, duration=0.5)
    robot.wait(0.2)
    bx, by, _ = robot.backend.base_pose()
    LC.goto_base(robot, bx - 0.10, by)          # rút tay ra bằng đế
    robot.move_joints({"right_gripper": 0}, duration=0.3)
    robot.move_joints({spec.joint_name(side, j): v for j, v in spec.HOME_ARM.items()}, duration=1.2)


def run(robot, objs=None, snap=None, log=print) -> dict:
    snap = snap or (lambda *a, **k: None)
    objs = objs or L.PICK_PLACE_OBJS
    snap("1-bat-dau")
    LC.raise_to_work(robot)
    for i, oid in enumerate(objs):
        log(f"{i + 1}/{len(objs)} {oid}")
        if oid == L.FRIDGE_OBJ[0]:
            fetch_from_fridge(robot, oid, slot(i), snap, log)
            LC.raise_to_work(robot)
            continue
        LC.pick(robot, oid, log=log)
        if i == 0:
            snap("2-dang-kep")
        LC.place(robot, oid, slot(i), log=log)
    LC.park(robot)
    robot.wait(1.0)
    snap("3-xong")
    return check(robot, objs)


def check(robot, objs) -> dict:
    (zx, zy), (lx, ly) = L.ZONE["pos"], L.ZONE["size"]
    rows = [LC.in_zone(robot, oid, (zx, zy), (lx / 2, ly / 2)) for oid in objs]
    n = sum(r["ok"] for r in rows)
    return {"task": "pick_place", "k": len(objs), "pass": n == len(objs), "done": n, "total": len(objs), "detail": rows}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true", help="không mở cửa sổ (mặc định khi không có --viewer)")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--seed", type=int, default=0, help="xáo vị trí vật trên bàn theo seed (0 = bố cục trong lab-scene.json)")
    ap.add_argument("--objects", default=",".join(L.PICK_PLACE_OBJS), help="danh sách vật, cách nhau dấu phẩy")
    ap.add_argument("--snap", action="store_true", help="chụp khung vào media/sim-snapshots/lab/")
    a = ap.parse_args()
    LC.headless_env(not a.viewer)
    robot = LC.make_robot(viewer=a.viewer)
    LC.scatter(robot, a.seed)
    snap = LC.Snapper(robot, "pick-place", "lab_front", enabled=a.snap)
    res = run(robot, a.objects.split(","), snap, log=lambda m: print(m, file=sys.stderr))
    res["seed"] = a.seed
    res["snapshots"] = [str(p) for p in snap.saved]
    print(json.dumps(res, ensure_ascii=False))
    robot.close()
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
