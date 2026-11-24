"""Test tự chơi (S1, cổng "test tự chơi"): gắp đặt trên bàn trong phòng lab MuJoCo bằng chính sách IK viết sẵn, nhiều seed, không cần render.

Mỗi seed: xáo vị trí/hướng vật (lab_common.scatter: ±1 cm, ±0,2 rad), gắp lần lượt các vật trên bàn của bài pick & place (khối đỏ, trụ xanh),
nhấc 5 cm, giữ 2 s, đo trượt, rồi đặt vào vùng đích. Seed đạt khi MỌI vật:
  - nằm trong vùng đích, đáy trên mặt bàn ±1,5 cm, đứng yên (lab_pick_place.check);
  - trượt <= SLIP_MAX_M so với TCP từ lúc nhấc 5 cm tới hết 2 s giữ (kẹp giữ bằng ma sát, không weld);
và bước khớp tay thực tế mỗi khung điều khiển (0,02 s) <= STEP_MAX_DEG trong cả seed.
Lon trong tủ lạnh (vật thứ 3 của bài lab) KHÔNG nằm trong test này: cần mở cửa tủ.

    .venv/bin/python demos/selfplay_pick_place.py --seeds 20          # in JSON, mã 0 khi >= 18/20 đạt
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np

import lab_common as LC
import lab_pick_place as PP
from lab_common import L

SLIP_MAX_M = 0.002
STEP_MAX_DEG = 2.0
LIFT_TEST_M = 0.05
HOLD_S = 2.0
TABLE_OBJS = [o for o in L.PICK_PLACE_OBJS if o != L.FRIDGE_OBJ[0]]
PASS_MIN = 18


def _hand_frame(robot, side=LC.SIDE):
    d = robot.backend.data
    b = d.body(f"{side}_hand")
    return b.xpos.copy(), b.xmat.reshape(3, 3).copy()


def _obj_in_hand(robot, oid, side=LC.SIDE):
    p, R = _hand_frame(robot, side)
    return R.T @ (LC.obj_pos(robot, oid) - p)


class StepMonitor:
    """Ghi bước lớn nhất của các khớp tay (qpos thật) giữa hai khung điều khiển."""

    def __init__(self, robot):
        from faibotic import spec
        self.robot = robot
        b = robot.backend
        self.adr = [b.model.joint(spec.joint_name(s, j)).qposadr[0] for s in spec.SIDES for j in spec.ARM_JOINTS]
        self.prev = b.data.qpos[self.adr].copy()
        self.max_deg = 0.0
        self.nan = False
        self._step = b.step

        def step(dt):
            self._step(dt)
            q = b.data.qpos[self.adr]
            if not np.all(np.isfinite(b.data.qpos)):
                self.nan = True
            self.max_deg = max(self.max_deg, float(np.degrees(np.max(np.abs(q - self.prev)))))
            self.prev = q.copy()
        b.step = step


def pick_lift_hold(robot, oid, log) -> dict:
    """Như lab_common.pick nhưng nhấc 5 cm rồi giữ 2 s để đo trượt."""
    p = LC.obj_pos(robot, oid)
    gy = LC.obj_yaw(robot, oid)
    LC.approach_y(robot, p[1])
    z = LC.grasp_z(oid)
    LC.move_tcp_world(robot, [p[0], p[1], LC.CLEAR_Z], 1.2, gy)
    g = LC.grasp_world(robot, [p[0], p[1], z], LC.obj_half_w(oid), gy)     # vật tựa ngón cố định
    LC.move_tcp_world(robot, [g[0], g[1], LC.CLEAR_Z], 0.4, gy)
    LC.gripper(robot, 100, 0.5)
    LC.move_tcp_world(robot, g, 1.0, gy)
    robot.wait(0.15)
    LC.gripper(robot, 0, 0.7)
    robot.wait(0.3)
    LC.move_tcp_world(robot, [g[0], g[1], z + LIFT_TEST_M], 0.6, gy)
    r0 = _obj_in_hand(robot, oid)
    h0 = LC.obj_pos(robot, oid)[2]
    robot.wait(HOLD_S)
    r1 = _obj_in_hand(robot, oid)
    lifted = LC.obj_pos(robot, oid)[2] - (L.TABLE_H + LC.obj_half_h(oid))
    slip = float(np.linalg.norm(r1 - r0))
    LC.move_tcp_world(robot, [g[0], g[1], LC.CLEAR_Z], 0.8, gy)
    log(f"  {oid}: nhấc {lifted * 1000:.1f} mm, trượt {slip * 1000:.2f} mm trong {HOLD_S} s")
    return {"id": oid, "slip_mm": round(slip * 1000, 3), "lifted_mm": round(float(lifted) * 1000, 1), "z_at_lift": round(float(h0), 4)}


def run_seed(seed: int, log=lambda m: None) -> dict:
    robot = LC.make_robot()
    LC.scatter(robot, seed)
    mon = StepMonitor(robot)
    grips = []
    try:
        LC.raise_to_work(robot)
        for i, oid in enumerate(TABLE_OBJS):
            grips.append(pick_lift_hold(robot, oid, log))
            LC.place(robot, oid, PP.slot(i), log=log)
        LC.park(robot)
        robot.wait(1.0)
        res = PP.check(robot, TABLE_OBJS)
        err = None
    except ValueError as e:      # ngoài tầm với: tính là trượt seed, không giấu
        res, err = {"pass": False, "done": 0, "total": len(TABLE_OBJS), "detail": []}, str(e)
    finally:
        robot.close()
    slip_ok = len(grips) == len(TABLE_OBJS) and all(g["slip_mm"] <= SLIP_MAX_M * 1000 and g["lifted_mm"] >= 40 for g in grips)
    step_ok = mon.max_deg <= STEP_MAX_DEG
    ok = bool(res["pass"] and slip_ok and step_ok and not mon.nan)
    return {"seed": seed, "ok": ok, "placed": f'{res["done"]}/{res["total"]}', "slip_ok": slip_ok, "grips": grips,
            "max_joint_step_deg": round(mon.max_deg, 3), "step_ok": step_ok, "nan": mon.nan, "error": err}


def run(seeds, log=lambda m: None) -> dict:
    rows = [run_seed(s, log) for s in seeds]
    n = sum(r["ok"] for r in rows)
    slips = [g["slip_mm"] for r in rows for g in r["grips"]]
    return {"task": "selfplay_pick_place", "objects": TABLE_OBJS, "seeds": list(seeds), "pass_count": n, "total": len(rows),
            "pass": n >= PASS_MIN * len(rows) / 20, "slip_max_mm": max(slips) if slips else None,
            "max_joint_step_deg": max(r["max_joint_step_deg"] for r in rows), "rows": rows}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--start", type=int, default=1)
    a = ap.parse_args()
    res = run(range(a.start, a.start + a.seeds), log=lambda m: print(m, file=sys.stderr))
    print(json.dumps(res, ensure_ascii=False))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
