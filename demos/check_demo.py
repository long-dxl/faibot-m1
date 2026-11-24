"""Nghiệm thu demo bằng toạ độ (không bằng mắt): in JSON, thoát mã 0 nếu đạt.

Tiêu chí (SPEC F016, Sửa sau QC0, M1):
- tâm hộp cuối cùng trong bán kính 3 cm quanh đích (x, y) và nằm trên mặt bàn đích (độ cao ±1 cm);
- 0 contact sai: thân robot (mọi geom không phải ngón kẹp) chạm bất cứ thứ gì ngoài chính nó ở khớp kề,
  ngón kẹp chạm bàn/sàn/tường, tay chạm tay, hộp chạm sàn;
- 0 NaN trong qpos/qvel suốt kịch bản.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))
sys.path.insert(0, str(ROOT / "demos"))

from demo_pick_place import run_pick_place  # noqa: E402
from faibotic import Robot  # noqa: E402
from faibotic import model as scene  # noqa: E402

TOL_XY = 0.03
TOL_Z = 0.01
WORLD = {"floor", "table_top", "wall_back", "wall_side"} | {f"table_leg{i}" for i in range(4)}


def is_finger(g: str) -> bool:
    return "_finger_" in g


def classify(g1: str, g2: str) -> str | None:
    """Trả về lý do nếu là contact sai, None nếu được phép."""
    pair = {g1, g2}
    if "box_geom" in pair:
        other = (pair - {"box_geom"}).pop() if len(pair) > 1 else "box_geom"
        if other == "table_top" or is_finger(other):
            return None
        return f"hộp chạm {other}"
    robot = [g for g in pair if g not in WORLD]
    if not robot:
        return None  # thế giới với thế giới
    if len(robot) == 2:
        return "robot tự va chạm"
    return "robot chạm môi trường"


def main() -> int:
    robot = Robot(backend="sim")
    b = robot.backend
    stats = {"steps": 0, "nan_steps": 0, "bad": {}, "max_box_speed_after_place": 0.0}

    def on_step(r):
        stats["steps"] += 1
        if not (np.all(np.isfinite(b.data.qpos)) and np.all(np.isfinite(b.data.qvel))):
            stats["nan_steps"] += 1
        for g1, g2 in b.contacts():
            why = classify(g1, g2)
            if why:
                k = f"{why}: {g1} / {g2}"
                stats["bad"][k] = stats["bad"].get(k, 0) + 1

    run_pick_place(robot, log=lambda *_: None, on_step=on_step)
    box = b.body_pos("box")
    target = np.array(scene.BOX_TARGET)
    dxy = float(np.linalg.norm(box[:2] - target))
    z_expected = scene.TABLE_HEIGHT + scene.BOX_HALF[2]
    dz = float(box[2] - z_expected)
    on_table = abs(dz) <= TOL_Z
    res = {
        "box_final": [round(float(v), 4) for v in box],
        "target_xy": list(scene.BOX_TARGET),
        "dist_xy_m": round(dxy, 4),
        "tol_xy_m": TOL_XY,
        "box_z_error_m": round(dz, 4),
        "on_target_table": bool(on_table),
        "bad_contact_steps": int(sum(stats["bad"].values())),
        "bad_contacts": stats["bad"],
        "nan_steps": stats["nan_steps"],
        "control_steps": stats["steps"],
        "sim_time_s": round(float(b.data.time), 2),
    }
    res["pass"] = bool(dxy <= TOL_XY and on_table and res["bad_contact_steps"] == 0 and stats["nan_steps"] == 0)
    print(json.dumps(res, ensure_ascii=False, indent=2))
    robot.close()
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
