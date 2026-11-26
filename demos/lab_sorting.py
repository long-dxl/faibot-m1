"""Demo phòng lab: phân loại theo màu, gắp từng vật trên bàn bỏ vào ngăn khay cùng màu (đỏ, xanh lá, xanh dương). Chỉ dùng API Robot.

    ./run.sh --lab-sort                      # không cửa sổ, in JSON, mã 0 = hoàn thành
    .venv/bin/python demos/lab_sorting.py --headless --seed 3 [--limit 3]

Kiểm hoàn thành: mọi vật đã xử lý nằm trong ngăn đúng màu (tâm trong hình chữ nhật ngăn), đáy trên đáy khay (±1,5 cm), đứng yên.
"""
from __future__ import annotations

import argparse
import json
import sys

import lab_common as LC
from lab_common import L

# chỗ thả trong ngăn (dx theo chiều sâu, dy theo bề ngang ngăn), theo thứ tự vật cùng màu; vật rộng theo y (khối 5 cm) để chỗ thứ 3
SLOTS = [(-0.03, -0.025), (-0.03, 0.025), (0.03, 0.0)]   # S2: ±0,022 -> ±0,025 (vai 0,475 lệch chỗ thả ~2 mm: khối 5 cm cạnh trụ Ø4 cm đè mép trụ)
RELEASE_OPEN = 65          # 60 -> 65; khối xanh dương (kẹp ở 59) chỉ hở 1 độ khi mở 60, ở 6+1 vật dính ngón khi nhấc. Ngón vẫn không chạm vách ngăn sâu 130 mm (sim)


def cells() -> dict:
    t = L.tray()
    return {c["color_class"]: c for c in t["cells"]}


def plan(limit: int | None = None) -> list[tuple[str, str]]:
    out = [(oid, color) for oid, _, _, color in L.OBJECTS]
    return out[:limit] if limit else out


CHOREO_DEFAULT = "new"     # vòng 3: hai tay cùng gắp, đế chở sang khay; 19/20 seed, 0 lần đứng hình, jerk p95 <= 4,83 (docs/CHOREO-M1-ketqua.md)


def run(robot, limit: int | None = None, snap=None, log=print, choreo: str = CHOREO_DEFAULT) -> dict:
    """choreo: "new" = biên đạo mới (lab_choreo.sorting): mỗi chuyến hai tay gắp hai vật cùng lúc, đế chở sang khay, hai tay đặt;
    chỗ trong ngăn theo lab_choreo.SORT_TRIPS (SLOTS chỉ dùng cho vật ngoài bảng chuyến); "old" = một tay, đế đi theo vật."""
    if choreo == "new":
        import lab_choreo as CH
        done = CH.sorting(robot, plan(limit), cells(), SLOTS, RELEASE_OPEN, snap=snap, log=log)
        robot.wait(1.0)
        return check(robot, done)
    snap = snap or (lambda *a, **k: None)
    cl = cells()
    used = {c: 0 for c in cl}
    jobs = plan(limit)
    snap("1-bat-dau")
    LC.raise_to_work(robot)
    for k, (oid, color) in enumerate(jobs):
        log(f"{k + 1}/{len(jobs)} {oid} -> ngăn {color}")
        LC.pick(robot, oid, log=log)
        if k == len(jobs) // 2:
            snap("2-dang-kep")
        dx, dy = SLOTS[used[color] % len(SLOTS)]
        used[color] += 1
        cx, cy, _ = cl[color]["center"]
        LC.place(robot, oid, (cx + dx, cy + dy), drop_h=0.008, open_to=RELEASE_OPEN, log=log)
    LC.park(robot)
    robot.wait(1.0)
    snap("3-xong")
    return check(robot, jobs)


def check(robot, jobs) -> dict:
    cl = cells()
    rows = []
    for oid, color in jobs:
        c = cl[color]
        r = LC.in_zone(robot, oid, c["center"][:2], (c["size"][0] / 2, c["size"][1] / 2))
        r["z_ok"] = bool(abs(LC.obj_pos(robot, oid)[2] - (L.TABLE_H + 0.004 + LC.obj_half_h(oid))) <= 0.015)
        r["ok"] = bool(r["inside"] and r["z_ok"] and r["speed"] <= 0.02)
        r["cell"] = color
        rows.append(r)
    n = sum(r["ok"] for r in rows)
    return {"task": "sorting", "pass": n == len(rows), "done": n, "total": len(rows), "detail": rows}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--seed", type=int, default=0, help="xáo vị trí vật theo seed; -1 = bố cục gốc")
    ap.add_argument("--limit", type=int, default=None, help="chỉ phân loại n vật đầu")
    ap.add_argument("--snap", action="store_true")
    ap.add_argument("--choreo", choices=("old", "new"), default=CHOREO_DEFAULT, help="biên đạo: new (CHOREO-M1) hoặc old (trước 25/11)")
    a = ap.parse_args()
    LC.headless_env(not a.viewer)
    robot = LC.make_robot(viewer=a.viewer)
    LC.scatter(robot, None if a.seed < 0 else a.seed)
    snap = LC.Snapper(robot, "sorting", "lab_front_sort", enabled=a.snap)
    res = run(robot, a.limit, snap, log=lambda m: print(m, file=sys.stderr), choreo=a.choreo)
    res["seed"] = a.seed
    res["snapshots"] = [str(p) for p in snap.saved]
    print(json.dumps(res, ensure_ascii=False))
    robot.close()
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
