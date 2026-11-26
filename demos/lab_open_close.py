"""Demo phòng lab: mở rồi đóng ngăn kéo trên, nắp hộp, cửa tủ lạnh. Chỉ dùng API Robot (IK, kẹp, đế).

    ./run.sh --lab-open                      # không cửa sổ, in JSON, mã 0 = hoàn thành
    .venv/bin/python demos/lab_open_close.py --headless [--only drawer,lid,fridge]

Cách làm: kẹp chĩa xuống ôm tay nắm (thanh ngang của ngăn kéo, núm cầu của nắp, thanh đứng của cửa tủ lạnh), tay giữ TCP cố định
trong khung robot còn đế omni đi theo cung/đường của tay nắm (quỹ đạo dài hơn tầm với của tay). Nắp hộp: trục đóng ngón song song
trục bản lề để núm cầu xoay được giữa hai ngón. Cửa tủ lạnh: trục đóng ngón xoay theo cửa (cuộn cổ tay).
Kiểm hoàn thành bằng khớp, sau khi nhả kẹp và chờ 1 s: mở khi góc/khoảng trượt >= open_min, đóng khi <= closed_max (lab-scene.json).
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np

import lab_common as LC
from lab_common import L

LIFT_HIGH = 0.45   # S1G: đỉnh hành trình P-L (trục vai 1,012 m)
# (cổ tay theo URDF upstream): khi kẹp chĩa xuống, trục gập nằm trên tâm kẹp 0,19 m và lùi ra ngoài 3 cm, nên với ngang ở gần độ cao vai chỉ còn ~0,28-0,30 m
# (trước 0,33 m). Đế tiến thêm 6 cm về phía tủ/hộp so với WORK_X (thân đế thấp hơn mặt bàn 0,56 m, chui dưới mép bàn; chân bàn ở y ±0,57).
REACH_IN = 0.06
# (giới hạn gập 87,5, bảng ghép theo cuộn): ở lift 0,50 kẹp chĩa xuống tại z 0,93-0,97 chỉ với được x 0,31-0,37 (robot) và không tới z 1,0:
# ngăn kéo: đế tiến thêm 5 cm (DRAWER_IN) và hạ điểm chờ trên tay nắm 7 -> 4,5 cm (3 cm: ngón mở chạm tay nắm, kéo chỉ được 0,066 m; thử 0,10/0,04 và 0,12/0,05 đều đạt).
# S1C (P-L): tầm kẹp chĩa xuống ở z ~0,92 là x 0,375-0,43 (robot): DRAWER_IN 0,11 -> 0,05; mở kẹp 40% (khe 34 mm) vì kẹp cũ mở hết 84 mm thì ngón sau chạm mặt ngăn (tay nắm cách mặt 40 mm)
DRAWER_IN, DRAWER_ABOVE, DRAWER_OPEN = 0.05, 0.03, 40
DRAWER_OPEN_PUSH = 30   # lúc đẩy giữ ngón cố định phía mặt ngăn như lúc kéo, hàm động (phía robot) mở 30% (đóng ngăn 0,000; đặt ngón cố định phía robot thì ngón tì mặt ngăn, còn hở 0,027)
# Nắp hộp: núm lên tới z 0,974 (góc 1,2-1,4 rad); ở lift 0,50 kẹp chĩa xuống tới z 0,986 khi x 0,32: đế tiến thêm 2 cm (LID_IN), nhả/kẹp lại ở góc mở chỉ nhích 1 cm.
LID_IN, LID_UP_OPEN = 0.04, 0.01   # S1C P-L: LID_IN 0,08 -> 0,02 (núm ở x ~0,39 khung robot); S2 (vai 0,475): 0,02 -> 0,04
FRIDGE_BACK = 0.36   # 24/11: 0,40 -> 0,36: trần gập 70, tay nắm tủ lạnh (z 0,775 + 0,07) ở x 0,40 khung robot ngoài tầm kẹp chĩa xuống (tới z 0,845), ở x 0,36 tới 0,87
# 24/11 (trần gập 70): kẹp chĩa thẳng xuống ở lift 0,45 chỉ tới z ~0,885 (x <= 0,33), thấp hơn tay nắm ngăn trên + 3 cm và núm nắp khi mở.
# Ngăn kéo và nắp hộp dùng dụng cụ ngẩng 5 độ (pitch -85; sát đỉnh tầm thêm 5 độ): tới z ~0,90 ở x 0,30-0,33. Đường đi nội suy khớp tới nghiệm IK có pitch.
LID_ABOVE = 0.04   # 24/11: 0,06 -> 0,04 (điểm chờ trên núm, nhấc sau khi đóng)
HIGH_PITCH = math.radians(-85)
LID_DY = 0.07      # 24/11: 0,03 -> 0,07 (trần gập 70, dụng cụ ngẩng 5 độ: lệch 7 cm mở 1,51 rad, 3 cm chỉ 0,37). S2: đặt núm lệch vào trong trục pan tay trái 3 cm (vai rộng: núm đúng trục pan chỉ mở được 1,14 rad < 1,2; lệch 3 cm: 1,53)


def _joint_spec(item_fn, jname):
    for ln in item_fn()["links"]:
        j = ln.get("joint") or {}
        if j.get("name") == jname:
            return j
    raise KeyError(jname)


def settle_q(robot, jname) -> float:
    robot.wait(1.0)
    return LC.joint_q(robot, jname)


# nửa bề rộng chỗ kẹp theo trục đóng (vật tựa ngón cố định của kẹp AM-ARM200): thanh tay nắm ngăn kéo r 6, núm nắp cầu r 12,
# tay nắm tủ lạnh r 8 mm (sim/lab/build_lab.py). Ngăn kéo: ngón cố định phía mặt ngăn (kéo bằng ngón cố định); để hàm động kéo thì thanh
# trượt khỏi kẹp (mở 0,066 < 0,08).
HW_DRAWER, HW_KNOB, HW_FRIDGE = 0.006, 0.012, 0.008

_PITCH = [None]   # None: kẹp chĩa thẳng xuống (move_tcp của SDK); số: pitch (rad) cho solve_ik


def _q_pitch(robot, p_robot, grip_yaw):
    from faibotic import spec
    q = None
    for dp in (0.0, 5.0, -5.0):     # gần đỉnh tầm thì ngẩng thêm 5 độ (đổi dần theo đường, bước khớp nhỏ)
        q = robot.solve_ik(LC.SIDE, p_robot, pitch=_PITCH[0] + math.radians(dp), grip_yaw=grip_yaw)
        if q is not None:
            break
    if q is None:
        raise ValueError(f"tay {LC.SIDE}: điểm {list(np.round(p_robot, 3))} ngoài tầm với (pitch {math.degrees(_PITCH[0]):.0f})")
    return {spec.joint_name(LC.SIDE, j): float(v) for j, v in zip(spec.ARM_JOINTS, q)}


def move_tcp_w(robot, p, duration, grip_yaw):
    """Như LC.move_tcp_world; khi đặt _PITCH thì nội suy khớp tới nghiệm IK có pitch (move_joints vẫn kiểm vùng cấm)."""
    if _PITCH[0] is None:
        return LC.move_tcp_world(robot, p, duration, grip_yaw)
    robot.move_joints(_q_pitch(robot, robot.world_to_robot(p), grip_yaw), duration=duration)


def follow(robot, path, duration):
    """LC.follow_with_base; khi đặt _PITCH thì mỗi chu kỳ đặt khớp theo IK có pitch thay cho set_tcp."""
    if _PITCH[0] is None:
        return LC.follow_with_base(robot, path, duration)
    from faibotic import spec
    m = 200
    xy = np.array([np.asarray(path(0.5 - 0.5 * math.cos(math.pi * i / m))[0][:2], float) for i in range(m + 1)])
    v = np.diff(xy, axis=0) * m
    a = np.diff(v, axis=0) * m
    vmax, amax = float(np.max(np.linalg.norm(v, axis=1))), float(np.max(np.linalg.norm(a, axis=1), initial=0.0))
    duration = max(duration, 1.1 * vmax / spec.BASE_MAX_V, 1.1 * math.sqrt(amax / spec.BASE_MAX_ACC))
    n = max(2, int(round(duration / robot.dt)))
    pr = robot.world_to_robot(path(0.0)[0])
    prev = np.asarray(path(0.0)[0][:2], float) - pr[:2]
    for i in range(1, n + 1):
        s = 0.5 - 0.5 * math.cos(math.pi * i / n)
        w, yaw = path(s)
        g = np.asarray(w[:2], float) - pr[:2]
        vb = (g - prev) / robot.dt
        prev = g
        robot.set_base_velocity(vb[0], vb[1], 0.0)
        robot.set_joint_positions(_q_pitch(robot, [pr[0], pr[1], w[2]], yaw))
        robot.step()
    robot.set_base_velocity(0, 0, 0)


def grasp_at(robot, p, grip_yaw, above=0.07, open_to=100, half_w=0.0, fixed_dir=None):
    """Kẹp tại p (thế giới). Trả độ lệch TCP so với p (thế giới) để các đường bám theo (follow) cộng vào, xoay theo đổi grip_yaw (shifted).
    fixed_dir (xy thế giới): muốn ngón cố định nằm phía này của vật; nếu nghiệm cuộn hiện tại cho phía ngược lại thì cuộn cổ tay thêm 180 độ."""
    move_tcp_w(robot, [p[0], p[1], p[2] + above], 1.0, grip_yaw)
    g = LC.grasp_world(robot, p, half_w, grip_yaw)
    d = g - np.asarray(p, float)
    if fixed_dir is not None and float(np.dot(d[:2], fixed_dir)) < 0:
        from faibotic import spec
        jn = spec.joint_name(LC.SIDE, "wrist_roll"); r = robot._targets[jn]
        lo, hi = spec.ARM_LIMITS["wrist_roll"]
        r2 = r + math.pi if r + math.pi <= hi else r - math.pi
        if lo <= r2 <= hi:
            robot.move_joints({jn: r2}, duration=1.0)
            move_tcp_w(robot, [p[0], p[1], p[2] + above], 0.6, grip_yaw)
            g = LC.grasp_world(robot, p, half_w, grip_yaw)
            d = g - np.asarray(p, float)
    move_tcp_w(robot, [g[0], g[1], g[2] + above], 0.4, grip_yaw)
    LC.gripper(robot, open_to, 0.4)
    move_tcp_w(robot, g, 0.8, grip_yaw)
    robot.wait(0.1)
    LC.gripper(robot, 0, 0.6)
    robot.wait(0.3)
    return d


def shifted(path, d, yaw0):
    """Đường TCP = đường điểm kẹp + độ lệch d (xoay theo grip_yaw so với yaw0)."""
    def f(s):
        w, yaw = path(s); c, sn = math.cos(yaw - yaw0), math.sin(yaw - yaw0)
        return np.asarray(w, float) + [c * d[0] - sn * d[1], sn * d[0] + c * d[1], d[2]], yaw
    return f


def release_up(robot, grip_yaw, up=0.07):
    LC.gripper(robot, 100, 0.4)
    p = robot.tcp_position(LC.SIDE, source="target")
    if _PITCH[0] is None:
        robot.move_tcp(LC.SIDE, [p[0], p[1], p[2] + up], duration=0.6, grip_yaw=grip_yaw)
    else:
        robot.move_joints(_q_pitch(robot, [p[0], p[1], p[2] + up], grip_yaw), duration=0.6)


# ---------------- ngăn kéo ----------------
def drawer(robot, snap, log, i=1) -> dict:
    jn = f"drawer_{i}_slide"
    js = _joint_spec(L.cabinet, jn)
    h = LC.site_world(robot, f"drawer_{i}_handle")
    log(f"ngăn kéo {i}: tay nắm {np.round(h, 3).tolist()}")
    LC.goto_base(robot, LC.WORK_X + DRAWER_IN, h[1] - LC.ARM_Y)
    robot.move_joints({"lift": LIFT_HIGH}, duration=1.0)
    d = grasp_at(robot, h, 0.0, above=DRAWER_ABOVE, open_to=DRAWER_OPEN, half_w=HW_DRAWER, fixed_dir=(1.0, 0.0))
    pull = 0.12
    follow(robot, shifted(lambda s: (h + [-pull * s, 0, 0], 0.0), d, 0.0), 2.5)
    release_up(robot, 0.0, up=DRAWER_ABOVE)
    q_open = settle_q(robot, jn)
    snap("ngan-keo-mo", "lab_cabinet")
    h2 = LC.site_world(robot, f"drawer_{i}_handle")
    # đặt lại đế theo tay nắm (đế đi vòng hở trôi ~1-2 cm sau khi kéo), tay nắm ở x 0,34 khung robot như lần kẹp đầu (0,39)
    LC.goto_base(robot, h2[0] - (h[0] - LC.WORK_X - DRAWER_IN), h2[1] - LC.ARM_Y)
    d = grasp_at(robot, h2, 0.0, above=DRAWER_ABOVE, open_to=DRAWER_OPEN_PUSH, half_w=HW_DRAWER, fixed_dir=(1.0, 0.0))
    push = (h[0] + 0.004) - h2[0]
    follow(robot, shifted(lambda s: (h2 + [push * s, 0, 0], 0.0), d, 0.0), 2.5)
    release_up(robot, 0.0, up=DRAWER_ABOVE)
    LC.goto_base(robot, LC.WORK_X - 0.05, robot.backend.base_pose()[1])
    q_closed = settle_q(robot, jn)
    return {"joint": jn, "open_q": round(q_open, 4), "closed_q": round(q_closed, 4),
            "open_ok": q_open >= js["open_min"], "closed_ok": q_closed <= js["closed_max"]}


# ---------------- nắp hộp ----------------
def lid(robot, snap, log) -> dict:
    jn = "lid_hinge"
    js = _joint_spec(L.lid_box, jn)
    lx, ly, lz = L.BOX["size"]
    cx, cy = L.BOX["center"]
    hinge = np.array([cx, cy + ly / 2, L.TABLE_H + lz])
    k0 = np.array([0.0, -L.BOX["knob_dy"], L.BOX["lid_t"] + 0.03])       # núm so với bản lề, nắp đóng

    def knob(th):
        y = k0[1] * math.cos(th) + k0[2] * math.sin(th)
        z = -k0[1] * math.sin(th) + k0[2] * math.cos(th)
        return hinge + [0.0, y, z]

    def arc(a, b):
        return lambda s: (knob(a + (b - a) * s), 0.0)

    p = LC.site_world(robot, "lid_box_lid_knob")
    log(f"nắp hộp: núm {np.round(p, 3).tolist()}")
    LC.goto_base(robot, LC.WORK_X + LID_IN, p[1] - (LC.ARM_Y - LID_DY))
    robot.move_joints({"lift": LIFT_HIGH}, duration=1.0)
    d = grasp_at(robot, p, 0.0, above=LID_ABOVE, half_w=HW_KNOB)
    th1 = 1.70
    follow(robot, shifted(arc(0.0, th1), d, 0.0), 3.0)
    release_up(robot, 0.0, up=LID_UP_OPEN)
    q_open = settle_q(robot, jn)
    snap("nap-mo", "lab_front_sort")
    p2 = LC.site_world(robot, "lid_box_lid_knob")
    d = grasp_at(robot, p2, 0.0, above=LID_UP_OPEN, half_w=HW_KNOB)
    follow(robot, shifted(arc(q_open, 0.12), d, 0.0), 3.0)
    release_up(robot, 0.0, up=LID_ABOVE)
    q_closed = settle_q(robot, jn)
    return {"joint": jn, "open_q": round(q_open, 4), "closed_q": round(q_closed, 4),
            "open_ok": q_open >= js["open_min"], "closed_ok": q_closed <= js["closed_max"]}


# ---------------- cửa tủ lạnh ----------------
def _fridge_geom(robot):
    p = LC.site_world(robot, "fridge_door_handle")
    hinge = np.array([L.FRIDGE["front_x"] - L.FRIDGE["door_t"] / 2, L.FRIDGE["center_y"] - L.FRIDGE["size"][1] / 2 + 0.005, 0.0])
    th0 = LC.joint_q(robot, "fridge_hinge")
    c, s = math.cos(-th0), math.sin(-th0)
    d = p[:2] - hinge[:2]
    rel = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])      # tay nắm so với bản lề khi cửa đóng

    def handle(th):
        c, s = math.cos(th), math.sin(th)
        return np.array([hinge[0] + c * rel[0] - s * rel[1], hinge[1] + s * rel[0] + c * rel[1], p[2]])

    def arc(a, b):
        return lambda s: (handle(a + (b - a) * s), math.pi / 2 + a + (b - a) * s)
    return handle, arc


def fridge_open(robot, snap, log, th1: float = 1.45) -> float:
    """Mở cửa tủ lạnh bằng tay trái, nhả kẹp, trả góc cửa sau 1 s."""
    handle, arc = _fridge_geom(robot)
    p = handle(0.0)
    log(f"tủ lạnh: tay nắm {np.round(p, 3).tolist()}")
    bx, by, _ = robot.backend.base_pose()
    LC.goto_base(robot, p[0] - FRIDGE_BACK, by)                 # lùi khỏi bàn trước khi đi ngang
    LC.goto_base(robot, p[0] - FRIDGE_BACK, p[1] - LC.ARM_Y)
    robot.move_joints({"lift": LIFT_HIGH}, duration=1.0)
    d = grasp_at(robot, p, math.pi / 2, half_w=HW_FRIDGE)
    LC.follow_with_base(robot, shifted(arc(0.0, th1), d, math.pi / 2), 5.0)
    snap("2-dang-kep", "lab_fridge")
    release_up(robot, math.pi / 2 + th1, up=0.06)
    return settle_q(robot, "fridge_hinge")


def fridge_close(robot, log) -> float:
    handle, arc = _fridge_geom(robot)
    th = LC.joint_q(robot, "fridge_hinge")
    p = handle(th)
    bx, by, _ = robot.backend.base_pose()
    LC.goto_base(robot, p[0] - FRIDGE_BACK, p[1] - LC.ARM_Y)
    robot.move_joints({"lift": LIFT_HIGH}, duration=1.0)
    d = grasp_at(robot, p, math.pi / 2 + th, half_w=HW_FRIDGE)
    LC.follow_with_base(robot, shifted(arc(th, -0.02), d, math.pi / 2 + th), 5.0)
    release_up(robot, math.pi / 2, up=0.06)
    bx, by, _ = robot.backend.base_pose()
    LC.goto_base(robot, bx - 0.1, by)
    return settle_q(robot, "fridge_hinge")


def fridge(robot, snap, log) -> dict:
    jn = "fridge_hinge"
    js = _joint_spec(L.fridge, jn)
    q_open = fridge_open(robot, snap, log)
    snap("tu-lanh-mo", "lab_fridge")
    q_closed = fridge_close(robot, log)
    return {"joint": jn, "open_q": round(q_open, 4), "closed_q": round(q_closed, 4),
            "open_ok": q_open >= js["open_min"], "closed_ok": q_closed <= js["closed_max"]}


def _pitched(fn):
    def run_task(robot, snap, log):
        _PITCH[0] = HIGH_PITCH
        try:
            return fn(robot, snap, log)
        finally:
            _PITCH[0] = None
    return run_task


TASKS = {"drawer": _pitched(drawer), "lid": _pitched(lid), "fridge": fridge}   # ngăn dưới chưa làm: tay nắm ngăn trên và mặt ngăn trên chắn lòng bàn kẹp khi hạ từ trên xuống


CHOREO_DEFAULT = "new"     # vòng 5: cả bài đạt ba cổng 20 seed (docs/CHOREO-M1-ketqua.md mục 8.6); cũ: --choreo old


def run(robot, only=("drawer", "lid", "fridge"), snap=None, log=print, choreo: str = CHOREO_DEFAULT) -> dict:
    """choreo: "old" = từng bước, đế đi theo tay nắm; "new" = biên đạo docs/CHOREO-M1.md (lab_choreo.open_close)."""
    snap = snap or (lambda *a, **k: None)
    if choreo == "new":
        import lab_choreo as CH
        out = CH.open_close(robot, only, snap=snap, log=log)
        n = sum(r["ok"] for r in out.values())
        return {"task": "open_close", "pass": n == len(out), "done": n, "total": len(out), "detail": out}
    snap("1-bat-dau", "lab_overview")
    LC.raise_to_work(robot, LIFT_HIGH)
    out = {}
    for name in only:
        r = TASKS[name](robot, snap, log)
        r["ok"] = bool(r["open_ok"] and r["closed_ok"])
        r["open_ok"], r["closed_ok"] = bool(r["open_ok"]), bool(r["closed_ok"])
        out[name] = r
        log(f"  {name}: {r}")
        robot.home(duration=1.0)
    snap("3-xong", "lab_overview")
    n = sum(r["ok"] for r in out.values())
    return {"task": "open_close", "pass": n == len(out), "done": n, "total": len(out), "detail": out}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--only", default="drawer,lid,fridge")
    ap.add_argument("--snap", action="store_true")
    ap.add_argument("--choreo", choices=("old", "new"), default=CHOREO_DEFAULT)
    a = ap.parse_args()
    LC.headless_env(not a.viewer)
    robot = LC.make_robot(viewer=a.viewer)
    snap = LC.Snapper(robot, "open-close", "lab_overview", enabled=a.snap)
    res = run(robot, a.only.split(","), snap, log=lambda m: print(m, file=sys.stderr), choreo=a.choreo)
    res["snapshots"] = [str(p) for p in snap.saved]
    print(json.dumps(res, ensure_ascii=False))
    robot.close()
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
