"""Độ lặp lại điểm kẹp (TCP) (EVT-14; Q-FQC-07): 20 lần về cùng tư thế, xen kẽ hai chiều tiếp cận.

Đồng hồ so (hoặc giá hiệu chỉnh) chạm đầu kẹp ở tư thế đích; mỗi lần người đo nhập số đọc (mm). Kịch bản cũng ghi TCP tính
từ encoder (động học SDK): chỉ thấy lệch phía servo, không thấy rơ hộp số/uốn link, nên số ký là số đồng hồ so.
"""
from __future__ import annotations

import math
import random

import numpy as np

import evtlib as E
from evtlib import spec


def add_args(ap):
    ap.add_argument("--tay", choices=spec.SIDES, default="left")
    ap.add_argument("--lan", type=int, default=20)
    ap.add_argument("--dich", default="0,40,20",   # 24/11 (G1/E3): 10,60,30 + 15 độ đưa kẹp vào vùng cấm bệ trạm
                     help="vai,khuỷu,cổ tay (độ) tư thế đích; mặc định điểm gần")
    ap.add_argument("--lech", type=float, default=15.0, help="độ lệch tư thế tiếp cận mỗi phía (độ)")


def main(args) -> int:
    ctx = E.Ctx(args, "lap_lai_tcp", ["EVT-14", "Q-FQC-07"])
    s = args.tay
    names = [spec.joint_name(s, j) for j in ("shoulder", "elbow", "wrist_flex")]
    goal = {n: math.radians(float(v)) for n, v in zip(names, args.dich.split(","))}
    ctx.confirm("Đồng hồ so gá chắc, chạm đầu kẹp ở tư thế đích; không ai trong vùng quét")
    r = ctx.robot()
    rng = random.Random(1)
    tcp, dial = [], []
    try:
        for k in range(args.lan):
            d = math.radians(args.lech) * (1 if k % 2 == 0 else -1)
            r.move_joints({n: v + d for n, v in goal.items()}, duration=1.5)
            r.move_joints(goal, duration=1.5)
            r.wait(1.0)
            tcp.append(np.asarray(r.tcp_position(s)) * 1000)
            dial.append(ctx.ask(f"lan{k + 1}", f"Lần {k + 1} ({'+' if d > 0 else '−'}): số đọc đồng hồ so (mm)", round(rng.gauss(0, 0.3), 3)))
    finally:
        r.close()
    tcp = np.array(tcp)
    spread = float(np.max(np.linalg.norm(tcp - tcp.mean(axis=0), axis=1))) if len(tcp) else None
    ctx.data.update(tcp_mm=tcp.round(3).tolist(), dong_ho_so_mm=dial)
    ctx.check("Q-FQC-07", f"lệch TCP theo encoder, tay {s} (lớn nhất so trung bình)", spread, "ghi nhận (encoder)", None, "mm")
    got = [v for v in dial if v is not None]
    pm = (max(got) - min(got)) / 2 if len(got) == len(dial) and got else None
    ctx.check("Q-FQC-07", f"độ lặp lại điểm kẹp tay {s} (đồng hồ so, ±nửa dải)", pm, "≤ ±2 mm [giả định]", None if pm is None else pm <= 2.0, "mm")
    ctx.check("EVT-14", "độ lặp lại ±2 mm", pm, "≤ ±2 mm [giả định]", None if pm is None else pm <= 2.0, "mm")
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
