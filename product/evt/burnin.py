"""Burn-in 30 phút (Q-FQC-10; hỗ trợ EVT-10 nhiệt dây, EVT-05): chu trình hai tay liên tục ở backend feetech.

Đạt: 30 phút không reset (PairFault/E-stop), không mất bus quá 3 gói liên tiếp, nhiệt vỏ servo ≤ 65 °C [giả định].
Trục nâng chỉ chạy khi lift_da_vong.py đã kết luận và có công tắc thật (--co-lift); bánh chỉ chạy khi đế kê cao (--co-de).
Chế độ khô mặc định 1 phút ảo (đặt --phut 30 nếu muốn mô phỏng đủ, chậm hơn).
"""
from __future__ import annotations

import math

import evtlib as E
from evtlib import P, spec

POSES = [
    {"shoulder": -60, "elbow": 90, "wrist_flex": 60, "pan": 0},
    {"shoulder": 0, "elbow": 45, "wrist_flex": 30, "pan": 30, "wrist_yaw": 30},       # chạy cả servo yaw
    {"shoulder": 0, "elbow": 60, "wrist_flex": 0, "pan": -30},     # 24/11 (G1/E3): (20, 70, 0) đưa tâm kẹp xuống dưới mặt khay + 3 cm ở lift 0, Robot từ chối
    {"shoulder": -30, "elbow": 20, "wrist_flex": -30, "pan": 0, "wrist_yaw": -30},
]


def add_args(ap):
    ap.add_argument("--phut", type=float, default=None, help="mặc định 30 (thật), 1 (chạy khô)")
    ap.add_argument("--co-lift", action="store_true")
    ap.add_argument("--co-de", action="store_true")


def main(args) -> int:
    from faibotic.backends.feetech import EStopActive, PairFault
    from faibotic.feetech.bus import SyncReadTimeout
    ctx = E.Ctx(args, "burnin", ["Q-FQC-10", "EVT-10", "EVT-05"])
    minutes = args.phut if args.phut is not None else (1.0 if ctx.dry else 30.0)
    ctx.confirm("Giới hạn mềm đã nạp, 3B xong, cặp ghép đôi đạt; không ai trong vùng quét; đế kê cao nếu --co-de; một người giữ E-stop")
    r = ctx.robot()
    be = r.backend
    ids = list(E.EVT_IDS.values())
    stats = dict(chu_trinh=0, mat_lien_tiep_max=0, loi_bus=0, reset=0, nhiet_max={}, v_min=99.0)
    lost, t0, t_log, k = 0, ctx.now(), -1e9, 0
    try:
        while ctx.now() - t0 < minutes * 60:
            p = POSES[k % len(POSES)]
            tgt = {spec.joint_name(s, j): math.radians(v * (1 if s == "left" else (-1 if j == "pan" else 1))) for s in spec.SIDES for j, v in p.items()}
            for s in spec.SIDES:
                tgt[spec.joint_name(s, "gripper")] = 100.0 if k % 2 else 0.0
            if args.co_lift:
                tgt["lift"] = 0.20 + 0.05 * (k % 2)
            if args.co_de:
                r.set_base_velocity(0.0, 0.0, 0.3 if k % 2 else -0.3)
            try:
                r.move_joints(tgt, duration=2.0)
                lost = 0
            except (TimeoutError, SyncReadTimeout, P.ProtocolError) as e:
                lost += 1
                stats["loi_bus"] += 1
                stats["mat_lien_tiep_max"] = max(stats["mat_lien_tiep_max"], lost)
                ctx.say(f"lỗi bus: {e}")
                if lost > 3:
                    break
            k += 1
            stats["chu_trinh"] = k
            ctx.checkpoint(dict(stats, phut_chay=(ctx.now() - t0) / 60))
            if ctx.now() - t_log >= 10:
                t_log = ctx.now()
                tem = be.bus.sync_read(P.PRESENT_TEMPERATURE, ids)
                vol = be.bus.sync_read(P.PRESENT_VOLTAGE, ids)
                for sid, v in tem.items():
                    stats["nhiet_max"][sid] = max(stats["nhiet_max"].get(sid, 0), v)
                stats["v_min"] = min(stats["v_min"], min(vol.values()) * E.VOLT_PER_LSB)
    except (PairFault, EStopActive) as e:
        stats["reset"] += 1
        stats["dung"] = str(e)
    except KeyboardInterrupt:
        stats["dung"] = "người đo dừng (Ctrl-C)"
    finally:
        r.close()
    elapsed = (ctx.now() - t0) / 60
    interrupted = stats.get("dung", "").startswith("người đo")
    ctx.data.update(stats, phut_chay=elapsed)
    tmax = max(stats["nhiet_max"].values()) if stats["nhiet_max"] else None
    full = elapsed >= minutes - 1e-6 and minutes >= 30
    ctx.check("Q-FQC-10", "thời gian chạy liên tục", round(elapsed, 2), "30 phút, 0 reset", (stats["reset"] == 0 and elapsed >= 30) if (full or stats["reset"]) and not interrupted else None,
              "phút", stats.get("dung", "") or ("" if full else f"chạy {minutes} phút: chưa đủ để ký"))
    ctx.check("Q-FQC-10", "mất bus liên tiếp lớn nhất", stats["mat_lien_tiep_max"], "≤ 3 gói", stats["mat_lien_tiep_max"] <= 3, "gói", f"tổng lỗi {stats['loi_bus']}")
    ctx.check("Q-FQC-10", "nhiệt servo lớn nhất qua bus", tmax, "≤ 65 °C [giả định]", None if tmax is None else tmax <= 65, "°C")
    ctx.check("EVT-05", "điện áp servo thấp nhất (bus)", stats["v_min"], "ghi nhận", None, "V")
    w = ctx.ask("nhiet_day_max", "EVT-10: nhiệt dây trục chính/nhánh/busbar nóng nhất sau burn-in (camera nhiệt, °C)", 41.0)
    ctx.check("EVT-10", "nhiệt dây/busbar sau burn-in (ở dòng teleop, chưa phải 49,8 A)", w, "ghi nhận; thử 49,8 A ở ghi_tay", None, "°C")
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
