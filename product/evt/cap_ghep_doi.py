"""Cặp servo ghép đôi vai/khuỷu: chiều quay, offset (zero), chênh góc và chênh tải (EVT-01; Q-FQC-C4 phần chênh tải; 06-lap-rap 3A, 3B, 9.4).

  --giai-doan 3A  horn CHƯA bắt vào link: cấp điện RIÊNG từng servo (servo kia rút cáp), quay +10° chậm, người đo ghi chiều
                  (dương = gập xuống theo quy ước spec.py). Kết quả: dấu từng servo; A và B phải ngược chiều (lắp đối xứng).
  --giai-doan 3B --pha tat   hai horn đã bắt vào link, 20 servo trên bus, MÔ-MEN TẮT suốt pha: đặt link ở tư thế zero
                  (tay duỗi ngang phía trước, thước thuỷ), đọc raw hai servo -> zero; kiểm chênh góc ở tư thế thứ hai (đưa bằng tay,
                  thước đo góc). Ghi calib.json vào thư mục serial.
  (giữa hai pha: quay_nguoc.py --doi-tuong cap --khop <khớp> phải ĐẠT — ĐIỂM DỪNG 06-lap-rap mục 1)
  --giai-doan 3B --pha giu   bật mô-men giữ 60 s, check_pairs: chênh góc ≤ 3° [giả định], chênh tải ≤ 20% (Q-FQC-C4).
                  Từ chối nếu trong thư mục serial không có kết quả quay_nguoc cặp của khớp này đạt, trừ khi
                  --ngoai-le-duyet "Họ tên người duyệt" (ghi vào kết quả).
"""
from __future__ import annotations

import json
import math
import warnings

import evtlib as E
from evtlib import P, spec


def add_args(ap):
    ap.add_argument("--giai-doan", choices=("3A", "3B"), default="3B")
    ap.add_argument("--khop", default="left_shoulder", help="left_shoulder/right_shoulder/left_elbow/right_elbow")
    ap.add_argument("--goc-2", type=float, default=-45.0, help="3B: tư thế kiểm thứ hai (độ khớp)")
    ap.add_argument("--giu-s", type=float, default=60.0, help="3B: thời gian giữ đọc tải (s)")
    ap.add_argument("--pha", choices=("tat", "giu"), default="tat", help="3B: tat = mô-men tắt (offset); giu = bật mô-men giữ tải")
    ap.add_argument("--ngoai-le-duyet", default=None, help="3B giu: bỏ kiểm quay_nguoc, BẮT BUỘC ghi họ tên người duyệt ngoại lệ")


def backdrive_ok(ctx, khop) -> tuple[bool, str]:
    """Có kết quả quay_nguoc --doi-tuong cap cho khớp này, không mục rớt, có mục so ngưỡng đạt (cùng chế độ thật/khô)."""
    best = None
    for f in sorted(ctx.result_dir().glob("quay_nguoc_*.json"), key=lambda f: f.stat().st_mtime_ns):   # lần đo mới nhất quyết định
        if ("_dryrun" in f.name) != ctx.dry:
            continue
        d = json.loads(f.read_text(encoding="utf-8"))
        du = d.get("du_lieu", {})
        if du.get("doi_tuong") != "cap" or (du.get("servo") or [None])[0] != khop:
            continue
        st = [c["trang_thai"] for c in d.get("tieu_chi", [])]
        best = (f.name, "dat" in st and "khong_dat" not in st and "chua_kiem" not in st)
    if best is None:
        return False, f"không có kết quả quay_nguoc cặp {khop}"
    return best[1], best[0] + (" đạt" if best[1] else " KHÔNG đạt")


def main(args) -> int:
    from faibotic.backends.feetech import FeetechBackend, PairFault, default_calibration
    a, b = args.khop, spec.pair_name(args.khop)
    if args.khop.partition("_")[2] not in E.EVT_PAIRED_JOINTS:
        raise SystemExit("--khop phải là khớp ghép đôi (vai/khuỷu)")
    ida, idb = E.EVT_IDS[a], E.EVT_IDS[b]
    mock = dict(ids={a: ida, b: idb}) if args.giai_doan == "3A" else None
    ctx = E.Ctx(args, "cap_ghep_doi", ["EVT-01", "Q-FQC-C4", "Q-IPQC-02"], mock)
    if args.giai_doan == "3A":
        ctx.confirm("Horn CHƯA bắt vào link; servo kẹp chắc trong cradle/ê tô; chỉ cắm cáp MỘT servo mỗi lần")
        bus, signs = ctx.bus(), {}
        bus.retries = 0
        for n, sid, other in ((a, ida, idb), (b, idb, ida)):
            ctx.action(f"Chỉ cắm cáp servo {n} (ID {sid}); rút servo ID {other}")
            alone = bus.ping(sid) and (ctx.dry or not bus.ping(other))
            ctx.check("EVT-01", f"{n}: chỉ một servo trên bus (ID {sid})", alone, "có", alone)
            if not alone:
                continue
            E.require_mode0(bus, sid)
            p0 = bus.read(sid, P.PRESENT_POSITION)
            try:
                bus.write(sid, P.TORQUE_LIMIT, 300)
                bus.write(sid, P.GOAL_SPEED, 200)
                bus.write(sid, P.GOAL_POSITION, min(4095, p0 + 114))
                bus.write(sid, P.TORQUE_ENABLE, 1)
                ctx.sleep(1.5)
            finally:
                bus.write(sid, P.TORQUE_ENABLE, 0)
            sim = 1 if n == a else -1
            s = ctx.ask(f"chieu_{n}", f"{n}: raw tăng làm khớp gập XUỐNG (+1) hay lên (-1)?", sim, int)
            signs[n] = s
            ctx.check("EVT-01", f"{n}: chiều dương thực tế (dấu hiệu chuẩn)", s, "+1 hoặc -1", None)
        if len(signs) == 2:
            ctx.check("EVT-01", f"{a} và {b} ngược chiều (lắp đối xứng)", f"{signs[a]}/{signs[b]}", "ngược dấu", signs[a] == -signs[b])
            _save_cal(ctx, {a: {"sign": signs[a]}, b: {"sign": signs[b]}}, default_calibration)
        return ctx.finish()

    # ---- 3B ----
    if args.pha == "giu":
        return _hold(ctx, args, a, b, ida, idb, PairFault)
    ctx.confirm("Hai horn đã bắt và siết (Q-IPQC-02); MÔ-MEN TẮT suốt pha này; lò xo CHƯA lắp; tay buộc dây chặn")
    cal = ctx.calibration()
    with warnings.catch_warnings():     # mô-men không bật ở bước này; chốt E-stop dùng từ ctx.robot() khi giữ tải
        warnings.simplefilter("ignore", RuntimeWarning)
        be0 = FeetechBackend(port=ctx.port if ctx.dry else args.port, baudrate=args.baud, sleep=ctx.sleep, allow_no_latch=True, calibration=cal,
                             paired_joints=E.EVT_PAIRED_JOINTS)
    be0.disable()
    ctx.action("Mô-men tắt. Đặt link ở tư thế zero (tay duỗi ngang phía trước, thước thuỷ) và giữ")
    ra, rb = be0.bus.read(ida, P.PRESENT_POSITION), be0.bus.read(idb, P.PRESENT_POSITION)
    ctx.check("EVT-01", f"offset {a} so 2048", ra - 2048, "ghi nhận (zero mới)", None, "bước")
    ctx.check("EVT-01", f"offset {b} so 2048", rb - 2048, "ghi nhận (zero mới)", None, "bước")
    cal[a]["zero"], cal[b]["zero"] = ra, rb
    be0.cal = cal
    q2 = math.radians(args.goc_2)
    ctx.action(f"Đưa link bằng tay tới {args.goc_2:.0f}° (thước đo góc), giữ", lambda: (
        ctx.port.set_position(ida, be0.to_raw(a, q2)), ctx.port.set_position(idb, be0.to_raw(b, q2))))
    qa = be0.from_raw(a, be0.bus.read(ida, P.PRESENT_POSITION))
    qb = be0.from_raw(b, be0.bus.read(idb, P.PRESENT_POSITION))
    d = math.degrees(abs(qa - qb))  # public-scan: ok
    ctx.check("EVT-01", f"chênh góc A/B ở {args.goc_2:.0f}° sau hiệu chuẩn", d, f"≤ {spec.PAIR_MAX_POS_DIFF_DEG}° [giả định]", d <= spec.PAIR_MAX_POS_DIFF_DEG, "°",
              f"A {math.degrees(qa):.1f}°, B {math.degrees(qb):.1f}°")  # public-scan: ok
    _save_cal(ctx, {a: cal[a], b: cal[b]}, default_calibration)
    be0.close()
    ctx.say(f"Tiếp theo: quay_nguoc.py --doi-tuong cap --khop {a}; chỉ khi đạt mới chạy --pha giu")
    return ctx.finish()


def _hold(ctx, args, a, b, ida, idb, PairFault):
    ok, why = backdrive_ok(ctx, a)
    ctx.data.update(quay_nguoc=why, ngoai_le_duyet=args.ngoai_le_duyet)
    if not ok:
        if not args.ngoai_le_duyet:
            raise E.Abort(f"ĐIỂM DỪNG 06-lap-rap: {why}. Chạy quay_nguoc.py --doi-tuong cap --khop {a} trước "
                          "(hoặc --ngoai-le-duyet \"Họ tên\" khi chủ 06 đã ký ngoại lệ)")
        ctx.check("QC2-v04", "NGOẠI LỆ: bật mô-men khi quay ngược cặp chưa đạt", args.ngoai_le_duyet, "có người duyệt", None, note=why)
    cal = ctx.calibration()
    ctx.confirm("Quay ngược cặp đã đạt; tay buộc dây chặn; một người giữ E-stop; không ai trong vùng quét")
    r = ctx.robot(enable=True)
    be = r.backend
    if ctx.dry:
        ctx.port.set_load(ida, 150)
        ctx.port.set_load(idb, 140 * cal[b]["sign"] * cal[a]["sign"])
    worst, t0, err = {"pos_diff_deg": 0.0, "load_diff": 0.0}, ctx.now(), None
    try:
        while ctx.now() - t0 < args.giu_s:
            m = be.check_pairs()[a]
            worst = {k: max(worst[k], m[k]) for k in worst}
            ctx.sleep(0.1)
    except PairFault as e:
        err = str(e)
    finally:
        la, lb = be.pair_metrics.get(a, {}).get("load_nm", (None, None))
        r.close()
    ctx.data.update(load_nm=(la, lb), worst=worst, pair_fault=err)
    ctx.check("Q-FQC-C4", f"chênh tải {a}/{b} khi giữ {args.giu_s:.0f} s (lớn nhất)", 100 * worst["load_diff"], "≤ 20%", err is None and worst["load_diff"] <= 0.20, "%",
              err or (f"tải {la:.2f} / {lb:.2f} N·m" if la is not None else ""))
    ctx.check("EVT-01", "chênh góc khi giữ (lớn nhất)", worst["pos_diff_deg"], f"≤ {spec.PAIR_MAX_POS_DIFF_DEG}°", worst["pos_diff_deg"] <= spec.PAIR_MAX_POS_DIFF_DEG, "°")
    return ctx.finish()


def _save_cal(ctx, upd: dict, default_calibration):
    f = ctx.result_dir() / "calib.json"
    cal = json.loads(f.read_text(encoding="utf-8")) if f.exists() else default_calibration(paired_joints=E.EVT_PAIRED_JOINTS)
    for k, v in upd.items():
        cal.setdefault(k, {}).update(v)
    f.write_text(json.dumps(cal, indent=1), encoding="utf-8")
    ctx.say(f"ghi hiệu chuẩn {f}")
    ctx.data["calib_file"] = str(f)


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
