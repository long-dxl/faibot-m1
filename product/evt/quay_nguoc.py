"""Đo mô-men quay ngược STS3095 khi tắt mô-men (CHẶN QC2-v04-mat-dien; ĐIỂM DỪNG 06-lap-rap mục 1: không lắp lò xo
M1-SH-013 và không cấp điện tay khi chưa đo).

Quy trình (mỗi lần thử):
 1. Servo tắt mô-men (kịch bản ghi TORQUE_ENABLE = 0 và đọc lại). Đo servo rời: thân kẹp ê tô má mềm. Đo cặp (sau 3B):
    tay buộc dây chặn, không người trong vùng quét, lò xo CHƯA lắp.
 2. Gắn tay đòn vào horn (servo rời) hoặc móc cân lực vào link/cẳng tay tại bán kính r đã đo (m), lực vuông góc tay đòn.
 3. Tăng lực từ từ (cân lực kéo, hoặc treo quả cân tăng dần) tới khi trục BẮT ĐẦU quay; kịch bản đọc vị trí và báo khi lệch > 1°.
    Ghi lực lớn nhất trước khi quay (N) hoặc khối lượng treo (kg). Trừ mô-men tự thân của tay đòn nếu có (nhập N·m).
 4. Hai chiều (+/−) x 3 góc. mô-men = F x r + mô-men tay đòn.
Ngưỡng (khớp, cả cặp): vai ≥ 1,5 N·m, khuỷu ≥ 2,5 N·m (cad/bao-cao-cad.md v0.4.1). Servo rời: ghi số; ước cặp = 2 x servo nhỏ nhất.
Dưới ngưỡng: KHÔNG lắp lò xo, chuyển phương án (b) phanh hoặc (d) giảm chấn (ghi 08-han-che).
"""
from __future__ import annotations


import evtlib as E
from evtlib import P, spec

THRESH_NM = {"shoulder": 1.5, "elbow": 2.5}
G = 9.81


def add_args(ap):
    ap.add_argument("--doi-tuong", choices=("servo", "cap"), default="servo", help="servo rời trên ê tô, hoặc cặp đã lắp (sau 3B)")
    ap.add_argument("--id", type=int, default=2, help="servo rời: ID (STS3095)")
    ap.add_argument("--khop", default="left_shoulder", help="cặp: left_shoulder/right_shoulder/left_elbow/right_elbow")
    ap.add_argument("--r", type=float, default=None, help="bán kính đặt lực (m), đo bằng thước")
    ap.add_argument("--kieu", choices=("luc", "khoi-luong"), default="luc", help="nhập lực (N) hay khối lượng treo (kg)")
    ap.add_argument("--goc", default="0,120,240", help="3 góc thử (độ, horn hoặc khớp)")


def main(args) -> int:
    if args.doi_tuong == "cap":
        joint = args.khop.partition("_")[2]
        if joint not in THRESH_NM:
            raise SystemExit("--khop phải là vai hoặc khuỷu ghép đôi")
        names = [args.khop, spec.pair_name(args.khop)]
    else:
        name = next((n for n, i in E.EVT_IDS.items() if i == args.id), f"id{args.id}")
        if E.servo_type(name) != "STS3095":
            raise SystemExit(f"ID {args.id} ({name}) không phải STS3095")
        names, joint = [name], ("shoulder" if "shoulder" in name else "elbow" if "elbow" in name else "lift")
    ids = {n: E.EVT_IDS[n] for n in names}
    ctx = E.Ctx(args, "quay_nguoc", ["QC2-v04 CHẶN 1", "EVT-07", "EVT-08"], dict(ids=ids))
    if args.doi_tuong == "cap":
        ctx.confirm("Lò xo cân bằng CHƯA lắp; tay buộc dây chặn; không người trong vùng quét; một người giữ E-stop")
    else:
        ctx.confirm("Servo kẹp ê tô má mềm, tay đòn bắt chắc vào horn, tay người không ở đường quay của tay đòn")
    bus = ctx.bus()
    for sid in ids.values():
        bus.write(sid, P.TORQUE_ENABLE, 0)
        if bus.read(sid, P.TORQUE_ENABLE) != 0:
            raise E.Abort(f"ID {sid} không tắt được mô-men")
    r = args.r if args.r is not None else ctx.ask("r_m", "Bán kính đặt lực r (m)", 0.10)
    if not r or r <= 0:
        raise E.Abort("cần bán kính r > 0")
    t_lever = ctx.ask("t_tay_don", "Mô-men tự thân tay đòn/cẳng tay tại góc thử (N·m, cộng thêm; 0 nếu tay đòn cân)", 0.0) or 0.0
    angles = [float(a) for a in args.goc.split(",")]
    trials, sid0 = [], next(iter(ids.values()))
    sim_nm = 1.4 if args.doi_tuong == "servo" else 2.9
    for ang in angles:
        for d in (+1, -1):
            key = f"g{int(ang)}_{'duong' if d > 0 else 'am'}"
            ctx.action(f"Đặt góc {ang:.0f}°; kéo chiều {'+' if d > 0 else '−'} tăng dần tới khi trục bắt đầu quay")
            p0 = bus.read(sid0, P.PRESENT_POSITION)
            unit = "N" if args.kieu == "luc" else "kg"
            sim_v = (sim_nm - t_lever) / r if args.kieu == "luc" else (sim_nm - t_lever) / r / G
            v = ctx.ask(key, f"Lực lớn nhất trước khi quay ({unit})", round(sim_v, 2))
            if ctx.dry:
                ctx.port.set_position(sid0, (p0 + d * 30) % 4096)
            p1 = bus.read(sid0, P.PRESENT_POSITION)
            moved = abs(((p1 - p0 + 2048) % 4096) - 2048) * 360 / 4096
            nm = None if v is None else (v if args.kieu == "luc" else v * G) * r + t_lever
            trials.append(dict(goc=ang, chieu=d, gia_tri=v, don_vi=unit, nm=nm, lech_doc_bus_deg=round(moved, 2)))
            ctx.say(f"{key}: {nm if nm is None else round(nm, 3)} N·m, encoder lệch {moved:.1f}°")
            if moved < 1.0:
                ctx.say("  cảnh báo: encoder lệch < 1°, kiểm lại trục đã quay thật chưa (hoặc đo ở trục khác)")
    ctx.data.update(trials=trials, r_m=r, t_tay_don=t_lever, doi_tuong=args.doi_tuong, servo=names)
    vals = [t["nm"] for t in trials if t["nm"] is not None]
    mn = min(vals) if len(vals) == len(trials) else None
    ctx.check("QC2-v04", f"mô-men quay ngược nhỏ nhất ({'/'.join(names)}, 2 chiều x {len(angles)} góc)", mn, "ghi nhận", None, "N·m",
              f"lớn nhất {max(vals):.3f}" if vals else "")
    if joint in THRESH_NM and mn is not None:
        thr = THRESH_NM[joint]
        pair_nm = mn if args.doi_tuong == "cap" else 2 * mn
        ok = pair_nm >= thr
        ctx.check("QC2-v04", f"{'cặp' if args.doi_tuong == 'cap' else 'ước cặp = 2 x servo'} {joint} so ngưỡng lắp lò xo", pair_nm,
                  f"≥ {thr} N·m", ok, "N·m", "" if ok else "KHÔNG lắp lò xo: phương án (b) phanh hoặc (d) giảm chấn")
        if args.doi_tuong == "servo":
            ctx.say("Ước cặp từ servo rời chỉ để sàng lọc; ký ĐIỂM DỪNG bằng số đo cặp đã lắp (--doi-tuong cap).")
    elif mn is None:
        ctx.check("QC2-v04", "đủ 2 chiều x 3 góc", None, "đủ", None)
    ctx.data["ma_sat_coulomb_nm"] = mn
    ctx.data["ghi_chu"] = "số này cập nhật FAILSAFE friction trong poweroff.py/test_poweroff (xfail ma sát) sau khi QC duyệt"
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
