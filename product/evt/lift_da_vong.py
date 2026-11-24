"""Thử chế độ đa vòng servo trục nâng (CHẶN QC2-cot-nang #5: 0,5 m = 64.000 bước >> 4.096; Q-FQC-C8, C9 phụ thuộc).

Trả lời bốn câu hỏi trên MỘT STS3095 rời trên bàn (ID 16 hoặc servo mẫu), horn tự do, KHÔNG nối vitme/đai:
  1. MODE đọc/ghi được giá trị 3 (chế độ bước) không, và giữ trong EEPROM sau mất điện?
  2. Ở MODE 3, đích > 4095 (+6000, −6000 tương đối; --toan-dai: ±32000 tuyệt đối) servo có tới và PRESENT_POSITION đọc có dấu, nhiều vòng?
  3. Ở MODE 0, đích > 4095 bị kẹp/quấn thế nào (để biết SDK hiện tại đặt MODE 0 cho lift là sai)?
  4. Sau mất điện, số đếm nhiều vòng còn giữ không (không giữ = cần homing về công tắc dưới mỗi lần bật)?
Kết thúc trả MODE về giá trị ban đầu (trừ --giu-mode). Giới hạn mô-men 166‰ (spec.LIFT_TORQUE_LIMIT_PERMILLE), tốc độ thấp.
Chế độ khô: --mock-da-vong co|khong, --mock-giu co|khong để xem hai nhánh kết luận.
"""
from __future__ import annotations

import evtlib as E
from evtlib import P, spec


def add_args(ap):
    ap.add_argument("--id", type=int, default=E.EVT_IDS["lift"])
    ap.add_argument("--toan-dai", action="store_true", help="thử thêm ±32000 (≈ 7,8 vòng mỗi phía, ~35 s mỗi lượt)")
    ap.add_argument("--toc-do", type=int, default=1000, help="GOAL_SPEED bước/s")
    ap.add_argument("--giu-mode", action="store_true")
    ap.add_argument("--mock-da-vong", choices=("co", "khong"), default="co")
    ap.add_argument("--mock-giu", choices=("co", "khong"), default="khong")


def set_mode(bus, sid, mode):
    bus.write(sid, P.TORQUE_ENABLE, 0)
    bus.unlock_eeprom(sid)
    try:
        bus.write(sid, P.MODE, mode)
    finally:
        bus.lock_eeprom(sid)
    return bus.read(sid, P.MODE)


def go(ctx, bus, sid, goal, speed, tol=20):
    bus.write(sid, P.GOAL_POSITION, P.encode_signed(goal))
    t_max = abs(goal - P.decode_signed(bus.read(sid, P.PRESENT_POSITION))) / max(speed, 1) + 3.0
    t0, p = ctx.now(), None
    while ctx.now() - t0 < t_max:
        ctx.sleep(0.2)
        p = P.decode_signed(bus.read(sid, P.PRESENT_POSITION))
        if abs(p - goal) <= tol:
            break
    return p


def main(args) -> int:
    mock = dict(ids={"lift": args.id}, multiturn=args.mock_da_vong == "co", retain=args.mock_giu == "co")
    ctx = E.Ctx(args, "lift_da_vong", ["QC2-cot-nang #5", "Q-FQC-C8", "Q-FQC-C9"], mock)
    ctx.confirm("Servo STS3095 RỜI trên bàn, kẹp ê tô, horn tự do, KHÔNG nối vitme/đai; nguồn DC giới hạn 2 A; chỉ servo này trên bus")
    bus = ctx.bus()
    sid = args.id
    res = {"fw": f"{bus.read(sid, E.FW_MAJOR)}.{bus.read(sid, E.FW_MINOR)}", "model": bus.read(sid, E.MODEL_NUMBER)}
    mode0 = res["mode_ban_dau"] = bus.read(sid, P.MODE)
    try:
        # (3) MODE 0 với đích > 4095
        set_mode(bus, sid, P.MODE_POSITION)
        bus.write(sid, P.TORQUE_LIMIT, spec.LIFT_TORQUE_LIMIT_PERMILLE)
        bus.write(sid, P.GOAL_SPEED, args.toc_do)
        bus.write(sid, P.GOAL_POSITION, 2048)
        bus.write(sid, P.TORQUE_ENABLE, 1)
        go(ctx, bus, sid, 2048, args.toc_do)
        bus.write(sid, P.GOAL_POSITION, 5000)
        ctx.sleep(4.0)
        res["mode0_dich_5000_doc"] = bus.read(sid, P.PRESENT_POSITION)
        ctx.check("QC2-cot-nang #5", "MODE 0, đích 5000: vị trí đọc", res["mode0_dich_5000_doc"], "ghi nhận (≤ 4095 = kẹp/quấn)", None, "bước")
        # (1) MODE 3
        got = set_mode(bus, sid, E.MODE_STEP)
        res["mode3_doc_lai"] = got
        ok_mode = got == E.MODE_STEP
        ctx.check("QC2-cot-nang #5", "ghi MODE = 3, đọc lại", got, "3", ok_mode)
        bus.write(sid, P.TORQUE_LIMIT, spec.LIFT_TORQUE_LIMIT_PERMILLE)
        bus.write(sid, P.GOAL_SPEED, args.toc_do)
        p0 = P.decode_signed(bus.read(sid, P.PRESENT_POSITION))
        bus.write(sid, P.GOAL_POSITION, P.encode_signed(p0))
        bus.write(sid, P.TORQUE_ENABLE, 1)
        targets = [p0 + 6000, p0 - 6000] + ([32000, -32000] if args.toan_dai else [])
        reach = []
        for g in targets:
            p = go(ctx, bus, sid, g, args.toc_do)
            reach.append(dict(dich=g, doc=p, toi=p is not None and abs(p - g) <= 20))
            ctx.say(f"đích {g:+6d} -> đọc {p}")
        res["mode3_thu"] = reach
        multi = ok_mode and all(r["toi"] for r in reach)
        signed = any(r["doc"] is not None and r["doc"] < 0 for r in reach) or any((r["doc"] or 0) > 4095 for r in reach)
        res["ho_tro_da_vong"], res["doc_co_dau_nhieu_vong"] = multi, signed
        ctx.check("QC2-cot-nang #5", "MODE 3 tới đích ngoài 0..4095", multi, "có = dùng được 128.000 bước/m", None,
                  note=f"{sum(r['toi'] for r in reach)}/{len(reach)} đích")
        ctx.check("QC2-cot-nang #5", "PRESENT_POSITION đọc nhiều vòng, có dấu", signed, "ghi nhận", None)
        last = reach[-1]["doc"] if reach else None
        bus.write(sid, P.TORQUE_ENABLE, 0)
        # (4) mất điện
        ctx.action("Tắt nguồn DC 5 s rồi bật lại (KHÔNG xoay horn)", ctx.port.power_cycle if ctx.dry else None)
        ctx.sleep(1.0)
        after = P.decode_signed(bus.read(sid, P.PRESENT_POSITION))
        mode_after = bus.read(sid, P.MODE)
        keep = last is not None and abs(after - last) <= 20
        res.update(sau_mat_dien_doc=after, truoc_mat_dien=last, giu_dem_sau_mat_dien=keep, mode_sau_mat_dien=mode_after)
        ctx.check("QC2-cot-nang #5", "số đếm sau mất điện", f"{last} -> {after}", "ghi nhận (không giữ = cần homing)", None)
        ctx.check("QC2-cot-nang #5", "MODE giữ sau mất điện (EEPROM)", mode_after, "3", mode_after == E.MODE_STEP if ok_mode else None)
    finally:
        try:
            bus.write(sid, P.TORQUE_ENABLE, 0)
            if not args.giu_mode:
                set_mode(bus, sid, mode0)
        except (TimeoutError, P.ProtocolError) as e:
            print(f"  cảnh báo: không trả MODE về {mode0}: {e}")
    if res.get("ho_tro_da_vong") and res.get("giu_dem_sau_mat_dien"):
        kl = "Đa vòng có, giữ đếm: giữ hiệu chuẩn 128.000 bước/m; SDK enable() phải đặt MODE 3 cho lift (hiện đặt 0)."
    elif res.get("ho_tro_da_vong"):
        kl = "Đa vòng có, KHÔNG giữ đếm: cần homing về công tắc dưới mỗi lần bật (SDK chưa có) + enable() đặt MODE 3."
    else:
        kl = "KHÔNG đa vòng: đổi sang chế độ vận tốc + đếm vòng phần mềm hoặc encoder ngoài, kèm homing (QC2-cot-nang 5c)."
    res["ket_luan"] = kl
    ctx.data.update(res)
    ctx.say("KẾT LUẬN: " + kl)
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
