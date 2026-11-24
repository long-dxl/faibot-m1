"""Thử E-stop và chốt reset (EVT-02, EVT-03, EVT-04, EVT-12; Q-IPQC-09; Q-FQC-01..04). Chạy TRƯỚC khi cắm bất kỳ servo nào.

Mạch chọn bằng --mach (evtlib): bang-di-day (bộ #0, mặc định, electrical/bang-di-day-evt.md) hoặc pcb (PCB nguồn sau EVT).

Bộ #0 (--mach bang-di-day): chân Pi K5_EN (ra), ESTOP_OK (opto trên RUN), ARM_SENSE (opto trên SERVO+); không có RELAY_EN,
LATCH_SENSE, U4, INA226.
  --che-do jig      thử bàn bằng nguồn 12 V giới hạn 3 A, chưa có Pi (bang-di-day-evt.md mục 6.3): K5 kích bằng công tắc tay,
                    người đo đi từng dòng bảng trạng thái, nhìn rơ-le/đồng hồ trên SERVO+ và nhập kết quả.
  --che-do pi       Pi nối WE07, dùng faibotic.safety.SafetyLatch (bang_di_day): Pi bật K5, NGƯỜI nhấn và nhả Reset, Pi chỉ
                    cho chạy khi ESTOP_OK và ARM_SENSE xác nhận K1 đóng; đo ESTOP_OK xuống -> ARM_SENSE xuống (~1 ms, tham khảo).
  Mỗi chu kỳ: reset -> K1 đóng; nhấn E-stop -> K1 nhả; nhả E-stop -> KHÔNG có lại điện; nhấn/nhả Reset khi K5 hạ -> vẫn
  không; giữ Reset (kẹt) -> không chạy; Pi hạ K5 -> K1 nhả; đứt dây E-stop (tháo một dây W14) -> K1 nhả.
  --che-do xa-tu    ĐÃ CẮM servo, mô-men tắt, trước lần bật mô-men đầu: Pi bật K5, người GIỮ Reset (K1 không đóng), đo từ
                    lúc K4 nhả (RUN lên + KT) tới lúc ARM_SENSE rớt, 3 lần, lấy lớn nhất. Số này truyền --xa-tu-ms cho
                    các kịch bản bật mô-men; phải <= EVT_ARM_CONFIRM_S / 2, nếu không SafetyLatch từ chối reset.

PCB (--mach pcb), giữ như cũ:
  --che-do jig      jig J6, chưa có Pi: kịch bản đọc từng bước, người đo quan sát LED/đồng hồ và nhập kết quả + số oscilloscope.
  --che-do pi       Pi nối J6 qua W15: Pi tạo cạnh RELAY_EN, đọc ESTOP_SENSE/LATCH_SENSE (K1_FEED), đo thời gian
                    từ ESTOP_SENSE xuống tới LATCH_SENSE xuống (độ phân giải ~1 ms, chỉ tham khảo; số ký Q-FQC-01 lấy oscilloscope).
  --che-do phan-mem chốt tạm bằng phần mềm (faibotic.safety.SafetyLatch) khi PCB chưa có chốt phần cứng: 10/10 "nhả không có lại điện".
  Mỗi chu kỳ: reset + cạnh RELAY_EN -> K1 đóng; nhấn E-stop -> K1 nhả (đo ms); nhả E-stop -> KHÔNG có lại điện;
  nhấn/nhả Reset -> vẫn chưa (cần cạnh RELAY_EN); giữ Reset rồi nhả E-stop -> không reset được; rút J8 -> K1 nhả.
"""
from __future__ import annotations

import contextlib
import io

import evtlib as E
from evtlib import FakeGPIO, SimBangDiDay

K1_DROP_SIM_S = 0.030   # thời gian nhả K1 trong mô phỏng (không phải số đo)


class SimPCB(FakeGPIO):
    """Mạch nguồn giả theo D4: K1 đóng chỉ khi E-stop nhả, J8 cắm, đã có chuỗi Reset nhấn-nhả hợp lệ rồi cạnh lên RELAY_EN
    (Reset không đang giữ). Nhả K1 trễ K1_DROP_SIM_S theo đồng hồ ảo."""

    def __init__(self, ctx, hw_latch: bool = True, reclose: bool = False):
        super().__init__(ESTOP_SENSE=True, LATCH_SENSE=False, RELAY_EN=False)
        self.ctx, self.hw = ctx, hw_latch
        self.reclose = reclose     # không có chốt phần cứng (hw_latch=False) cũng vậy. PCB lỗi (thiếu/hỏng chốt K2/K3/U4): nhả E-stop khi RELAY_EN cao thì K1 tự đóng lại
        self.estop = self.reset_held = self.j8_out = False
        self.armed, self.k1, self.t_drop = False, False, None

    def press(self, what: str, down: bool) -> None:
        if what == "estop":
            self.estop = down
        elif what == "reset":
            if self.reset_held and not down and not self.estop:
                self.armed = True
            self.reset_held = down
        elif what == "j8":
            self.j8_out = down
        if (self.estop or self.j8_out) and self.k1 and self.t_drop is None:
            self.t_drop = self.ctx.now() + K1_DROP_SIM_S

    def write(self, pin, level):
        prev = self.levels.get(pin)
        super().write(pin, level)
        if pin == "RELAY_EN":
            ok_in = not self.estop and not self.j8_out and not self.reset_held
            if self.hw:
                if prev is False and level and self.armed and ok_in:
                    self.k1, self.armed, self.t_drop = True, False, None
            else:                          # không có chốt phần cứng: K1 theo RELAY_EN (phần mềm chịu trách nhiệm)
                self.k1 = bool(level) and ok_in
            if not level and self.k1:
                self.k1 = False

    def read(self, pin):
        if self.t_drop is not None and self.ctx.now() >= self.t_drop:
            self.k1, self.t_drop = False, None
        if (self.reclose or not self.hw) and not self.estop and not self.j8_out and not self.reset_held and self.levels.get("RELAY_EN") and self.t_drop is None:
            self.k1 = True
        if pin == "ESTOP_SENSE":
            return not self.estop and not self.j8_out
        if pin == "LATCH_SENSE":
            return self.k1
        return super().read(pin)


def add_args(ap):
    ap.add_argument("--che-do", choices=("jig", "pi", "phan-mem", "xa-tu"), default="pi")
    ap.add_argument("--tu-giu-s", type=float, default=0.25, help="chạy khô --che-do xa-tu: thời gian tụ giả giữ ARM_SENSE")
    ap.add_argument("--chu-ky", type=int, default=10, help="số chu kỳ (6.6: 10; lặp với Pi/pack: 3)")
    ap.add_argument("--timeout", type=float, default=30.0, help="giây chờ người đo thao tác")
    ap.add_argument("--mock-loi", choices=("khong", "khong-chot", "k1-dinh"), default="khong",
                    help="chạy khô: khong-chot = mạch giả mất chốt (PCB cấp lại điện khi nhả E-stop; bộ #0 K2 dính); "
                         "k1-dinh = bộ #0 K1 dính tiếp điểm khi E-stop lần đầu. Kịch bản phải báo KHÔNG ĐẠT")


def wait_level(ctx, g, pin, level, timeout, step=0.0005):
    t0 = ctx.now()
    while g.read(pin) != level:
        if ctx.now() - t0 > timeout:
            return None
        ctx.sleep(step)
    return ctx.now()


def _cycle(ctx, g, sim, tmo, k):
    """Một chu kỳ 6.6 với Pi (hoặc PCB giả). Trả dict kết quả."""
    op = (lambda what, down: (lambda: sim.press(what, down))) if sim else (lambda *a: None)
    drop_ms = []
    r = {"chu_ky": k}
    g.write("RELAY_EN", False)
    if wait_level(ctx, g, "ESTOP_SENSE", True, tmo) is None:
        ctx.say("ESTOP_SENSE không lên: nhả E-stop / kiểm J8")
    ctx.action("Nhấn rồi nhả nút Reset", lambda: (sim.press("reset", True), sim.press("reset", False)))
    g.write("RELAY_EN", True)
    r["k1_dong"] = wait_level(ctx, g, "LATCH_SENSE", True, 0.5) is not None
    ctx.action("Nhấn E-stop (giữ nhấn)", op("estop", True))
    t0 = wait_level(ctx, g, "ESTOP_SENSE", False, tmo)
    t1 = wait_level(ctx, g, "LATCH_SENSE", False, 1.0) if t0 is not None else None
    r["nha_k1_ms"] = None if t1 is None else round(1000 * (t1 - t0), 2)
    if r["nha_k1_ms"] is not None:
        drop_ms.append(r["nha_k1_ms"])
    ctx.action("Nhả E-stop (RELAY_EN vẫn cao)", op("estop", False))
    wait_level(ctx, g, "ESTOP_SENSE", True, tmo)
    ctx.sleep(1.0)
    r["nha_estop_khong_cap_lai"] = not g.read("LATCH_SENSE")
    ctx.action("Nhấn rồi nhả Reset (chưa có cạnh RELAY_EN)", lambda: (sim.press("reset", True), sim.press("reset", False)))
    ctx.sleep(0.5)
    r["reset_can_canh"] = not g.read("LATCH_SENSE")
    g.write("RELAY_EN", False)
    g.write("RELAY_EN", True)
    r["reset_canh_dong_lai"] = wait_level(ctx, g, "LATCH_SENSE", True, 0.5) is not None
    ctx.action("Nhấn E-stop, GIỮ nút Reset, rồi nhả E-stop (vẫn giữ Reset)",
               lambda: (sim.press("estop", True), sim.press("reset", True), sim.press("estop", False)))
    ctx.sleep(0.1)
    g.write("RELAY_EN", False)
    g.write("RELAY_EN", True)
    ctx.sleep(0.5)
    r["reset_ket_khong_chay"] = not g.read("LATCH_SENSE")
    ctx.action("Nhả nút Reset", lambda: sim.press("reset", False))
    g.write("RELAY_EN", False)
    ctx.action("Nhấn rồi nhả Reset", lambda: (sim.press("reset", True), sim.press("reset", False)))
    g.write("RELAY_EN", True)
    wait_level(ctx, g, "LATCH_SENSE", True, 0.5)
    ctx.action("Rút giắc J8 khi K1 đang đóng", op("j8", True))
    r["rut_j8_k1_nha"] = wait_level(ctx, g, "LATCH_SENSE", False, 1.0) is not None
    ctx.action("Cắm lại J8", op("j8", False))
    g.write("RELAY_EN", False)
    r["dat"] = all(v for kk, v in r.items() if kk not in ("chu_ky", "nha_k1_ms")) and r["nha_k1_ms"] is not None and r["nha_k1_ms"] < 100
    return r, drop_ms


CODES = ["EVT-02", "EVT-03", "EVT-04", "EVT-12", "Q-IPQC-09", "Q-FQC-01", "Q-FQC-02", "Q-FQC-03", "Q-FQC-04"]


def main(args) -> int:
    ctx = E.Ctx(args, "estop_jig", CODES, dict(ids={}))
    if ctx.hw == "bang_di_day":
        return _main_bang(ctx, args)
    return _main_pcb(ctx, args)


def _main_pcb(ctx, args) -> int:
    ctx.confirm("KHÔNG có servo nào cắm vào J2–J4; nguồn DC giới hạn 2 A thay pack (hoặc pack ở 6.14); jig J6 có mã số")
    sim = SimPCB(ctx, hw_latch=args.che_do != "phan-mem", reclose=args.mock_loi == "khong-chot") if ctx.dry else None
    if args.che_do == "jig":
        return _jig(ctx, args, sim)
    g = ctx.gpio(lambda: sim, required=E.GPIO_PINS["pcb"]["required"])
    if args.che_do == "phan-mem":
        return _software(ctx, args, g, sim)
    tmo = args.timeout
    rows, drop_ms = [], []
    for k in range(1, args.chu_ky + 1):
        r, dm = _cycle(ctx, g, sim, tmo, k)
        drop_ms += dm
        rows.append(r)
        ctx.say(f"chu kỳ {k}: {r}")
    ctx.data["chu_ky"] = rows
    n_ok = sum(r["dat"] for r in rows)
    ctx.check("Q-IPQC-09", "chuỗi chốt reset đạt", f"{n_ok}/{len(rows)}", f"{len(rows)}/{len(rows)}", n_ok == len(rows))
    ctx.check("Q-FQC-03", "nhả E-stop không cấp lại điện", f"{sum(r['nha_estop_khong_cap_lai'] for r in rows)}/{len(rows)}", f"{len(rows)}/{len(rows)}",
              all(r["nha_estop_khong_cap_lai"] for r in rows))
    ctx.check("EVT-03", "Reset kẹt không chạy; cần cạnh RELAY_EN", f"{sum(r['reset_ket_khong_chay'] and r['reset_can_canh'] for r in rows)}/{len(rows)}",
              "tất cả", all(r["reset_ket_khong_chay"] and r["reset_can_canh"] for r in rows))
    ctx.check("Q-FQC-04", "rút J8 thì K1 nhả", f"{sum(r['rut_j8_k1_nha'] for r in rows)}/{len(rows)}", "tất cả", all(r["rut_j8_k1_nha"] for r in rows))
    mx = max(drop_ms) if drop_ms else None
    ctx.check("EVT-04", "ESTOP_SENSE xuống -> LATCH_SENSE xuống (Pi, lớn nhất)", mx, "< 100 ms (tham khảo)", None if mx is None else mx < 100, "ms",
              "đo bằng Pi đọc GPIO; KHÔNG thay oscilloscope")
    _scope(ctx)
    return ctx.finish()


def _scope(ctx):
    """Số oscilloscope và đồng hồ (người đo nhập)."""
    t = ctx.ask("estop_cat_ms_max", "Oscilloscope: nhấn nút -> 12 V servo < 1,2 V, lớn nhất 10 lần (ms)", 42.0)
    ctx.check("Q-FQC-01", "E-stop cắt điện servo (oscilloscope, 10 lần, lớn nhất)", t, "< 100 ms", None if t is None else t < 100, "ms")
    ctx.check("EVT-02", "E-stop cắt điện servo (SOR-011)", t, "< 100 ms", None if t is None else t < 100, "ms")
    k1 = ctx.ask("k1_nha_ms", "Oscilloscope: thời gian nhả K1 (cuộn mất điện -> tiếp điểm hở) (ms)", 18.0)
    ctx.check("EVT-04", "thời gian nhả K1 (oscilloscope)", k1, "< 100 ms", None if k1 is None else k1 < 100, "ms")
    z1 = ctx.ask("z1_chap_bat_duoc", "Chập Z1 thử (mẫu riêng): thời gian nhả K1 tăng và bắt được? (c/k)", True, bool)
    ctx.check("EVT-04", "bắt được Zener Z1 chập", z1, "có", z1)
    v = ctx.ask("v_servo_sau_1s", "Điện áp J2–J4 sau E-stop 1 s (V)", 0.0)
    ctx.check("Q-FQC-02", "điện áp servo sau E-stop 1 s", v, "0 V (< 0,1 V)", None if v is None else v < 0.1, "V")
    por = ctx.ask("por_ms", "POR U4 /CLR thấp sau cấp nguồn (ms, oscilloscope)", 240.0)
    ctx.check("EVT-12", "POR ≥ 200 ms", por, "≥ 200 ms", None if por is None else por >= 200, "ms")
    n1 = ctx.ask("latch_sense_khong_keo_clr", "Ép LATCH_SENSE (chân Pi) cao/thấp: /CLR U4 không đổi? (c/k)", True, bool)
    ctx.check("EVT-12", "LATCH_SENSE không kéo /CLR U4 (N1)", n1, "có", n1)


def _jig(ctx, args, sim=None) -> int:
    steps = [("k1_dong", "Nhấn/nhả Reset, gạt RELAY_EN tắt->bật: K1 đóng, J2–J4 = điện áp J1 (sụt ≤ 0,2 V)?"),
             ("nhan_k1_nha", "Nhấn E-stop: J2–J4 mất điện, K1 nhả, LED ESTOP_SENSE tắt?"),
             ("nha_khong_cap_lai", "Nhả E-stop, RELAY_EN vẫn bật: J2–J4 KHÔNG có lại điện?"),
             ("reset_can_canh", "Nhấn/nhả Reset: vẫn chưa có điện cho tới khi gạt RELAY_EN tắt rồi bật?"),
             ("reset_ket", "Giữ Reset rồi nhả E-stop, gạt RELAY_EN: KHÔNG reset được?"),
             ("rut_j8", "Rút J8 khi K1 đóng: K1 nhả?")]
    ok_cycles = 0
    sim_map = dict(k1_dong="k1_dong", nhan_k1_nha="nha_k1_ms", nha_khong_cap_lai="nha_estop_khong_cap_lai",
                   reset_can_canh="reset_can_canh", reset_ket="reset_ket_khong_chay", rut_j8="rut_j8_k1_nha")
    for k in range(1, args.chu_ky + 1):
        simr = {}
        if sim is not None:     # chạy khô: câu trả lời lấy từ PCB giả chạy đúng chuỗi thao tác, không mặc định "có"
            with contextlib.redirect_stdout(io.StringIO()):
                r, _ = _cycle(ctx, sim, sim, args.timeout, k)
            simr = {key: (r[v] is not None and r[v] < 100) if v == "nha_k1_ms" else bool(r[v]) for key, v in sim_map.items()}
        res = {key: ctx.ask(f"c{k}_{key}", f"Chu kỳ {k}: {q} (c/k)", simr.get(key), bool) for key, q in steps}
        ok_cycles += all(v is True for v in res.values())
        ctx.data[f"chu_ky_{k}"] = res
    ctx.check("Q-IPQC-09", "chuỗi chốt reset bằng jig J6", f"{ok_cycles}/{args.chu_ky}", f"{args.chu_ky}/{args.chu_ky}", ok_cycles == args.chu_ky)
    ctx.check("EVT-03", "chốt reset: nhả không tự cấp, Reset kẹt, cần cạnh", f"{ok_cycles}/{args.chu_ky}", "tất cả", ok_cycles == args.chu_ky)
    _scope(ctx)
    return ctx.finish()


def _software(ctx, args, g, sim) -> int:
    """ĐIỂM DỪNG bước 9 khi không có chốt phần cứng: SafetyLatch hạ RELAY_EN khi ESTOP_SENSE thấp, chỉ cấp lại khi reset()."""
    from faibotic.safety import ResetRefused, SafetyLatch
    stops = []
    lt = SafetyLatch(g, on_stop=lambda: stops.append(ctx.now()), latch_pin=None, sleep=ctx.sleep, clock=ctx.now, hw="pcb")   # không có chốt phần cứng: không dựa LATCH_SENSE
    ok, lat = 0, []
    for k in range(1, args.chu_ky + 1):
        try:
            if sim:
                sim.press("reset", True), sim.press("reset", False)
            lt.reset()
        except ResetRefused as e:
            ctx.say(f"chu kỳ {k}: reset bị từ chối ({e})")
        run_ok = lt.ok
        ctx.action("Nhấn E-stop", (lambda: sim.press("estop", True)) if sim else None)
        t0 = ctx.now()
        while lt.poll() and ctx.now() - t0 < args.timeout:
            ctx.sleep(0.001)
        lat.append(1000 * (ctx.now() - t0))
        relay_low = g.read("RELAY_EN") is False
        ctx.action("Nhả E-stop, đợi 2 s", (lambda: sim.press("estop", False)) if sim else None)
        ctx.sleep(2.0)
        lt.poll()
        stays_off = not lt.ok and g.read("RELAY_EN") is False and not g.read("LATCH_SENSE")
        good = run_ok and relay_low and stays_off
        ok += good
        ctx.say(f"chu kỳ {k}: chạy {run_ok}, hạ RELAY_EN {relay_low}, nhả không cấp lại {stays_off}")
    ctx.data["nhat_ky_chot"] = [m for _, m in lt.log]
    ctx.check("Q-IPQC-09", "chốt tạm phần mềm: nhả E-stop không có lại điện (ĐIỂM DỪNG bước 9)", f"{ok}/{args.chu_ky}", "10/10", ok == args.chu_ky,
              note="Q-IPQC-09 phần cứng vẫn RỚT nếu PCB không có K2/K3/U4: robot không được giao")
    ctx.check("EVT-03", "phát hiện E-stop bằng poll (lớn nhất, Pi)", max(lat), "ghi nhận", None, "ms")
    return ctx.finish()


# ---------------- bộ #0: bảng đi dây (K5, K2 tự giữ, KT, ESTOP_OK, ARM_SENSE) ----------------
class SimBang(SimBangDiDay):
    """Mạch bộ #0 giả cho chạy khô, thêm lỗi giả: khong-chot = K2 dính từ đầu; k1-dinh = K1 dính khi E-stop lần đầu."""

    def __init__(self, ctx, mock: str = "khong"):
        super().__init__(clock=ctx.now)
        self.mock, self.wire_cut = mock, False
        if mock == "khong-chot":
            self.weld.add("K2")

    def press(self, what, down=True):
        if what == "estop" and down and self.mock == "k1-dinh" and self.k1:
            self.weld.add("K1")
        if what == "wire":                     # đứt dây E-stop = mạch hở như nhấn nút
            self.wire_cut = down
            what, down = "estop", down
        super().press(what, down)


def _bang_cycle(ctx, g, lt, sim, tmo, k):
    """Một chu kỳ với Pi trên bộ #0. Phần mềm chỉ bật/hạ K5; người bấm Reset (chạy khô: người ảo)."""
    from faibotic.safety import ResetRefused
    op = (lambda what, down: (lambda: sim.press(what, down))) if sim else (lambda *a: None)
    r = {"chu_ky": k}

    def reset_ok():
        try:
            ctx.reset_latch(lt, g)
        except ResetRefused as e:
            ctx.say(f"reset bị từ chối: {e}")
        return lt.ok and g.read("ARM_SENSE")
    r["k1_dong"] = reset_ok()
    ctx.action("Nhấn E-stop (giữ nhấn)", op("estop", True))
    t0 = wait_level(ctx, g, "ESTOP_OK", False, tmo)
    t1 = wait_level(ctx, g, "ARM_SENSE", False, 1.0) if t0 is not None else None
    lt.poll()
    r["nha_k1_ms"] = None if t1 is None else round(1000 * (t1 - t0), 2)
    r["pi_ha_k5"] = not lt.ok                  # trip() đã hạ K5
    ctx.action("Nhả E-stop", op("estop", False))
    _poll_for(ctx, lt, 1.0)
    r["nha_estop_khong_cap_lai"] = not g.read("ARM_SENSE") and not lt.ok
    ctx.action("Nhấn rồi nhả Reset (phần mềm CHƯA reset, K5 hạ)", (lambda: sim.click_reset()) if sim else None)
    _poll_for(ctx, lt, 1.0)
    r["reset_can_k5"] = not g.read("ARM_SENSE") and not g.read("ESTOP_OK")
    ctx.action("Nhấn và GIỮ nút Reset (giả Reset kẹt)", op("reset", True))
    try:
        lt.reset(timeout_s=3.0)
    except ResetRefused:
        pass
    _poll_for(ctx, lt, 0.5)
    r["reset_ket_khong_chay"] = not lt.ok and not g.read("ARM_SENSE")
    ctx.action("Nhả nút Reset", op("reset", False))
    if reset_ok():
        lt.trip("thử: Pi hạ K5")
        r["k5_ha_k1_nha"] = wait_level(ctx, g, "ARM_SENSE", False, 1.0) is not None
    else:
        r["k5_ha_k1_nha"] = False
    if reset_ok():
        ctx.action("Tháo một dây E-stop W14 ở domino khi K1 đang đóng", op("wire", True))
        r["dut_day_k1_nha"] = wait_level(ctx, g, "ARM_SENSE", False, 1.0) is not None
        ctx.action("Đấu lại dây W14", op("wire", False))
    else:
        r["dut_day_k1_nha"] = False
    _poll_for(ctx, lt, 0.5)
    r["khong_loi_dinh"] = lt.fault is None
    if lt.fault:
        ctx.say(f"LỖI: {lt.fault}")
    r["dat"] = all(v for kk, v in r.items() if kk not in ("chu_ky", "nha_k1_ms")) and r["nha_k1_ms"] is not None and r["nha_k1_ms"] < 100
    return r


def _poll_for(ctx, lt, secs, step=0.01):
    t0 = ctx.now()
    while ctx.now() - t0 < secs:
        lt.poll()
        ctx.sleep(step)


def _main_bang(ctx, args) -> int:
    if args.che_do == "xa-tu":
        return _xa_tu(ctx, args)
    ctx.confirm("KHÔNG có servo nào cắm vào hub; nguồn DC 12 V giới hạn 3 A thay pack (bang-di-day-evt.md mục 6.3); "
                "kỹ sư điện đã xem sơ đồ; R_pc gắn tản nhiệt")
    if args.che_do == "phan-mem":
        raise SystemExit("--che-do phan-mem chỉ dùng cho --mach pcb (PCB không có chốt phần cứng); bộ #0 có K2 tự giữ: dùng jig hoặc pi")
    sim = SimBang(ctx, args.mock_loi) if ctx.dry else None
    if args.che_do == "jig":
        return _jig_bang(ctx, args, sim)
    g = ctx.gpio(lambda: sim)
    lt = ctx.latch(g, bench=True)              # thử bàn, không servo: xác nhận K1 không cần số xả tụ
    rows = []
    for k in range(1, args.chu_ky + 1):
        r = _bang_cycle(ctx, g, lt, sim, args.timeout, k)
        rows.append(r)
        ctx.say(f"chu kỳ {k}: {r}")
    ctx.data["chu_ky"] = rows
    ctx.data["nhat_ky_chot"] = [m for _, m in lt.log]
    n = len(rows)

    def cnt(key):
        return sum(bool(r[key]) for r in rows)
    ctx.check("Q-IPQC-09", "chuỗi chốt reset bộ #0 (K5 + Reset tay + ARM_SENSE)", f"{cnt('dat')}/{n}", f"{n}/{n}", cnt("dat") == n)
    ctx.check("Q-FQC-03", "nhả E-stop không cấp lại điện", f"{cnt('nha_estop_khong_cap_lai')}/{n}", f"{n}/{n}", cnt("nha_estop_khong_cap_lai") == n)
    ctx.check("EVT-03", "Reset kẹt không chạy; Reset khi K5 hạ không chốt", f"{sum(r['reset_ket_khong_chay'] and r['reset_can_k5'] for r in rows)}/{n}",
              "tất cả", all(r["reset_ket_khong_chay"] and r["reset_can_k5"] for r in rows))
    ctx.check("Q-FQC-04", "Pi hạ K5 / đứt dây E-stop thì K1 nhả (bộ #0 thay rút J8)",
              f"{sum(r['k5_ha_k1_nha'] and r['dut_day_k1_nha'] for r in rows)}/{n}", "tất cả",
              all(r["k5_ha_k1_nha"] and r["dut_day_k1_nha"] for r in rows))
    ctx.check("EVT-04", "không có tín hiệu dính K1/K4/K2 (ARM_SENSE, ESTOP_OK khi K5 hạ)", f"{cnt('khong_loi_dinh')}/{n}", "tất cả",
              cnt("khong_loi_dinh") == n, note="" if lt.fault is None else lt.fault)
    ms = [r["nha_k1_ms"] for r in rows if r["nha_k1_ms"] is not None]
    mx = max(ms) if ms else None
    ctx.check("EVT-04", "ESTOP_OK xuống -> ARM_SENSE xuống (Pi, lớn nhất)", mx, "< 100 ms (tham khảo)", None if mx is None else mx < 100, "ms",
              "gồm thời gian tụ servo xả qua ngưỡng opto; KHÔNG thay oscilloscope")
    _scope_bang(ctx)
    return ctx.finish()


def _jig_bang(ctx, args, sim=None) -> int:
    """Thử bàn không Pi: K5 bằng công tắc tay, đi từng dòng bảng trạng thái (bang-di-day-evt.md mục 2, mục 6.3)."""
    steps = [("k5_tat_khong_chot", "K5 tắt, nhấn/nhả Reset: K2 không hút, SERVO+ = 0?"),
             ("nap_truoc", "Bật K5, nhấn và GIỮ Reset: K2 hút, K4 đóng nạp trước, sau ~0,5 s K4 nhả, K1 vẫn hở?"),
             ("nha_reset_k1_dong", "Nhả Reset: K1 đóng, SERVO+ bằng điện áp vào (sụt ≤ 0,2 V)?"),
             ("estop_cat", "Nhấn E-stop: K1 và K2 nhả, SERVO+ về 0?"),
             ("nha_estop_khong_cap_lai", "Nhả E-stop, K5 vẫn bật: SERVO+ KHÔNG có lại điện?"),
             ("reset_ket", "Giữ Reset > 2 s (giả kẹt): K1 không đóng, SERVO+ về 0 sau ~0,5 s?"),
             ("tat_k5", "Nhả Reset cho K1 đóng, rồi tắt K5: K1 nhả?"),
             ("mat_12v", "Bật K5, Reset cho K1 đóng, gạt Q6: K1 nhả; bật lại Q6: KHÔNG có lại điện?"),
             ("dut_day", "Reset cho K1 đóng, tháo một dây W14: K1 nhả?")]
    ok_cycles = 0
    for k in range(1, args.chu_ky + 1):
        simr = _jig_bang_sim(ctx, sim) if sim is not None else {}
        res = {key: ctx.ask(f"c{k}_{key}", f"Chu kỳ {k}: {q} (c/k)", simr.get(key), bool) for key, q in steps}
        ok_cycles += all(v is True for v in res.values())
        ctx.data[f"chu_ky_{k}"] = res
    ctx.check("Q-IPQC-09", "chuỗi chốt reset bộ #0, thử bàn 12 V", f"{ok_cycles}/{args.chu_ky}", f"{args.chu_ky}/{args.chu_ky}", ok_cycles == args.chu_ky)
    ctx.check("EVT-03", "nhả không tự cấp, Reset kẹt, K5 cần cho phép", f"{ok_cycles}/{args.chu_ky}", "tất cả", ok_cycles == args.chu_ky)
    _scope_bang(ctx)
    return ctx.finish()


def _jig_bang_sim(ctx, s):
    """Chạy khô: lấy câu trả lời từ mạch giả đi đúng chuỗi thao tác (không mặc định 'có')."""
    def wait(t):
        ctx.sleep(t)
        s.read("ARM_SENSE")
    on = lambda: s.k1 and s.arm                     # noqa: E731
    r = {}
    s.write("K5_EN", False)
    s.click_reset()
    wait(0.6)
    r["k5_tat_khong_chot"] = not s.k2 and not s.arm
    s.write("K5_EN", True)
    s.press("reset", True)
    wait(0.1)
    pre = s.k2 and s.k4 and not s.k1
    wait(0.6)
    r["nap_truoc"] = pre and not s.k4 and not s.k1
    s.press("reset", False)
    wait(0.05)
    r["nha_reset_k1_dong"] = on()
    s.press("estop", True)
    wait(0.1)
    r["estop_cat"] = not s.k1 and not s.k2 and not s.arm
    s.press("estop", False)
    wait(1.0)
    r["nha_estop_khong_cap_lai"] = not s.arm
    s.press("reset", True)
    wait(2.0)
    r["reset_ket"] = not s.k1 and not s.arm
    s.press("reset", False)
    wait(0.05)
    ok = on()
    s.write("K5_EN", False)
    wait(0.1)
    r["tat_k5"] = ok and not s.arm
    s.write("K5_EN", True)
    s.click_reset()
    wait(0.6)
    ok = on()
    s.press("v12", True)
    wait(0.1)
    off = not s.arm
    s.press("v12", False)
    wait(1.0)
    r["mat_12v"] = ok and off and not s.arm
    s.click_reset()
    wait(0.6)
    ok = on()
    s.press("wire", True)
    wait(0.1)
    r["dut_day"] = ok and not s.arm
    s.press("wire", False)
    s.write("K5_EN", False)
    wait(0.1)
    return r


def _scope_bang(ctx):
    """Số oscilloscope và đồng hồ cho bộ #0 (người đo nhập). Không có U4 nên không đo POR."""
    t = ctx.ask("estop_cat_ms_max", "Oscilloscope: nhấn nút -> SERVO+ < 1,2 V, lớn nhất 10 lần (ms)", 42.0)
    ctx.check("Q-FQC-01", "E-stop cắt điện servo (oscilloscope, 10 lần, lớn nhất)", t, "< 100 ms", None if t is None else t < 100, "ms")
    ctx.check("EVT-02", "E-stop cắt điện servo (SOR-011)", t, "< 100 ms", None if t is None else t < 100, "ms")
    k1 = ctx.ask("k1_nha_ms", "Oscilloscope: thời gian nhả K1 (cuộn mất điện -> tiếp điểm hở) (ms)", 18.0)
    ctx.check("EVT-04", "thời gian nhả K1 (oscilloscope)", k1, "< 100 ms", None if k1 is None else k1 < 100, "ms")
    v = ctx.ask("v_servo_sau_1s", "Điện áp SERVO+ sau E-stop 1 s (V)", 0.0)
    ctx.check("Q-FQC-02", "điện áp servo sau E-stop 1 s", v, "0 V (< 0,1 V)", None if v is None else v < 0.1, "V")
    kt = ctx.ask("kt_tre_ms", "Oscilloscope: RUN lên -> K4 nhả (trễ KT) (ms)", 500.0)
    ctx.check("EVT-12", "trễ KT nạp trước (bộ #0 thay POR U4)", kt, "ghi nhận, đặt 0,5 s", None, "ms")
    ctx.check("EVT-12", "POR U4 / LATCH_SENSE kéo /CLR", "không áp dụng bộ #0 (không có U4)", "đo trên PCB sau EVT", None)


def _xa_tu(ctx, args) -> int:
    """Đo thời gian xả tụ servo qua ngưỡng opto ARM_SENSE khi K4 nhả mà K1 không đóng (Reset giữ)."""
    from faibotic import spec
    ctx.confirm("Servo ĐÃ cắm hub, mô-men tắt; tay buộc dây chặn; một người giữ E-stop; kỹ sư điện đã xem sơ đồ")
    sim = SimBangDiDay(clock=ctx.now, cap_hold_s=args.tu_giu_s) if ctx.dry else None
    g = ctx.gpio(lambda: sim)
    try:
        ms = []
        for k in range(1, 4):
            g.write("K5_EN", False)
            ctx.sleep(0.5)
            if g.read("ARM_SENSE") or g.read("ESTOP_OK"):
                ctx.say("ARM_SENSE hoặc ESTOP_OK cao khi K5 hạ: dừng, kiểm K1/K4/K2")
                break
            g.write("K5_EN", True)                    # chỉ cho phép; K2 chốt khi người nhấn Reset
            ctx.action("Nhấn và GIỮ nút Reset tới khi được báo nhả (K1 không đóng khi Reset đang giữ)",
                       (lambda: sim.press("reset", True)) if sim else None)
            t0 = wait_level(ctx, g, "ESTOP_OK", True, args.timeout)
            t1 = wait_level(ctx, g, "ARM_SENSE", False, 10.0) if t0 is not None else None
            g.write("K5_EN", False)                   # K2 nhả trước khi người nhả Reset: K1 không đóng
            ctx.action("Nhả nút Reset", (lambda: sim.press("reset", False)) if sim else None)
            if t1 is None:
                ctx.say(f"lần {k}: ARM_SENSE không rớt trong 10 s (K4 dính hoặc tụ giữ quá lâu)")
                ms.append(None)
                break
            ms.append(round(1000 * (t1 - t0) - 1000 * spec.EVT_KT_S, 1))
            ctx.say(f"lần {k}: xả {ms[-1]} ms sau K4 nhả")
    finally:
        g.write("K5_EN", False)                   # mọi đường thoát (lỗi, Abort, Ctrl-C): K5 hạ
    ctx.data["xa_tu_ms"] = ms
    mx = None if not ms or None in ms else max(ms)
    lim = 1000 * spec.EVT_ARM_CONFIRM_S / 2
    ctx.check("EVT-11", "thời gian xả tụ servo qua ngưỡng ARM_SENSE (có servo, Reset giữ, lớn nhất 3 lần)", mx,
              f"≤ {lim:.0f} ms (EVT_ARM_CONFIRM_S / 2)", None if mx is None else mx <= lim, "ms",
              "đo mốc từ RUN lên + KT danh định; truyền --xa-tu-ms cho kịch bản bật mô-men")
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
