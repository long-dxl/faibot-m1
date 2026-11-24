"""Quét bus servo: ID, model, firmware, điện áp, nhiệt (EVT-01, EVT-16; Q-IQC-01..05; Q-IPQC-06).

Chế độ:
  --che-do bo     bus robot đủ 20 servo (sau giai đoạn 3B; Q-IPQC-06)
  --che-do 16     bus robot trước 3B: ID 1–16, cáp 4 servo B (17–20) rút (06-lap-rap bước 9.1, ghi "16 + 4 riêng")
  --che-do rieng  một servo cắm riêng vào bộ test (IQC từng servo), --id ID dự kiến; --quay đo dòng không tải 10 s;
                  --quet-vi-tri quét 20 điểm 0–360° (Q-IQC-05). Servo kẹp ê tô má mềm, tay không chạm horn.
Ví dụ: python quet_bus.py --dry-run ; python quet_bus.py --port /dev/ttyUSB0 --che-do rieng --id 17 --quay --serial FM1-0000-000 --nguoi-do "..."
"""
from __future__ import annotations

import json
import statistics

import evtlib as E
from evtlib import P, spec

NAME_OF = {v: k for k, v in E.EVT_IDS.items()}


def add_args(ap):
    ap.add_argument("--che-do", choices=("bo", "16", "rieng"), default="bo")
    ap.add_argument("--id", type=int, default=None, help="ID dự kiến khi --che-do rieng")
    ap.add_argument("--quay", action="store_true", help="rieng: quay ±10° chậm 10 s, đọc dòng/nhiệt (Q-IQC-03)")
    ap.add_argument("--quet-vi-tri", action="store_true", help="rieng: quét 20 điểm 0..4095 (Q-IQC-05)")
    ap.add_argument("--v-nguon", type=float, default=12.0, help="điện áp nguồn DC đặt (V)")
    ap.add_argument("--quet-den", type=int, default=40, help="quét ID 0..N để bắt ID lạ (253 = toàn dải, chậm)")
    ap.add_argument("--model-3095", type=int, default=None, help="số model STS3095 chuẩn (chốt từ mẫu đầu)")
    ap.add_argument("--model-3215", type=int, default=None)


def expected_ids(args) -> dict[int, str]:
    if args.che_do == "bo":
        return dict(NAME_OF)
    if args.che_do == "16":
        return {i: n for i, n in NAME_OF.items() if i <= 16}
    if args.id is None:
        raise SystemExit("--che-do rieng cần --id")
    return {args.id: NAME_OF.get(args.id, f"id{args.id}")}


def main(args) -> int:
    exp = expected_ids(args)
    mock = dict(ids={n: i for i, n in exp.items()})
    ctx = E.Ctx(args, "quet_bus", ["EVT-01", "EVT-16", "Q-IQC-01", "Q-IQC-02", "Q-IQC-03", "Q-IQC-05", "Q-IPQC-06"], mock)
    ctx.confirm(f"Nguồn DC giới hạn dòng (≤ 5 A khi quét bus robot; 1 servo: ≤ 2 A), {args.v_nguon} V; E-stop đã thử đạt (06-lap-rap 6.6)")
    bus = ctx.bus()
    bus.retries = 0
    found = {}
    for sid in sorted(set(range(0, min(args.quet_den, 253) + 1)) | set(exp)):
        if bus.ping(sid):
            r = {}
            for key, reg in (("fw_major", E.FW_MAJOR), ("fw_minor", E.FW_MINOR), ("model", E.MODEL_NUMBER), ("volt_raw", P.PRESENT_VOLTAGE),
                             ("temp_c", P.PRESENT_TEMPERATURE), ("mode", P.MODE), ("torque", P.TORQUE_ENABLE), ("pos_raw", P.PRESENT_POSITION)):
                try:
                    r[key] = bus.read(sid, reg)
                except (TimeoutError, P.ProtocolError) as e:
                    r[key] = f"lỗi: {e}"
            r["vi_tri"] = exp.get(sid, "ID LẠ")
            r["loai_du_kien"] = E.servo_type(exp[sid]) if sid in exp else "?"
            found[sid] = r
            ctx.say(f"ID {sid:3} {r['vi_tri']:18} model {r['model']} fw {r['fw_major']}.{r['fw_minor']} "
                    f"{r['volt_raw'] * E.VOLT_PER_LSB if isinstance(r['volt_raw'], int) else r['volt_raw']} V {r['temp_c']} °C mode {r['mode']}")
    ctx.data["servo"] = found
    ctx.data["che_do"] = args.che_do
    missing = sorted(set(exp) - set(found))
    stray = sorted(set(found) - set(exp))
    label = {"bo": "đủ 20", "16": "16 (ID 1–16, servo B rút)", "rieng": f"ID {args.id}"}[args.che_do]
    ctx.check("Q-IPQC-06", f"ID trên bus: {label}", f"{len(found) - len(stray)}/{len(exp)}", "đủ, đúng ID, không ID lạ",
              not missing and not stray, note=(f"thiếu {missing} " if missing else "") + (f"lạ {stray}" if stray else ""))
    # model và firmware theo loại
    for typ, std in (("STS3095", args.model_3095), ("STS3215", args.model_3215)):
        rows = [r for r in found.values() if r["loai_du_kien"] == typ and isinstance(r["model"], int)]
        if not rows:
            continue
        models = sorted({r["model"] for r in rows})
        ok = len(models) == 1 and (std is None or models[0] == std)
        ctx.check("Q-IQC-01", f"model {typ} đồng nhất" + ("" if std else " (chưa có số chuẩn)"), models, f"1 giá trị{' = ' + str(std) if std else ''}", ok)
        fws = sorted({f"{r['fw_major']}.{r['fw_minor']}" for r in rows})
        ctx.check("Q-IQC-02", f"firmware {typ} trùng nhau", fws, "1 phiên bản trong lô", len(fws) == 1)
    m95 = {r["model"] for r in found.values() if r["loai_du_kien"] == "STS3095"}
    m15 = {r["model"] for r in found.values() if r["loai_du_kien"] == "STS3215"}
    if m95 and m15:
        ctx.check("Q-IQC-01", "STS3095 và STS3215 khác model (bắt lắp nhầm vị trí)", f"{sorted(m95)} / {sorted(m15)}", "không giao nhau", not (m95 & m15))
    vs = [r["volt_raw"] * E.VOLT_PER_LSB for r in found.values() if isinstance(r["volt_raw"], int)]
    if vs:
        dv = max(abs(v - args.v_nguon) for v in vs)
        ctx.check("Q-IQC-03", "điện áp đọc qua bus lệch nguồn lớn nhất", dv, "ghi nhận (tiêu chí QC không có ngưỡng)", None, "V",
                  f"thấp nhất {min(vs):.1f} V")
    if args.che_do == "rieng" and args.id in found:
        _single(ctx, bus, args)
    if args.che_do == "rieng":
        _lot(ctx, args)
    return ctx.finish()


def _single(ctx, bus, args):
    sid = args.id
    if not (args.quay or args.quet_vi_tri):
        return
    ctx.confirm("Thân servo kẹp ê tô má mềm, horn tự do không gắn tải, tay không chạm horn")
    E.require_mode0(bus, sid)
    try:
        bus.write(sid, P.TORQUE_LIMIT, 300)
        bus.write(sid, P.GOAL_SPEED, 400)
        p0 = bus.read(sid, P.PRESENT_POSITION)
        p0 = min(max(p0, 200), 3895)
        bus.write(sid, P.GOAL_POSITION, p0)
        bus.write(sid, P.TORQUE_ENABLE, 1)
        if args.quay:
            cur, raw, temps, t0, k = [], [], [], ctx.now(), 0
            while ctx.now() - t0 < 10.0:
                bus.write(sid, P.GOAL_POSITION, p0 + (114 if k % 2 == 0 else -114))   # ±10°
                for _ in range(10):
                    ctx.sleep(0.05)
                    raw.append(bus.read(sid, P.PRESENT_CURRENT))
                    cur.append(raw[-1] * E.CURRENT_MA_PER_LSB)
                temps.append(bus.read(sid, P.PRESENT_TEMPERATURE))
                k += 1
            ctx.data["dong_khong_tai_ma"] = statistics.median(cur)
            ctx.data["dong_khong_tai_raw"] = dict(trung_vi=statistics.median(raw), dinh=max(raw), mau=raw)
            ctx.check("Q-IQC-03", f"ID {sid} dòng không tải (trung vị 10 s)", statistics.median(cur), "so trung vị lô ±30% (mục lô)", None, "mA",
                      f"đỉnh {max(cur):.0f} mA, nhiệt {max(temps)} °C; quy đổi {E.CURRENT_MA_PER_LSB} mA/LSB [giả định]")
            ok = ctx.ask("quay_em", "Servo quay êm, không kêu lạ, không báo lỗi? (c/k)", True, bool)
            ctx.check("Q-IQC-03", f"ID {sid} quay êm, không báo lỗi", ok, "có", ok)
        if args.quet_vi_tri:
            errs, reads = [], []
            for k in range(20):
                goal = int(100 + k * (3895 / 19))
                bus.write(sid, P.GOAL_POSITION, goal)
                ctx.sleep(0.6)
                got = bus.read(sid, P.PRESENT_POSITION)
                reads.append(got)
                errs.append(abs(got - goal) * 360 / 4096)
            mono = all(b > a for a, b in zip(reads, reads[1:]))
            ctx.data["quet_vi_tri"] = reads
            ctx.check("Q-IQC-05", f"ID {sid} sai khác lệnh–đọc lớn nhất (20 điểm)", max(errs), "≤ 2° [giả định], đơn điệu", max(errs) <= 2.0 and mono, "°",
                      "" if mono else "KHÔNG đơn điệu")
    finally:
        bus.write(sid, P.TORQUE_ENABLE, 0)


def _lot(ctx, args):
    """Gộp các lần quét riêng trong thư mục serial: trung vị lô dòng không tải và số mẫu (EVT-16: ≥ 5 mẫu)."""
    cur = {}
    for f in sorted(ctx.result_dir().glob("quet_bus_*.json")):
        if ("_dryrun" in f.name) != ctx.dry:
            continue
        d = json.loads(f.read_text(encoding="utf-8"))
        if d.get("du_lieu", {}).get("che_do") != "rieng":
            continue
        for sid in d.get("du_lieu", {}).get("servo", {}):
            cur.setdefault(int(sid), None)
        if d["du_lieu"].get("dong_khong_tai_ma") is not None:
            cur[int(next(iter(d["du_lieu"]["servo"])))] = d["du_lieu"]["dong_khong_tai_ma"]
    if "dong_khong_tai_ma" in ctx.data:
        cur[args.id] = ctx.data["dong_khong_tai_ma"]
    cur.setdefault(args.id, None)
    vals = [v for v in cur.values() if v is not None]
    n = len(cur)
    ctx.check("EVT-16", "số servo đã qua IQC riêng (trong thư mục serial)", n, "≥ 5 mẫu", True if n >= 5 else None, note="" if n >= 5 else "đang tích luỹ: chạy tiếp các servo còn lại")
    if len(vals) >= 3 and args.id in cur and cur[args.id] is not None:
        med = statistics.median(vals)
        dev = abs(cur[args.id] - med) / med if med else 0
        ctx.check("Q-IQC-03", f"ID {args.id} dòng không tải so trung vị lô {med:.0f} mA", dev * 100, "≤ 30% [giả định]", dev <= 0.30, "%")


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
