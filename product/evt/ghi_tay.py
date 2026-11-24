"""Ghi số đo dụng cụ (không cần bus) cho các tiêu chí còn lại: EVT-05 (khởi động/chặn rotor), EVT-06, EVT-08, EVT-10,
EVT-11 (xung nạp trước), EVT-13, EVT-14 (tốc độ đế), EVT-15, EVT-16 (mô-men siết), EVT-17, EVT-18;
Q-FQC-06 (tốc độ vai, tính từ nhật ký bus), Q-FQC-C2, C3, C7, C8 (hành trình, độ cao vai), C9, C10, C11.

  python ghi_tay.py --ma EVT-13 --serial FM1-0000-000 --nguoi-do "..."     (chỉ một mã; mặc định tất cả)
Ngưỡng chép từ readiness-checklist.md và bộ tiêu chí QC, không đặt ngưỡng mới (C9 ≥ 6 kg, đã đồng bộ SOR-003; chờ QC2 cột nâng duyệt). Bỏ trống = chưa kiểm (KHÔNG ĐẠT khi ký).
"""
from __future__ import annotations

import evtlib as E

le = lambda x: (lambda v: v <= x)   # noqa: E731
ge = lambda x: (lambda v: v >= x)   # noqa: E731
yes = lambda v: bool(v)             # noqa: E731

# (khoá, mã, hạng mục, đơn vị, ngưỡng, hàm đạt hoặc None = ghi nhận, kiểu, giá trị mô phỏng)
FIELDS = [
    ("i_khoi_dong_a", "EVT-05", "dòng khởi động (xung khi K1 đóng, kẹp dòng/oscilloscope)", "A", "ghi nhận", None, float, 9.0),
    ("i_chan_rotor_a", "EVT-05", "dòng chặn rotor một STS3095 (servo rời, 12 V)", "A", "ghi nhận (hãng 9,8 A)", None, float, 9.5),
    ("r_moi_noi_mohm", "EVT-06", "R tổng pack + BMS + K1 + F1 + D0 (đo từng mối, cộng)", "mΩ", "thay giả định 10,31 mΩ", None, float, 9.8),
    ("sut_ap_47a_mv", "Q-FQC-C10", "sụt áp trục chính ở tải 47,2 A", "mV", "≤ 333 mV", le(333), float, 320.0),
    ("can_bang_sor002", "EVT-08", "cân bằng trọng lực vai đạt SOR-002 (thiết kế duyệt + thử tải)", "", "có", yes, bool, True),
    ("nhiet_day_498a", "EVT-10", "nhiệt dây 2 AWG / 4 AWG / PTC / busbar sau 30 phút ở 49,8 A (nóng nhất)", "°C", "ghi nhận (chưa có ngưỡng trong checklist)", None, float, 48.0),
    ("bms_k1_nhiet", "Q-FQC-C11", "nhiệt BMS và contactor sau 10 phút 50 A, BMS không cắt", "°C", "≤ 60 °C [giả định]", le(60), float, 47.0),
    ("xung_nap_truoc_a", "EVT-11", "dòng xung nạp trước (đỉnh)", "A", "ghi nhận", None, float, 6.0),
    ("c_servo_mf", "EVT-11", "điện dung tổng servo thật (đo)", "mF", "ghi nhận (giả định 4,4 mF)", None, float, 4.1),
    ("k1_tre_dong_ms", "EVT-11", "trễ K1 đóng (bộ #0: RUN lên -> K1, KT 0,5 s; PCB: sau cạnh RELAY_EN, Q1 thật)", "ms", "≥ ~170 ms (QC2-sdk-evt)", ge(170), float, 190.0),
    ("khoi_luong_kg", "EVT-13", "khối lượng robot", "kg", "≤ 15 kg", le(15), float, 14.2),
    ("tay_kg", "EVT-13", "khối lượng một tay", "kg", "1,042 kg ± 10% (v0.2: 1,076)", lambda v: abs(v - 1.042) <= 0.1042, float, 1.05),
    ("toc_do_de_ms", "EVT-14", "tốc độ đế tịnh tiến (5 m, 3 hướng, chậm nhất)", "m/s", "≥ 0,3 m/s", ge(0.3), float, 0.33),
    ("toc_do_quay", "Q-FQC-C7", "tốc độ quay đế lớn nhất (lệnh quay tại chỗ; SDK kẹp 0,25 rad/s theo thông số)", "rad/s", "≤ 0,26 rad/s (thông số 0,25 rad/s)", le(0.26), float, 0.25),
    ("nang_6kg", "Q-FQC-C9", "nâng tổng tải ≥ 6 kg (hai tay + quả cân), 20 chu kỳ, không trượt, không mất bước (sau lift_da_vong)", "", "có (≥ 6 kg)", yes, bool, True),
    ("toc_do_vai_dps", "Q-FQC-06", "tốc độ vai mang 0,5 kg: lệnh quay 60°, góc / thời gian từ nhật ký bus (chậm nhất hai tay)", "°/s", "≥ 30°/s", ge(30), float, 34.0),
    ("tam_voi_trai_mm", "Q-FQC-C2", "tầm với tay trái: duỗi ngang, thước từ trục vai tới điểm kẹp", "mm", "524 mm ± 5 mm", lambda v: abs(v - 524) <= 5, float, 523.0),
    ("tam_voi_phai_mm", "Q-FQC-C2", "tầm với tay phải: duỗi ngang, thước từ trục vai tới điểm kẹp", "mm", "524 mm ± 5 mm", lambda v: abs(v - 524) <= 5, float, 525.0),
    ("tay_trai_c3_kg", "Q-FQC-C3", "khối lượng tay trái (không thân xoay, không camera, WaveShare)", "kg", "1,042 kg ± 10%", lambda v: abs(v - 1.042) <= 0.1042, float, 1.05),
    ("tay_phai_c3_kg", "Q-FQC-C3", "khối lượng tay phải (không thân xoay, không camera, WaveShare)", "kg", "1,042 kg ± 10%", lambda v: abs(v - 1.042) <= 0.1042, float, 1.05),
    ("hanh_trinh_m", "Q-FQC-C8", "hành trình nâng giữa hai công tắc, đo bằng thước", "m", "ghi nhận, so 0,50 m (chưa có dung sai trong tiêu chí QC)", None, float, 0.50),
    ("vai_thap_m", "Q-FQC-C8", "độ cao trục vai ở lift 0, từ sàn", "m", "ghi nhận, so 0,55 m (chưa có dung sai trong tiêu chí QC)", None, float, 0.55),
    ("vai_cao_m", "Q-FQC-C8", "độ cao trục vai ở lift 0,50 m, từ sàn", "m", "ghi nhận, so 1,05 m (chưa có dung sai trong tiêu chí QC)", None, float, 1.05),
    ("chay_pin_phut", "EVT-15", "chạy pin teleop điển hình tới BMS cắt / 9,6 V", "phút", "≥ 120 phút", ge(120), float, 128.0),
    ("mo_men_truot_5mau", "EVT-16", "mô-men trượt trung bình 5 mẫu in (hạt chèn M3)", "N·m", "ghi nhận; siết = 50%", None, float, 1.1),
    ("review_ky", "EVT-17", "REVIEW.md điện có chữ ký kỹ sư điện; PCB rev 1 đã chạy", "", "có", yes, bool, True),
    ("eco_rev_b", "EVT-18", "ECO-log có dòng đóng băng rev B; rev B trong parts-register", "", "có", yes, bool, True),
]


def add_args(ap):
    ap.add_argument("--ma", default=None, help="lọc theo mã, ví dụ EVT-13")


def main(args) -> int:
    ctx = E.Ctx(args, "ghi_tay", sorted({f[1] for f in FIELDS if not args.ma or f[1] == args.ma}), dict(ids={}))
    rows = [f for f in FIELDS if not args.ma or f[1] == args.ma]
    if not rows:
        raise SystemExit(f"không có mã {args.ma}; có: {sorted({f[1] for f in FIELDS})}")
    dung_cu = ctx.ask("dung_cu", "Dụng cụ đo dùng (mã, số hiệu, hạn hiệu chuẩn)", "mô phỏng", str)
    ctx.data["dung_cu"] = dung_cu
    for key, code, item, unit, thr, fn, kind, sim in rows:
        v = ctx.ask(key, f"{code} {item} ({unit or 'c/k'})", sim, kind)
        ctx.check(code, item, v, thr, None if v is None or fn is None else bool(fn(v)), unit)
    return ctx.finish()


if __name__ == "__main__":
    raise SystemExit(E.run(main, desc=__doc__, add_args=add_args))
