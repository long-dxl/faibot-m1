# M1 trong mô phỏng: bắt đầu nhanh

M1 hiện là robot dạng số (mô phỏng MuJoCo + SDK), chưa phải robot thật.

## Cài và chạy (macOS hoặc Linux, cần Python 3.10+)

```bash
cd faibotic-m1   # thư mục đã clone repo
./run.sh              # lần đầu tự tạo .venv (khoảng 1–2 phút), rồi mở cửa sổ mô phỏng
./run.sh --selftest   # tự kiểm không mở cửa sổ, in JSON, mã thoát 0 = đạt
./run.sh --repl       # gõ lệnh SDK, xem robot chạy trong cửa sổ
./run.sh --demo       # chạy demo gắp đặt, kiểm toạ độ, in JSON
./run.sh --test       # pytest
```

## Phím trong cửa sổ (bấm vào cửa sổ trước)

| Phím | Việc |
|---|---|
| W / S | đế tiến / lùi (giữ phím = chạy, nhả = dừng) |
| A / D | đế sang trái / phải (bánh omni) |
| Q / E | đế xoay trái / phải |
| R / F | trục nâng lên / xuống |
| 1 / 2 | chọn tay trái / phải |
| mũi tên hoặc I K J L | đầu kẹp tới / lui / trái / phải (IK) |
| PgUp PgDn hoặc U O | đầu kẹp lên / xuống |
| Space | đóng / mở kẹp |
| H | về tư thế gốc |
| Backspace | đặt lại cảnh |
| P | chạy demo gắp đặt tự động (Esc dừng) |
| C | đổi camera (tự do, tổng quan, bám đế, camera đầu, 2 camera cổ tay) |
| Tab | ẩn / hiện bảng phím |
| Esc | thoát |

Chuột: kéo trái để xoay, kéo phải để dời, cuộn để thu phóng. Chữ trên cửa sổ không dấu vì font của MuJoCo chỉ có ASCII.

## Năm lệnh SDK (trong `./run.sh --repl`, đã có sẵn biến `robot`)

```python
robot.move_base(dx=0.5, duration=2)                       # đế tiến 0,5 m
robot.move_joints({"lift": 0.35}, duration=1.5)           # nâng trục lên 0,35 m (trục vai 0,90 m)
robot.move_tcp("left", [0.35, 0.10, 0.80], duration=2)    # kẹp trái tới điểm (khung robot, m); chạy sau lệnh nâng trục ở dòng trên, không thì báo "ngoài tầm với"
robot.set_gripper("left", 100); robot.wait(0.5)           # mở kẹp (0 = đóng)
robot.get_state()                                         # khớp, trục nâng, kẹp, tư thế đế
```

Cùng code chạy với robot thật bằng `Robot(backend="feetech", port="/dev/ttyUSB0")` khi đã lắp và được phép chạy (backend này mới test với bus giả).
