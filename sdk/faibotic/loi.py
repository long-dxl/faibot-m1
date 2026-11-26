# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Lớp lỗi có mã của SDK M1 (Tầng 1b, docs/S4/KE-HOACH-TANG-1B.md mục 2.5; bảng mã ở sdk/faibotic/loi.py).

Mọi lỗi SDK chủ động ném kế thừa `M1Loi` (có `.ma` và `.khop`) và giữ lớp chuẩn quen dùng (ConnectionError, ValueError, TypeError,
KeyError, RuntimeError) để code cũ bắt theo lớp chuẩn vẫn chạy. Lỗi được ghi qua `logging.getLogger("faibotic")` dạng `[MÃ] câu`.

Bảng mã (core plugin phải dùng cùng mã):
  BUS-01 BusMat          cổng nối tiếp mất giữa phiên (rút USB, OSError)          -> chốt E-stop bị cắt, enabled=False
  BUS-02 ServoMat        servo trả gói hỏng (checksum) N chu kỳ liền              -> TTAT
  BUS-03 ServoMat        servo không trả lời N chu kỳ liền                       -> TTAT
  BUS-04 BusMat          bus im lặng: mọi servo không trả lời N chu kỳ liền       -> TTAT
  BUS-05 BusMat          không mở được cổng / cổng đang được chương trình khác dùng
  BUS-06 BusCauHinhSai   thiếu servo lúc kết nối
  BUS-07 BusCauHinhSai   trùng ID (gói trả chồng nhau)
  SER-01 ServoBaoLoi     servo bật bit lỗi (quá nhiệt, quá tải, quá dòng, cảm biến góc)
  SER-02 PairFault       cặp servo ghép đôi lệch/im lặng/báo lỗi
  PWR-01 DienApThap      điện áp đo dưới spec.V_CAT
  PWR-02 DienApThap      servo bật bit lỗi điện áp
  SYS-01 RobotDaDong     lệnh sau close()
  SYS-02 RobotBan        luồng khác đang điều khiển robot
  SYS-03 WatchdogHet     không có step() quá spec.WATCHDOG_S khi đế đang chạy
  SAF-01 EStopActive     chốt E-stop không ở trạng thái chạy
  SAF-02 DockOpen        khoá trạm mở
  SAF-03 OutOfSoftLimits tay ngoài giới hạn mềm khi enable
  SAF-04 KeepoutError    đích hoặc đường đi vào vùng cấm trạm
  LIFT-01 LiftFault      chốt/kẹt trục nâng;  LIFT-02 LiftUnderpowered
  PWR-03 CurrentBudgetExceeded
  CMD-01 LenhKhongHopLe  giá trị lệnh không hữu hạn / ngoài miền (ValueError)
  CMD-02 KieuLenhSai     kiểu lệnh sai (TypeError)
  CMD-03 KhopKhongCo     tên khớp không có (KeyError)
  CFG-01 HieuChuanSai    hiệu chuẩn sai (ValueError)
  CFG-02 CauHinhSai      bảng ID / cấu hình sai (ValueError)
"""
from __future__ import annotations

import logging

log = logging.getLogger("faibotic")


class M1Loi(RuntimeError):
    """Lỗi có mã của SDK M1. `.ma`: mã (vd "BUS-01"); `.khop`: tên khớp/servo liên quan (hoặc None)."""

    ma = "M1-00"
    muc = logging.ERROR          # mức ghi log; lỗi đầu vào ghi INFO để không làm ồn

    def __init__(self, msg: str = "", *, ma: str | None = None, khop: str | None = None):
        super().__init__(msg)
        if ma is not None:
            self.ma = ma
        self.khop = khop
        log.log(self.muc, "[%s] %s", self.ma, msg)


# ---------- bus và servo ----------
class BusMat(M1Loi, ConnectionError):
    """Mất bus servo (cổng mất, bus im lặng, không mở được cổng)."""
    ma = "BUS-01"


class BusCauHinhSai(M1Loi, ConnectionError):
    """Bus không đúng cấu hình lúc kết nối (thiếu servo, trùng ID)."""
    ma = "BUS-06"


class ServoMat(M1Loi):
    """Một servo không trả lời hoặc trả gói hỏng nhiều chu kỳ liền."""
    ma = "BUS-03"


class ServoBaoLoi(M1Loi):
    """Servo bật bit lỗi (quá nhiệt, quá tải, quá dòng, cảm biến góc)."""
    ma = "SER-01"


class DienApThap(M1Loi):
    """Điện áp servo dưới spec.V_CAT hoặc servo bật bit lỗi điện áp."""
    ma = "PWR-01"


# ---------- vòng đời ----------
class RobotDaDong(M1Loi):
    """Robot/backend đã close(): không nhận lệnh nữa."""
    ma = "SYS-01"


class RobotBan(M1Loi):
    """Robot đang do một luồng khác điều khiển (một chủ sở hữu)."""
    ma = "SYS-02"


class WatchdogHet(M1Loi):
    """Máy chủ không gọi step() quá spec.WATCHDOG_S khi đế đang chạy: luồng giám sát đã dừng bánh và tắt mô-men."""
    ma = "SYS-03"


# ---------- đầu vào và cấu hình ----------
class LenhKhongHopLe(M1Loi, ValueError):
    ma = "CMD-01"
    muc = logging.INFO


class KieuLenhSai(M1Loi, TypeError):
    ma = "CMD-02"
    muc = logging.INFO


class KhopKhongCo(M1Loi, KeyError):
    ma = "CMD-03"
    muc = logging.INFO

    def __str__(self) -> str:          # KeyError mặc định in repr có nháy; giữ câu đọc được
        return str(self.args[0]) if self.args else ""


class HieuChuanSai(M1Loi, ValueError):
    ma = "CFG-01"


class CauHinhSai(M1Loi, ValueError):
    ma = "CFG-02"
