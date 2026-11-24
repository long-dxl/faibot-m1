"""Khai báo vật trên khay đế (docs/M1-TRAM/R5-khay-de.md).

Khay đế là sàn thiết bị: laptop (<= 2,5 kg) HOẶC Jetson kèm DC-DC, cộng phụ kiện, tổng <= spec.KHAY_DE_TAI_MAX. Tay robot không
đặt vật lên khay. Khi người dùng khai báo vật trên khay, SDK:
  - từ chối khai báo ngoài vùng cho phép (tâm khối x trong spec.KHAY_DE_TAM_X, |y| <= spec.KHAY_DE_TAM_Y, tải <= định mức): CauHinhSai (CFG-02);
  - đặt giới hạn mềm dưới của lift (lift_min): >= spec.KHAY_DE_LIFT_MIN, cao hơn nếu đỉnh vật cần khe spec.KHAY_DE_KHE tới đáy xe nâng;
  - nâng vùng cấm tâm kẹp trên khay lên đỉnh vật + spec.KEEPOUT_TRAY_MARGIN.
Dốc đế mặc định giữ nguyên: trong vùng cho phép, hệ số chống lật động với tay theo CAD vẫn >= 1,5 (tests/test_khay_tai.py kiểm các góc vùng).

    >>> from faibotic import Robot, KhayTai
    >>> robot = Robot("sim", khay=KhayTai("laptop", kg=2.0, x=0.19, y=0.0))
    >>> robot.set_khay_tai(None)          # bỏ khai báo khi đã nhấc vật xuống
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from . import spec
from .loi import CauHinhSai

# Mặc định theo loại: (kg, chiều cao đỉnh vật trên mặt khay m). Jetson gồm DC-DC 0,15 kg và đệm chống rung 8 mm. [ước]
LOAI = {
    "laptop": (2.5, 0.025),            # gập nắp; mở nắp khi chạy thì truyền cao=0,26
    "jetson-nano": (0.75, 0.068),      # Orin Nano/NX devkit + quạt
    "jetson-agx": (1.25, 0.080),       # AGX Orin devkit
    "khac": (None, None),              # phụ kiện: phải ghi kg và cao
}
# Đáy thấp nhất của xe nâng + trạm ở lift 0 (khung T giữa ôm trụ, spec.TMID_Z) [tính từ spec; chờ quét mesh xác nhận]
Z_DAY_XE_NANG_LIFT0 = spec.ARM_BASE_Z0 + min(spec.TMID_Z[0], spec.DOCK_Z[0], spec.STATION_EAR_Z[0], spec.STATION_RAIL_Z[0])
MAT_KHAY = spec.TRAY_POS[2] + spec.TRAY_HALF[2] - 0.010      # mặt tấm khay (dưới gờ 10 mm), 0,210 m


@dataclass(frozen=True)
class KhayTai:
    """Vật trên khay đế. loai: "laptop" | "jetson-nano" | "jetson-agx" | "khac". kg: tổng khối lượng trên khay (vật + phụ kiện);
    x, y: tâm khối trong khung robot (m); cao: đỉnh vật trên mặt khay (m). Bỏ trống kg/cao thì lấy mặc định theo loại."""
    loai: str = "laptop"
    kg: float | None = None
    x: float = 0.19
    y: float = 0.0
    cao: float | None = None

    def __post_init__(self):
        if self.loai not in LOAI:
            raise CauHinhSai(f"khay: loai phải là một trong {', '.join(LOAI)}, nhận {self.loai!r}")
        kg0, cao0 = LOAI[self.loai]
        kg = kg0 if self.kg is None else self.kg
        cao = cao0 if self.cao is None else self.cao
        if kg is None or cao is None:
            raise CauHinhSai(f"khay: loại {self.loai!r} phải ghi kg và cao")
        for ten, v in (("kg", kg), ("x", self.x), ("y", self.y), ("cao", cao)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise CauHinhSai(f"khay: {ten} phải là số hữu hạn, nhận {v!r}")
        object.__setattr__(self, "kg", float(kg))
        object.__setattr__(self, "cao", float(cao))
        kiem(self)

    @property
    def dinh_z(self) -> float:
        """Độ cao đỉnh vật (m, khung robot)."""
        return MAT_KHAY + self.cao

    @property
    def lift_min(self) -> float:
        """Giới hạn mềm dưới của lift khi có vật này (m)."""
        return max(spec.KHAY_DE_LIFT_MIN, self.dinh_z + spec.KHAY_DE_KHE - Z_DAY_XE_NANG_LIFT0)


def kiem(k: KhayTai) -> None:
    """Từ chối khai báo ngoài định mức hoặc ngoài vùng tâm cho phép (CauHinhSai, CFG-02)."""
    if not 0.0 < k.kg <= spec.KHAY_DE_TAI_MAX:
        raise CauHinhSai(f"khay: tổng khối lượng {k.kg:g} kg ngoài (0; {spec.KHAY_DE_TAI_MAX:g}] kg (định mức khay đế)")
    x0, x1 = spec.KHAY_DE_TAM_X
    if not (x0 <= k.x <= x1 and abs(k.y) <= spec.KHAY_DE_TAM_Y):
        raise CauHinhSai(f"khay: tâm vật ({k.x:g}; {k.y:g}) m ngoài vùng cho phép x {x0:g}..{x1:g}, |y| <= {spec.KHAY_DE_TAM_Y:g} "
                         "(đẩy vật về giữa, sát gờ sau; vật lệch trước hoặc lệch bên làm giảm chống lật)")
    if not 0.0 < k.cao <= 0.30:
        raise CauHinhSai(f"khay: cao {k.cao:g} m ngoài (0; 0,30]")
    if k.lift_min > spec.LIFT_RANGE[1]:
        raise CauHinhSai(f"khay: vật cao {k.cao:g} m cần lift >= {k.lift_min:.3f} m, vượt hành trình")
