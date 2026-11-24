"""Giao thức bus nối tiếp Feetech STS/SMS (TTL half-duplex), viết lại từ tài liệu giao thức của hãng.

Gói lệnh:   0xFF 0xFF ID LEN INSTR P1..Pn CHK,  LEN = n + 2
Gói trả về: 0xFF 0xFF ID LEN ERR   P1..Pn CHK
CHK = ~(ID + LEN + INSTR/ERR + P1 + ... + Pn) & 0xFF
Số 16 bit ghi little-endian; vị trí/tốc độ âm dùng bit 15 làm dấu.
"""
from __future__ import annotations

HEADER = b"\xff\xff"
BROADCAST_ID = 0xFE

# Lệnh
PING = 0x01
READ = 0x02
WRITE = 0x03
REG_WRITE = 0x04
ACTION = 0x05
SYNC_READ = 0x82
SYNC_WRITE = 0x83

# Bảng điều khiển STS (địa chỉ, số byte)
MODEL_NUMBER = (3, 2)   # EEPROM, chỉ đọc
ID_ADDR = (5, 1)
BAUD = (6, 1)
MIN_ANGLE = (9, 2)
MAX_ANGLE = (11, 2)
MODE = (33, 1)          # 0 vị trí, 1 quay liên tục (bánh xe), 3 bước
TORQUE_ENABLE = (40, 1)
ACCELERATION = (41, 1)
GOAL_POSITION = (42, 2)
GOAL_TIME = (44, 2)
GOAL_SPEED = (46, 2)
TORQUE_LIMIT = (48, 2)   # RAM, 0-1000 = 0-100% mô-men chặn
LOCK = (55, 1)          # 0 = mở khoá ghi EEPROM
PRESENT_POSITION = (56, 2)
PRESENT_SPEED = (58, 2)
PRESENT_LOAD = (60, 2)
PRESENT_VOLTAGE = (62, 1)
PRESENT_TEMPERATURE = (63, 1)
MOVING = (66, 1)
PRESENT_CURRENT = (69, 2)

MODE_POSITION = 0
MODE_WHEEL = 1

STEPS_PER_REV = 4096
LOAD_SIGN_BIT = 10        # PRESENT_LOAD: bit 10 = chiều, 0-1000 = 0-100%

# Bit lỗi trong gói trả về
ERROR_BITS = {0: "điện áp", 1: "cảm biến góc", 2: "quá nhiệt", 3: "quá dòng", 5: "quá tải"}


class ProtocolError(Exception):
    pass


class ChecksumError(ProtocolError):
    """Gói đủ độ dài nhưng sai checksum; .sid = ID ghi trong gói (có thể đã hỏng), .used = số byte để bỏ qua gói này."""

    def __init__(self, sid: int, got: int, want: int, used: int):
        super().__init__(f"sai checksum ID {sid}: {got:#04x} != {want:#04x}")
        self.sid, self.used = sid, used


class ServoError(ProtocolError):
    """Servo trả gói hợp lệ nhưng bật bit lỗi (quá tải, quá nhiệt, quá dòng...); .sid, .err."""

    def __init__(self, sid: int, err: int):
        super().__init__(f"servo {sid} báo lỗi: {error_text(err)}")
        self.sid, self.err = sid, err


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def make_packet(servo_id: int, instr: int, params: bytes = b"") -> bytes:
    if not 0 <= servo_id <= 0xFE:
        raise ValueError("ID phải trong 0..254")
    body = bytes([servo_id, len(params) + 2, instr]) + bytes(params)
    return HEADER + body + bytes([checksum(body)])


MAX_LEN = 0xFE         # LEN = 0xFF không hợp lệ (header giả do byte rác 0xFF 0xFF 0x00 trước gói thật)


def parse_packet(buf: bytes) -> tuple[int, int, bytes, int]:
    """Tách một gói ở đầu buf. Trả (id, instr_hoặc_err, params, số byte đã dùng).

    Byte rác trước gói được bỏ qua; 0xFF thừa trước ID được bỏ; header giả (LEN < 2 hoặc 0xFF) thì dò header kế tiếp.
    Chỉ ném ProtocolError: "chưa đủ"/"bị cắt" (chờ thêm byte) hoặc ChecksumError (gói đủ nhưng hỏng)."""
    start = 0
    while True:
        i = buf.find(HEADER, start)
        if i < 0:
            raise ProtocolError("gói chưa đủ hoặc thiếu header")
        while i + 2 < len(buf) and buf[i + 2] == 0xFF:     # bỏ qua 0xFF thừa
            i += 1
        if len(buf) < i + 6:
            raise ProtocolError("gói chưa đủ hoặc thiếu header")
        sid, length = buf[i + 2], buf[i + 3]
        if length < 2 or length > MAX_LEN:
            start = i + 1                                    # header giả: dò tiếp
            continue
        end = i + 4 + length
        if len(buf) < end:
            raise ProtocolError("gói bị cắt")
        body = buf[i + 2:end - 1]
        if checksum(body) != buf[end - 1]:
            raise ChecksumError(sid, buf[end - 1], checksum(body), end)
        return sid, buf[i + 4], bytes(buf[i + 5:end - 1]), end


def u16(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def from_u16(b: bytes) -> int:
    return b[0] | (b[1] << 8)


def encode_signed(v: int, sign_bit: int = 15) -> int:
    v = int(v)
    if v < 0:
        return (-v & ((1 << sign_bit) - 1)) | (1 << sign_bit)
    return v & ((1 << sign_bit) - 1)


def decode_signed(raw: int, sign_bit: int = 15) -> int:
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if raw & (1 << sign_bit) else mag


def read_packet(servo_id: int, reg: tuple[int, int]) -> bytes:
    return make_packet(servo_id, READ, bytes([reg[0], reg[1]]))


def write_packet(servo_id: int, addr: int, data: bytes) -> bytes:
    return make_packet(servo_id, WRITE, bytes([addr]) + bytes(data))


def sync_write_packet(addr: int, size: int, items: dict[int, bytes]) -> bytes:
    params = bytes([addr, size])
    for sid, data in items.items():
        if len(data) != size:
            raise ValueError("mỗi servo phải có đúng size byte")
        params += bytes([sid]) + bytes(data)
    return make_packet(BROADCAST_ID, SYNC_WRITE, params)


def sync_read_packet(addr: int, size: int, ids: list[int]) -> bytes:
    return make_packet(BROADCAST_ID, SYNC_READ, bytes([addr, size]) + bytes(ids))


def error_text(err: int) -> str:
    return ", ".join(v for b, v in ERROR_BITS.items() if err & (1 << b)) or "ok"
