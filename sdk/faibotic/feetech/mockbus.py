"""Bus giả: giả lập phía servo Feetech STS để test backend khi chưa có phần cứng.

Hiểu PING, READ, WRITE, SYNC_READ, SYNC_WRITE; kiểm checksum từng gói nhận; lưu nhật ký gói.
Servo ở chế độ vị trí "tới đích ngay"; chế độ bánh xe: tốc độ hiện tại = tốc độ đặt.
"""
from __future__ import annotations

import random
import time
from collections import deque

from . import protocol as P

try:
    from serial import SerialException as UnplugError   # rút USB thật ném lớp này (con của OSError)
except Exception:                                        # noqa: BLE001 - không có pyserial
    class UnplugError(OSError):
        pass

# Ngưỡng bật bit lỗi khi dùng set_voltage/set_temperature(auto_flag=True) [CẦN-XÁC-MINH theo STS3215: đây là giá trị giả định].
VOLT_LOW_DV, VOLT_HIGH_DV, TEMP_HOT_C = 90, 140, 70


class FakeServoPort:
    """Lớp chèn lỗi (Tầng 1b, chỉ thêm, mặc định không đổi hành vi): drop_reply / drop_rate, corrupt_rate / corrupt_ids,
    drop_rx_rate / drop_rx_ids, dup_ids, set_voltage, set_temperature, unplug / replug, latency_s, chunk, garbage_rate,
    on_write + timeline (record_timeline), log_max. Mọi ngẫu nhiên dùng self.rng (random.Random(seed))."""

    def __init__(self, ids, model_number: int = 777, *, seed: int = 0, log_max: int | None = None):
        self.mem = {}
        for sid in ids:
            m = bytearray(256)
            m[P.ID_ADDR[0]] = sid
            m[3], m[4] = model_number & 0xFF, model_number >> 8
            m[P.LOCK[0]] = 1
            m[P.PRESENT_POSITION[0]:P.PRESENT_POSITION[0] + 2] = P.u16(2048)
            m[P.GOAL_POSITION[0]:P.GOAL_POSITION[0] + 2] = P.u16(2048)
            m[P.PRESENT_VOLTAGE[0]] = 120  # 12,0 V
            m[P.PRESENT_TEMPERATURE[0]] = 30
            self.mem[sid] = m
        self.rx = b""           # byte máy chủ gửi, chưa xử lý
        self.tx = bytearray()   # byte servo trả về
        self.log = [] if log_max is None else deque(maxlen=log_max)
        self.bad_checksums = 0
        self.corrupt_next_reply = False
        self.dead: set[int] = set()    # servo không phản hồi (mất cáp, cháy): im lặng, bỏ qua lệnh ghi
        self.stuck: set[int] = set()   # servo kẹt: nhận lệnh nhưng vị trí không đổi
        self.errors: dict[int, int] = {}  # bit lỗi servo trả trong gói (vd 1 << 5 quá tải, 1 << 2 quá nhiệt)
        # ---- lớp chèn lỗi Tầng 1b ----
        self.rng = random.Random(seed)
        self.drop_reply = None            # Callable[[sid, instr], bool]: True = bỏ gói trả
        self.drop_rate = 0.0              # xác suất bỏ mỗi gói trả
        self.corrupt_rate = 0.0           # xác suất hỏng checksum mỗi gói trả
        self.corrupt_ids: set[int] = set()  # servo luôn trả gói hỏng checksum
        self.drop_rx_rate = 0.0           # xác suất servo bỏ một lệnh ghi (không trả lời)
        self.drop_rx_ids: set[int] = set()  # servo luôn bỏ lệnh ghi
        self.dup_ids: dict[int, int] = {}   # sid -> số servo trùng ID: gói trả chồng lên nhau (byte bị trộn)
        self.latency_s = 0.0              # trả lời chậm (giây, đồng hồ thật)
        self.chunk: int | None = None     # mỗi read() trả tối đa chừng này byte
        self.garbage_rate = 0.0           # xác suất chèn 1..3 byte rác 0x00/0xFF trước gói trả
        self.unplugged: str | None = None   # None | "exc" | "silent"
        self.on_write = None              # Callable[[t, sid, addr, data]]: gọi TRƯỚC khi servo áp lệnh ghi
        self.timeline = None              # None (tắt) hoặc list/deque các (t, sid, addr, data) đã áp (record_timeline)
        self.rx_dropped = 0
        self.powered = True               # False: nguồn servo bị cắt (K1), xem power_from()
        self._cur_instr = 0
        self._delayed: list[tuple[float, bytes]] = []

    # giao diện kiểu pyserial
    def write(self, data: bytes) -> int:
        if self.unplugged == "exc":
            raise UnplugError("thiết bị không còn (rút USB giả)")
        self.rx += bytes(data)
        while True:
            try:
                sid, instr, params, used = P.parse_packet(self.rx)
            except P.ProtocolError as e:
                if "checksum" in str(e):
                    self.bad_checksums += 1
                    self.rx = b""
                break
            self.log.append(self.rx[:used])
            self.rx = self.rx[used:]
            self._handle(sid, instr, params)
        if self.unplugged == "silent":     # lệnh tới được servo nhưng không có gói trả nào về máy chủ
            self.tx.clear()
            self._delayed.clear()
        return len(data)

    def read(self, n: int = 1) -> bytes:
        if self.unplugged == "exc":
            raise UnplugError("thiết bị không còn (rút USB giả)")
        if self.unplugged == "silent":
            return b""
        if self._delayed:
            now = time.monotonic()
            ready = [d for d in self._delayed if d[0] <= now]
            self._delayed = [d for d in self._delayed if d[0] > now]
            for _, b in sorted(ready, key=lambda d: d[0]):
                self.tx += b
        if self.chunk:
            n = min(n, self.chunk)
        out = bytes(self.tx[:n])
        del self.tx[:n]
        return out

    def reset_input_buffer(self) -> None:
        if self.unplugged == "exc":
            raise UnplugError("thiết bị không còn (rút USB giả)")
        self.tx.clear()
        self._delayed.clear()

    # ---------- chèn lỗi (API cho test) ----------
    def unplug(self, mode: str = "exc") -> None:
        """exc: write/read ném UnplugError (con của OSError); silent: lệnh ghi vẫn tới servo nhưng gói trả không về, đọc trả rỗng mãi (đường về đứt, USB còn)."""
        if mode not in ("exc", "silent"):
            raise ValueError("mode phải là 'exc' hoặc 'silent'")
        self.unplugged = mode

    def power_from(self, gpio, pin: str = "RELAY_EN") -> None:
        """Mô hình nguồn servo theo chân cho phép của chốt E-stop (K1): chân `pin` của gpio xuống thấp thì cắt nguồn mọi servo
        (mô-men về 0 kể cả servo im lặng/dead, không nhận lệnh, không trả lời); lên cao thì có nguồn lại (mô-men vẫn tắt)."""
        orig = gpio.write

        def write(p, level):
            orig(p, level)
            if p == pin:
                self.powered = bool(level)
                if not level:
                    for m in self.mem.values():
                        m[P.TORQUE_ENABLE[0]] = 0
        gpio.write = write
        self.powered = bool(getattr(gpio, "levels", {}).get(pin, True))

    def replug(self) -> None:
        self.unplugged = None
        self.tx.clear()
        self._delayed.clear()
        self.rx = b""

    def set_voltage(self, sid: int, dv: int, auto_flag: bool = False) -> None:
        """PRESENT_VOLTAGE theo 0,1 V (120 = 12,0 V). auto_flag: ngoài [VOLT_LOW_DV, VOLT_HIGH_DV] thì bật bit lỗi 0 (điện áp)."""
        self.mem[sid][P.PRESENT_VOLTAGE[0]] = max(0, min(255, int(dv)))
        if auto_flag:
            if dv < VOLT_LOW_DV or dv > VOLT_HIGH_DV:
                self.errors[sid] = self.errors.get(sid, 0) | 1
            else:
                self.errors[sid] = self.errors.get(sid, 0) & ~1

    def set_temperature(self, sid: int, c: int, auto_flag: bool = False) -> None:
        """PRESENT_TEMPERATURE (°C). auto_flag: >= TEMP_HOT_C thì bật bit lỗi 2 (quá nhiệt)."""
        self.mem[sid][P.PRESENT_TEMPERATURE[0]] = max(0, min(255, int(c)))
        if auto_flag:
            if c >= TEMP_HOT_C:
                self.errors[sid] = self.errors.get(sid, 0) | (1 << 2)
            else:
                self.errors[sid] = self.errors.get(sid, 0) & ~(1 << 2)

    def record_timeline(self, maxlen: int | None = None) -> None:
        """Bật ghi mốc thời gian (time.monotonic) mọi lệnh ghi servo đã áp: (t, sid, addr, data)."""
        self.timeline = deque(maxlen=maxlen) if maxlen else []

    def writes_to(self, addr: int, sid: int | None = None) -> list:
        """Các mục timeline ghi vào thanh ghi addr (và servo sid nếu cho)."""
        return [e for e in (self.timeline or []) if e[2] == addr and (sid is None or e[1] == sid)]

    def close(self) -> None:
        pass

    # ---------- phía servo ----------
    def _reply(self, sid: int, params: bytes = b"", err: int = 0) -> None:
        err |= self.errors.get(sid, 0)
        if self.drop_reply is not None and self.drop_reply(sid, self._cur_instr):
            return
        if self.drop_rate and self.rng.random() < self.drop_rate:
            return
        pkt = bytearray(P.make_packet(sid, err, params))
        if sid in self.dup_ids:      # hai servo cùng ID trả đồng thời: byte chồng lên nhau (OR hai gói khác dữ liệu)
            other = bytearray(P.make_packet(sid, (err ^ 0x40) & 0xFF, bytes((b ^ 0x5A) & 0xFF for b in params)))
            pkt = bytearray(a | b for a, b in zip(pkt, other))
        if self.corrupt_next_reply:
            pkt[-1] ^= 0xFF
            self.corrupt_next_reply = False
        elif sid in self.corrupt_ids or (self.corrupt_rate and self.rng.random() < self.corrupt_rate):
            pkt[-1] ^= 0xFF
        if self.garbage_rate and self.rng.random() < self.garbage_rate:
            pkt = bytearray(self.rng.choice((0x00, 0xFF)) for _ in range(self.rng.randint(1, 3))) + pkt
        if self.latency_s > 0:
            self._delayed.append((time.monotonic() + self.latency_s, bytes(pkt)))
        else:
            self.tx += pkt

    def _get(self, sid, addr, size) -> bytes:
        self._tick(sid)
        return bytes(self.mem[sid][addr:addr + size])

    def set_load(self, sid: int, permille: int) -> None:
        """Đặt tải hiện tại (‰ mô-men chặn, có dấu) để test giám sát cặp và lò xo."""
        raw = P.encode_signed(int(permille), P.LOAD_SIGN_BIT)
        self.mem[sid][P.PRESENT_LOAD[0]:P.PRESENT_LOAD[0] + 2] = P.u16(raw)

    def set_position(self, sid: int, raw: int) -> None:
        """Đặt vị trí hiện tại (vd tay nằm ở chặn cơ khí sau mất điện)."""
        self.mem[sid][P.PRESENT_POSITION[0]:P.PRESENT_POSITION[0] + 2] = P.u16(raw)

    def _set(self, sid, addr, data: bytes):
        if sid in self.dead:
            return
        if sid in self.drop_rx_ids or (self.drop_rx_rate and self.rng.random() < self.drop_rx_rate):
            self.rx_dropped += 1
            return False
        m = self.mem[sid]
        eeprom = addr < 40 and addr != P.LOCK[0]
        if eeprom and m[P.LOCK[0]] == 1:
            return  # EEPROM đang khoá: servo thật bỏ qua lệnh ghi
        if self.on_write is not None or self.timeline is not None:
            t = time.monotonic()
            if self.on_write is not None:
                self.on_write(t, sid, addr, bytes(data))
            if self.timeline is not None:
                self.timeline.append((t, sid, addr, bytes(data)))
        m[addr:addr + len(data)] = data
        self._tick(sid)

    def _tick(self, sid) -> None:
        m = self.mem[sid]
        if m[P.MODE[0]] == P.MODE_WHEEL:
            m[P.PRESENT_SPEED[0]:P.PRESENT_SPEED[0] + 2] = m[P.GOAL_SPEED[0]:P.GOAL_SPEED[0] + 2]
        elif m[P.TORQUE_ENABLE[0]] and sid not in self.stuck:
            m[P.PRESENT_POSITION[0]:P.PRESENT_POSITION[0] + 2] = m[P.GOAL_POSITION[0]:P.GOAL_POSITION[0] + 2]

    def _handle(self, sid, instr, params) -> None:
        """Gói hỏng cấu trúc (thiếu params, địa chỉ vượt bảng nhớ, cỡ đọc quá gói) bị bỏ qua như servo thật, không ném lỗi."""
        self._cur_instr = instr
        if not self.powered:
            return                       # mất nguồn servo: không servo nào nhận lệnh hay trả lời
        if instr == P.SYNC_WRITE:
            if len(params) < 3 or params[1] == 0:
                return
            addr, size = params[0], params[1]
            body = params[2:]
            for k in range(0, len(body) - size, size + 1):
                tid = body[k]
                if tid in self.mem and addr + size <= len(self.mem[tid]):
                    self._set(tid, addr, body[k + 1:k + 1 + size])
            return
        if instr == P.SYNC_READ:
            if len(params) < 2 or params[1] == 0 or params[1] > P.MAX_LEN - 2:
                return
            addr, size = params[0], params[1]
            for tid in params[2:]:
                if tid in self.mem and tid not in self.dead:
                    self._reply(tid, self._get(tid, addr, size))
            return
        if sid not in self.mem or sid in self.dead:
            return  # không có servo: im lặng
        if instr == P.PING:
            self._reply(sid)
        elif instr == P.READ:
            if len(params) < 2 or params[1] > P.MAX_LEN - 2:
                return
            self._reply(sid, self._get(sid, params[0], params[1]))
        elif instr == P.WRITE:
            if len(params) < 2 or params[0] + len(params) - 1 > len(self.mem[sid]):
                return
            if self._set(sid, params[0], params[1:]) is not False:
                self._reply(sid)
