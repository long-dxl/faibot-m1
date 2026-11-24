"""Bus servo Feetech trên cổng nối tiếp (pyserial) hoặc bất kỳ đối tượng có read/write/reset_input_buffer.

Tầng 1b: lỗi I/O của cổng (rút USB: OSError/SerialException) -> BusMat (BUS-01); mở cổng lỗi/bận -> BusMat (BUS-05);
mỗi lần đọc có trần thời gian cố định max_wait (spec.BUS_READ_DEADLINE_S) thay cho timeout x (2 + n); gói trả thiếu dữ liệu -> ProtocolError.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

from .. import spec
from ..loi import BusMat
from . import protocol as P


class SyncReadTimeout(TimeoutError):
    """sync_read thiếu trả lời; .missing = danh sách ID im lặng."""

    def __init__(self, missing: list[int]):
        super().__init__(f"sync read thiếu ID {missing}")
        self.missing = missing


@dataclass
class SyncResult:
    """Kết quả sync_read_all: data {sid: bytes}, err {sid: bit lỗi != 0}, missing (im lặng), corrupt (gói hỏng checksum)."""
    data: dict = field(default_factory=dict)
    err: dict = field(default_factory=dict)
    missing: list = field(default_factory=list)
    corrupt: list = field(default_factory=list)


def _open_serial(port: str, baudrate: int, timeout: float):
    import serial  # pyserial, chỉ cần khi chạy phần cứng thật
    kw = dict(baudrate=baudrate, timeout=timeout)
    if os.name == "posix":
        kw["exclusive"] = True          # R-38: hai tiến trình không cùng mở một cổng (khoá TIOCEXCL/flock)
    try:
        return serial.Serial(port, **kw)
    except (serial.SerialException, OSError, ValueError) as e:
        busy = "busy" in str(e).lower() or getattr(e, "errno", None) == 16 or "exclusive" in str(e).lower()
        why = ("cổng đang được chương trình khác dùng" if busy else
               "không có thiết bị, không đủ quyền, hoặc cổng đang được chương trình khác dùng")
        raise BusMat(f"không mở được cổng {port}: {why} ({e})", ma="BUS-05") from None


class FeetechBus:
    def __init__(self, port, baudrate: int = 1_000_000, timeout: float = 0.02, retries: int = 2,
                 max_wait: float | None = None):
        if isinstance(port, str):
            self.port = _open_serial(port, baudrate, timeout)
            self.port_name = port
        else:
            self.port = port
            self.port_name = getattr(port, "name", None) or type(port).__name__
        self.timeout = timeout
        self.retries = retries
        self.max_wait = spec.BUS_READ_DEADLINE_S if max_wait is None else max_wait

    # ---------- I/O cổng: OSError -> BusMat ----------
    def _io(self, fn, *a):
        try:
            return fn(*a)
        except OSError as e:
            if isinstance(e, BusMat):
                raise
            raise BusMat(f"mất cổng {self.port_name}: {e} (rút USB hoặc mất nguồn bộ chuyển)", ma="BUS-01") from e

    def _wait(self, n: int) -> float:
        return min(self.timeout * n, self.max_wait)

    # ---------- truyền nhận ----------
    def _send(self, pkt: bytes) -> None:
        if hasattr(self.port, "reset_input_buffer"):
            self._io(self.port.reset_input_buffer)
        self._io(self.port.write, pkt)

    def _recv(self) -> tuple[int, int, bytes]:
        buf = b""
        deadline = time.monotonic() + self._wait(5)
        while True:
            chunk = self._io(self.port.read, 64)
            if chunk:
                buf += chunk
            try:
                sid, err, params, used = P.parse_packet(buf)
                self._rest = buf[used:]
                return sid, err, params
            except P.ChecksumError:
                raise
            except P.ProtocolError:
                pass
            if time.monotonic() > deadline:
                raise TimeoutError("servo không trả lời")

    def _transact(self, pkt: bytes, expect_id: int) -> bytes:
        last = None
        for _ in range(self.retries + 1):
            try:
                self._send(pkt)
                sid, err, params = self._recv()
                if sid != expect_id:
                    raise P.ProtocolError(f"trả lời từ ID {sid}, chờ ID {expect_id}")
                if err:
                    raise P.ServoError(sid, err)
                return params
            except (TimeoutError, P.ProtocolError) as e:
                last = e
        raise last

    # ---------- lệnh ----------
    def ping(self, sid: int) -> bool:
        return self.ping_detail(sid) == "ok"

    def ping_detail(self, sid: int) -> str:
        """'ok' | 'silent' (không trả lời) | 'garbled' (mọi lần thử đều trả gói hỏng checksum: nghi trùng ID) | 'error' (khác)."""
        try:
            self._transact(P.make_packet(sid, P.PING), sid)
            return "ok"
        except P.ServoError:
            return "ok"                     # servo trả lời (bit lỗi xử lý ở vòng chạy)
        except P.ChecksumError:
            return "garbled"
        except TimeoutError:
            return "silent"
        except P.ProtocolError:
            return "error"

    def read(self, sid: int, reg: tuple[int, int]) -> int:
        data = self._transact(P.read_packet(sid, reg), sid)
        if len(data) < reg[1]:
            raise P.ProtocolError(f"gói trả ID {sid} thiếu dữ liệu: {len(data)} byte, cần {reg[1]}")
        return data[0] if reg[1] == 1 else P.from_u16(data)

    def write(self, sid: int, reg: tuple[int, int], value: int) -> None:
        data = bytes([value & 0xFF]) if reg[1] == 1 else P.u16(value)
        self._transact(P.write_packet(sid, reg[0], data), sid)

    def sync_write(self, reg: tuple[int, int], values: dict[int, int]) -> None:
        items = {sid: (bytes([v & 0xFF]) if reg[1] == 1 else P.u16(v)) for sid, v in values.items()}
        self._send(P.sync_write_packet(reg[0], reg[1], items))  # broadcast: không có gói trả

    def sync_read(self, reg: tuple[int, int], ids: list[int]) -> dict[int, int]:
        self._send(P.sync_read_packet(reg[0], reg[1], ids))
        out = {}
        self._rest = b""
        buf = b""
        want = set(ids)
        deadline = time.monotonic() + self._wait(2 + len(ids))
        while len(out) < len(want):
            buf += self._io(self.port.read, 64)
            while True:
                try:
                    sid, err, params, used = P.parse_packet(buf)
                except P.ChecksumError:
                    raise
                except P.ProtocolError:
                    break
                buf = buf[used:]
                if sid not in want:
                    continue                     # gói lạ (ID không hỏi): bỏ
                if err:
                    raise P.ServoError(sid, err)
                if len(params) < reg[1]:
                    raise P.ProtocolError(f"gói trả ID {sid} thiếu dữ liệu: {len(params)} byte, cần {reg[1]}")
                out[sid] = params[0] if reg[1] == 1 else P.from_u16(params)
            if time.monotonic() > deadline:
                missing = [i for i in ids if i not in out]
                raise SyncReadTimeout(missing)
        return out

    def sync_read_all(self, addr: int, size: int, ids: list[int]) -> SyncResult:
        """Một lần SYNC_READ, không ném lỗi giao thức: gói hỏng checksum được bỏ qua (ghi vào corrupt) rồi dò tiếp gói sau,
        bit lỗi ghi vào err, servo không trả lời trong trần max_wait vào missing. Lỗi cổng vẫn ném BusMat."""
        self._send(P.sync_read_packet(addr, size, ids))
        res = SyncResult()
        want = set(ids)
        seen: set[int] = set()
        buf = b""
        deadline = time.monotonic() + self._wait(2 + len(ids))
        while len(seen) < len(want):
            buf += self._io(self.port.read, 64)
            while True:
                try:
                    sid, err, params, used = P.parse_packet(buf)
                except P.ChecksumError as e:
                    buf = buf[e.used:]
                    if e.sid in want and e.sid not in seen:
                        res.corrupt.append(e.sid)
                        seen.add(e.sid)
                    continue
                except P.ProtocolError:
                    break
                buf = buf[used:]
                if sid not in want or sid in seen:
                    continue
                seen.add(sid)
                if len(params) < size:
                    res.corrupt.append(sid)
                    continue
                res.data[sid] = bytes(params[:size])
                if err:
                    res.err[sid] = err
            if time.monotonic() > deadline:
                break
        res.missing = [i for i in ids if i not in seen]
        return res

    def unlock_eeprom(self, sid: int) -> None:
        self.write(sid, P.LOCK, 0)

    def lock_eeprom(self, sid: int) -> None:
        self.write(sid, P.LOCK, 1)

    def close(self) -> None:
        if hasattr(self.port, "close"):
            self.port.close()
