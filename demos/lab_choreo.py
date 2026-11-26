"""Biên đạo mới (docs/CHOREO-M1.md) cho các demo lab trên sim SDK: đế đứng yên ở chỗ đứng cố định, hai tay chạy gối nhau,
đoạn min-jerk, tiếp cận nhanh rồi chậm ở 2 cm cuối, kẹp đóng/mở trong lúc tay đang đi, không có chờ cố định giữa các pha.

Cách chạy song song: mỗi tay (và đế, trục nâng) là một generator; mỗi chu kỳ điều khiển Engine lấy đích của mọi generator, gộp lại,
gọi Robot.set_joint_positions (giới hạn khớp, trần gập 70 độ, ghép yaw theo gập, vùng cấm trạm vẫn do SDK kiểm ở mọi lệnh) và Robot.step().
Đế chỉ nhận vận tốc qua Robot.set_base_velocity: hồ sơ min-jerk với gia tốc đỉnh dưới dốc gia tốc đế 0,05 m/s² (BaseRamp vẫn chặn ở backend).
Hai tay đồng bộ qua cờ chung (dict), không dùng wait cố định.

Đọc vị trí vật từ ground truth của sim như demo cũ (thực tế lấy từ thị giác).
"""
from __future__ import annotations

import math
import os as _os

import numpy as np

import lab_common as LC
from lab_common import L
from faibotic import kinematics as K, spec

DEG = math.pi / 180

# ---------------- thông số (CHOREO-M1 mục 3.2, 4) ----------------
V_ARM, A_ARM, J_ARM, T_MIN = 0.40, 1.2, 6.0, 0.5     # đoạn tay tự do: trần v, a, jerk kế hoạch, thời lượng tối thiểu
BASE_ACC = 0.045                                     # gia tốc đỉnh kế hoạch của đế (< dốc 0,05 m/s²: chừa 10% cho dốc BaseRamp)
SLOW = 0.02                                          # chậm ở 2 cm cuối khi tiếp cận chạm
V_END = 0.06                                         # vận tốc nối giữa đoạn nhanh và đoạn chậm (m/s)
T_FAST_MIN, T_SLOW = 0.45, 0.45                      # thời lượng đoạn nhanh (tối thiểu) và đoạn chậm
CLOSE_AHEAD = 0.008     # bắt đầu khép khi TCP còn cách đích 8 mm (đoạn cuối chậm; khép 0,6 s, hàm chạm vật sau khi TCP tới đích)
T_CLOSE = 0.6           # khép (0,35 s: hàm đập vào tay nắm/lon, rung vào jerk)
J_APP = 4.0             # trần jerk kế hoạch của đoạn nhanh khi tiếp cận
T_OPEN = float(_os.environ.get("M1_CHOREO_T_OPEN", "0.5"))    # nhả: 0,5 s (thiết kế 0,25 s: mở nhanh hất đổ vật trụ trên sim)
PAD_OPEN = float(_os.environ.get("M1_CHOREO_PAD_OPEN", "70"))     # nhả trên đệm: mở 70% trong 0,5 s (mở nhanh 0,25 s hất đổ vật trụ)
TRAY_T_OPEN = 0.8      # nhả trong ngăn khay hẹp (vật cạnh vật): mở chậm như demo cũ (0,9 s) để vật không xô sang chỗ bên
PAD_T_OPEN = float(_os.environ.get("M1_CHOREO_PAD_T_OPEN", "0.5"))
T_PUSH = 3.0            # đẩy cửa từ 0,30 tới 1,30 rad
T_HOME = 3.0            # tay về HOME (khớp, min-jerk)
GRIP_AIR = 10.0         # khép trên không: dừng ở 10% (khép về 0 thì hai ngón chạm nhau, va đập vào số đo)
LEG_OVERLAP = 0.85      # chặng đế sau bắt đầu khi chặng trước đi được 85% thời lượng (còn ~3% quãng)
BASE_LEAD = 0.25        # tay bắt đầu bước sau khi đế còn 0,25 s (vận tốc < 5 mm/s, quãng còn < 0,1 mm)
T_PREP, T_PREP_L = 3.5, 4.0   # tay vào tư thế chuẩn bị trong lúc đế đi (min-jerk khớp)
GC0 = 0.3               # khép kẹp từ 30% đoạn lùi khỏi tay nắm
LEAD_DOOR = 0.25        # cửa: rút bắt đầu khi đẩy còn 0,25 s
LEAD_PUSH = 0.25        # đẩy bắt đầu khi đường vòng còn LEAD_PUSH s
AROUND_BLEND = 0.2      # bo góc đường vòng qua mép cửa
RELEASE_LEAD = 0.12     # đặt lon: bắt đầu mở kẹp 0,12 s trước cuối đoạn hạ
CLOSE_TAIL = 0.12       # khép: bước sau bắt đầu khi kẹp còn 0,12 s (đã khép > 90%)
CAN_LIFT = 0.04         # nhấc lon trong tủ 4 cm (2,5 cm: lon rung khi đế lùi)
RETRACT = (0.10, 0.06)  # rút sau khi đặt lon: lùi theo trục dụng cụ, nâng
F2B_BACK_FIRST = True   # F2 -> F2b: lùi thẳng trước rồi mới sang +y (đi chéo: kẹp cọ vách trái tủ khi lon lệch +y)
PP_DROP = 0.004         # P&P: đặt vật cao hơn mặt thảm (m)
PP_T_OPEN = 0.5         # P&P: thời gian mở kẹp khi nhả
PP_LIFT_DELAY = 4.0     # P&P: đế đi từ S tới F1, hạ lift sau 4 s (đã xa mép bàn)
PP_CAN_DX = float(_os.environ.get("M1_PP_CAN_DX", "0.0"))   # cổng 4: lon đặt xa vách trước khay hơn (lòng kẹp trái chạm vách khi nhả)
PP_CAN_DY = -0.03       # P&P: lon đặt lệch -3 cm theo y so với ô 3 (xem pick_place)
PLACE_BLEND = 0.2       # bo góc đường nhấc -> trên chỗ đặt -> hạ đặt
PICK_MARGIN = 0.008     # khe 8 mm giữa ngón cố định và vật khi hạ kẹp (xem pick_down)
APP_LEAD = 0.25        # gắp: tiếp cận bắt đầu khi đường tới trên vật còn 0,25 s
T_LIFT_OBJ = 0.8        # nhấc vật 2,5-4 cm sau khi khép
ABOVE = 0.06                                         # điểm trên vật: 6 cm trên đỉnh vật

S_SORT = (0.30, 0.02)                                # chỗ đứng Sorting (thế giới), lift 0,45 (choreo_check S)
PAD = (0.28, 0.0)                                    # đệm đổi tay M (khung robot), đúng giữa thân. Thiết kế (0,33; -0,05): tay trái chỉ
                                                     # đặt được ở -70 độ, vật trụ đổ khi nhả nghiêng; ở (0,28; 0) cả hai tay cùng -85 độ (IK SDK)
PAD_PITCH = (-85.0, -80.0)
REACH_Y = 0.42          # |y| vật (khung robot) lớn nhất tay trái hạ tới mặt bàn được ở S (vật xa hơn: đế dời sang, xem sorting)
F1, F2 = (-0.20, -0.85), (0.05, -1.35)
F2B = (-0.04, -1.31)    # thiết kế (-0,05; -1,35): lùi 10 cm. Ở đây lùi 9 cm và thêm 4 cm sang +y cùng lúc vì chỗ đặt (0,56; -0,95) cộng lệch ngón cố định 2 cm
                        # ra ngoài tầm tay trái ngang từ (-0,05; -1,35) (IK SDK); lon trong tủ còn cách vách 6 cm
PLACE_REACH = (0.60, 0.36)   # tâm lon so với đế khi đặt bằng tay trái ngang (P&P: chỗ đứng = đích - PLACE_REACH)
LIFT_FRIDGE = 0.27
CAN_OPEN = 60           # mở vừa lon (khe ~58 mm): mở hết 97 mm thì hàm động chạm vách trái miệng tủ khi tiếp cận chéo
# Cổng 4 (docs/CHOREO-M1-ketqua.md mục 9): bản mục 5 đẩy mặt trong cửa bằng lòng kẹp phải (vỏ kẹp chạm cửa, quy tắc B) và cửa trôi tới
# 1,65-1,75 chạm thân trạm. FR5_PULL = 1: tay phải giữ thanh tay nắm (kẹp ngang như cũ, khớp đứng yên), đế tịnh tiến theo cung tay nắm
# kéo cửa thẳng tới FR5_TH (không vòng qua mép, không đẩy), nhả, rút. Tay nắm ở FR5_REL (khung robot) lúc kẹp: thẳng trước vai phải;
# dò tĩnh: khe khâu tay - cửa >= 1,5 cm suốt cung 0 -> 1,75.
FR5_PULL = _os.environ.get("M1_FR5_PULL", "1") == "1"
FR5_REL = (float(_os.environ.get("M1_FR5_BX", "0.55")), float(_os.environ.get("M1_FR5_BY", "-0.17")))
FR5_TH = float(_os.environ.get("M1_FR5_TH", "1.75"))   # kéo tới chặn cửa (range 1,75): cửa xa thân trạm ở F2 nhất
FR5_K = 1.0
# lấy lon (cổng 4): giá camera tay trái nhô ~10 cm trên TCP, lòng kẹp 2,8 cm dưới TCP. Kẹp lon ở tâm + 1,2 cm và nhấc 4 cm (bản cũ) thì
# camera quẹt nóc trong tủ (0,5 mm); hạ điểm kẹp thì lòng kẹp chạm kệ. Kẹp ở tâm + 1,8 cm, nhấc 2,5 cm: hai phía cùng hở ~0,8 cm.
FR5_ZC = float(_os.environ.get("M1_FR5_ZC", "0.018"))        # TCP kẹp lon so với tâm lon (z)
FR5_CAN_LIFT = float(_os.environ.get("M1_FR5_CAN_LIFT", "0.025"))
PP_PLACE_DZ = float(_os.environ.get("M1_PP_PLACE_DZ", "0.0"))   # P&P: đặt lon cao thêm (m)
FR5_F2DY = float(_os.environ.get("M1_FR5_F2DY", "-0.01"))   # F2 lệch -y 1 cm: servo cổ tay trái không chạm vách trái tủ khi lon lệch +y (cần cửa mở 1,75)
FR5_OUT_X = float(_os.environ.get("M1_FR5_OUT_X", "0.45"))   # 0,45: lòng kẹp trái quẹt mép vách trái tủ (0,1 mm) khi mang lon ra
FR5_CORNER_Y = float(_os.environ.get("M1_FR5_CORNER_Y", "-1.08"))   # y (thế giới) góc tủ khi mang lon ra
FR5_F2K = float(_os.environ.get("M1_FR5_F2K", "0.0"))
FR5_F2B_DY = float(_os.environ.get("M1_FR5_F2B_DY", "-0.01"))   # F2b: sang +y 3 cm (thay 4 cm) sau khi lùi
FR5_BACK = float(_os.environ.get("M1_FR5_BACK", "0.10"))     # rút tay dọc mặt ngoài cửa sau khi nhả (m)
FR5_PRE = np.array([float(x) for x in _os.environ.get("M1_FR5_PRE", "-0.08,0,0").split(",")])
FR5_REL_LEAD = float(_os.environ.get("M1_FR5_REL_LEAD", "0.7"))
FR5_RET_LEAD = float(_os.environ.get("M1_FR5_RET_LEAD", "0.2"))   # rút tay bắt đầu khi cung kéo còn 0,2 s (kẹp đã mở xong)
PULL_TH = 0.30          # kéo tay nắm tới 0,30 rad (thiết kế 0,50: từ F1 tay không vòng qua mép cửa được ở 0,50, IK SDK)
PUSH_TH = 1.30          # đẩy mặt trong tới 1,30 rad
Z_PUSH = 0.72           # vòng qua mép cửa và đẩy ở z 0,72 (thanh tay nắm dài 0,62-0,80; ở z 0,775 vòng mép không có nghiệm)
U_PUSH2, V_PUSH2 = 0.03, 0.42   # điểm đẩy khi cửa đã mở rộng
TH_SLIDE = (0.35, 0.60)          # điểm đẩy trượt từ (U_PUSH, V_PUSH) sang (U_PUSH2, V_PUSH2) khi cửa đi từ 0,35 tới 0,60 rad
U_PUSH, V_PUSH = 0.03, 0.44   # điểm đẩy đầu (khung cửa): 3 cm phía trong mặt phẳng giữa cửa, cách bản lề 0,44 m, sát mép cửa (0,50):
                              # ở góc nhỏ thân kẹp không lọt ra sau cánh, nên tì vào góc mép trong
PITCH_DOWN = (-90.0, -85.0, -80.0, -75.0, -70.0)                  # ưu tiên chĩa thẳng xuống; trần gập 70 thì dùng -80/-70 (choreo_check.json)


def mj(s: float) -> float:
    """min-jerk 0..1."""
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10 - 15 * s + 6 * s * s)


def mj_d(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return 30 * s * s * (1 - s) ** 2


def dur_for(d: float, v=V_ARM, a=A_ARM, j=J_ARM, tmin=T_MIN) -> float:
    return max(tmin, 1.875 * d / v, math.sqrt(5.774 * d / a), (60 * d / j) ** (1 / 3))


def hermite5(p0, v0, p1, v1, T, t):
    """Quintic Hermite (gia tốc 0 ở hai đầu) cho đại lượng vô hướng."""
    s = min(max(t / T, 0.0), 1.0)
    h00 = 1 - 10 * s ** 3 + 15 * s ** 4 - 6 * s ** 5
    h10 = s - 6 * s ** 3 + 8 * s ** 4 - 3 * s ** 5
    h01 = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    h11 = -4 * s ** 3 + 7 * s ** 4 - 3 * s ** 5
    return h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1


class Engine:
    """Chạy song song nhiều generator; mỗi generator yield dict đích khớp (có thể rỗng), khoá "_base" = (vx, vy) đế."""

    def __init__(self, robot):
        self.r = robot
        self.dt = robot.dt
        self.st: dict = {}
        self.ik_miss = 0
        self.retries = 0
        self.log = lambda *a: None

    def run(self, *gens):
        gens = [g for g in gens if g is not None]
        while gens:
            tg, base, alive = {}, None, []
            for g in gens:
                try:
                    y = next(g)
                except StopIteration:
                    continue
                alive.append(g)
                if y:
                    y = dict(y)
                    b = y.pop("_base", None)
                    if b is not None:
                        base = b
                    tg.update(y)
            gens = alive
            if not gens:
                break
            if base is not None:
                self.r.set_base_velocity(base[0], base[1], 0.0)
            if tg:
                self.r.set_joint_positions(tg)
            self.r.step()

    # ---------- IK ----------
    def ik(self, side, p, pitch_deg, gy=0.0, roll=None, lift=None):
        return self.r.solve_ik(side, np.asarray(p, float), pitch=pitch_deg * DEG, grip_yaw=gy, roll=roll, lift=lift)

    def ik_down(self, side, p, gy=0.0, lift=None):
        """IK chĩa xuống, thử -90/-80/-70 độ (trần gập 70)."""
        for pd in PITCH_DOWN:
            q = self.ik(side, p, pd, gy, lift=lift)
            if q is not None:
                return q
        return None

    def pick_pitch(self, side, pts, gy=0.0, cands=PITCH_DOWN, lift=None):
        """Góc ngẩng đầu tiên trong cands có nghiệm IK ở mọi điểm pts (để cả đoạn tiếp cận cùng một góc)."""
        for pd in cands:
            if all(self.ik(side, p, pd, gy, lift=lift) is not None for p in pts):
                return pd
        raise ValueError(f"tay {side}: không có góc ngẩng chung cho {[list(np.round(p, 3)) for p in pts]}")

    def level_pitch(self, side, pts, cands=(0.0, 10.0, 15.0, -10.0)):
        """Góc ngẩng (dụng cụ gần ngang, cuộn 0) có nghiệm ở mọi điểm pts."""
        for pd in cands:
            if all(self.ik(side, p, pd, roll=0.0) is not None for p in pts):
                return pd
        raise ValueError(f"tay {side}: không có góc ngang chung cho {[list(np.round(p, 3)) for p in pts]}")

    def q_now(self, side):
        return self.r.arm_q(side, "target")

    def tcp_now(self, side):
        return self.r.tcp_position(side, "target")

    def w2r(self, p, base=None):
        """Thế giới -> khung robot. base: (x, y) đế dùng thay tư thế hiện tại (tư thế cuối dự đoán của chặng đế đang chạy,
        để tay bắt đầu bước sau trước khi đế dừng hẳn); yaw lấy hiện tại."""
        if base is None:
            return self.r.world_to_robot(p)
        yaw = self.r.backend.base_pose()[2]
        d = np.asarray(p, float)[:2] - np.asarray(base, float)[:2]
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], float(p[2])])

    def jd(self, side, q):
        return {spec.joint_name(side, j): float(v) for j, v in zip(spec.ARM_JOINTS, q)}


# ---------------- primitive (generator) ----------------

def grip_sched(side, g0, g1, t0, T):
    """Hàm thời gian -> đích kẹp (min-jerk từ g0 tới g1 bắt đầu ở t0, dài T)."""
    gk = spec.joint_name(side, "gripper")
    return lambda t: {gk: g0 + (g1 - g0) * mj((t - t0) / T)} if t >= t0 else {}


def jmove(E, side, q1, T, grips=(), t_start=0.0):
    """Nội suy khớp min-jerk từ đích hiện tại tới q1 (dùng cho đoạn vào tư thế đầu, về HOME)."""
    for _ in range(int(round(t_start / E.dt))):
        yield {}
    q0 = E.q_now(side)
    n = max(1, int(round(T / E.dt)))
    for i in range(1, n + 1):
        t = i * E.dt
        d = E.jd(side, q0 + (np.asarray(q1) - q0) * mj(i / n))
        for g in grips:
            d.update(g(t))
        yield d


def gripper_to(E, side, g1, T):
    gk = spec.joint_name(side, "gripper")
    g0 = E.r._targets[gk]
    n = max(1, int(round(T / E.dt)))
    for i in range(1, n + 1):
        yield {gk: g0 + (g1 - g0) * mj(i / n)}


BLEND = 0.2      # nối đoạn: đoạn sau bắt đầu khi đoạn trước còn 20% thời lượng (cộng chồng min-jerk, bo góc ~3% chiều dài đoạn)
ROT_M = 0.15     # quy đổi xoay dụng cụ ra quãng (m/rad) khi tính thời lượng đoạn


def path_sched(pts, pitches=None, yaws=None, T=None, blend=BLEND):
    """Lịch các đoạn con của cpath: (giờ bắt đầu, thời lượng) mỗi đoạn và tổng thời lượng.
    Mỗi đoạn min-jerk riêng (thời lượng theo dur_for, tính cả góc xoay), đoạn sau chồng lên đuôi đoạn trước `blend`.
    T: ép tổng thời lượng (co giãn đều)."""
    pts = [np.asarray(p, float) for p in pts]
    durs = []
    for k in range(len(pts) - 1):
        d = float(np.linalg.norm(pts[k + 1] - pts[k]))
        if pitches is not None:
            d += ROT_M * abs(pitches[k + 1] - pitches[k]) * DEG
        if yaws is not None:
            d += ROT_M * abs(yaws[k + 1] - yaws[k])
        durs.append(dur_for(d) if d > 1e-6 else 0.0)
    starts, t = [], 0.0
    for k, Tk in enumerate(durs):
        starts.append(t)
        t += Tk * (1.0 - blend) if k + 1 < len(durs) else Tk
    total = max(t, E_MIN_T)
    if T is not None and total > 1e-9:
        f = T / total
        starts = [s * f for s in starts]
        durs = [d * f for d in durs]
        total = T
    return starts, durs, total


E_MIN_T = 0.02


def _lin(delta, T):
    """Hàm dời min-jerk tuyến tính (cho superpose): tau -> delta * mj(tau / T)."""
    delta = np.asarray(delta, float)
    return lambda tau: delta * mj(tau / T)


def superpose(E, side, p0, moves, pitch=0.0, roll=0.0, grips=()):
    """Vị trí TCP = p0 + tổng các chuyển động con (t0, T, f): f(tau) là độ dời sau tau giây (f(0) = 0), giữ f(T) sau khi xong.
    Dụng cụ ngang (pitch, roll cố định). Chuyển động con có thể chồng nhau (cộng chồng kiểu Flash-Hogan)."""
    p0 = np.asarray(p0, float)
    Tt = max(t0 + T for t0, T, _ in moves)
    n = max(1, int(round(Tt / E.dt)))
    last = None
    for i in range(1, n + 1):
        t = i * E.dt
        p = p0.copy()
        for t0, T, f in moves:
            if t <= t0:
                continue
            p = p + f(min(t - t0, T))
        q = E.ik(side, p, pitch, roll=roll)
        if q is None:
            E.ik_miss += 1
            d = {}
        else:
            last = q
            d = E.jd(side, q)
        for g in grips:
            d.update(g(t))
        yield d
    if last is None:
        raise ValueError(f"tay {side}: chuyển động cộng chồng không có nghiệm IK")


def cpath(E, side, pts, pitches, yaws, T=None, roll_mode=False, grips=(), slow_tail=False, blend=BLEND, then=None):
    """Đường Descartes qua các điểm (khung robot), không dừng ở điểm giữa: mỗi đoạn là một chuyển động min-jerk, đoạn sau bắt đầu
    khi đoạn trước còn `blend` thời lượng; vị trí = tổng các đoạn (cộng chồng kiểu Flash-Hogan), nên vận tốc, gia tốc liên tục và
    góc được bo tròn (không đổi hướng tức thời ở điểm giữa). pitches, yaws: theo từng điểm (độ / rad), cộng chồng như vị trí.
    roll_mode: yaws là góc cuộn (dụng cụ ngang), không phải grip_yaw. Điểm không có nghiệm IK thì giữ đích trước (đếm vào E.ik_miss)."""
    pts = [np.asarray(p, float) for p in pts]
    starts, durs, T = path_sched(pts, pitches, yaws, T, blend)
    T_end = T
    if then is not None:        # then = (f, T_f, lead): dời thêm f(tau) (f(0) = 0) bắt đầu khi đường còn `lead` giây, giữ góc cuối
        f_then, T_then, lead = then
        t_then = max(0.0, T - lead)
        T_end = max(T, t_then + T_then)
    n = max(1, int(round(T_end / E.dt)))

    def at(t):
        p, pd, yw = pts[0].copy(), float(pitches[0]), float(yaws[0])
        for k, (t0, Tk) in enumerate(zip(starts, durs)):
            s = 1.0 if Tk <= 1e-9 else mj((t - t0) / Tk)
            p = p + (pts[k + 1] - pts[k]) * s
            pd += (pitches[k + 1] - pitches[k]) * s
            yw += (yaws[k + 1] - yaws[k]) * s
        return p, pd, yw
    last = None
    for i in range(1, n + 1):
        t = i * E.dt
        p, pd, yw = at(t if (i < n and t < T) else T + 1.0)
        if then is not None and t > t_then:
            p = p + f_then(min(t - t_then, T_then) if i < n else T_then)
        q = E.ik(side, p, pd, roll=yw) if roll_mode else E.ik(side, p, pd, yw)
        if q is None:
            E.ik_miss += 1
            d = {}
        else:
            last = q
            d = E.jd(side, q)
        for g in grips:
            d.update(g(t))
        yield d
    if last is None:
        raise ValueError(f"tay {side}: đường {[list(np.round(p, 3)) for p in pts]} không có nghiệm IK")


def app_profile(D, slow=SLOW, t_slow=T_SLOW):
    """Hồ sơ tiếp cận quãng D: đoạn nhanh D - slow và đoạn chậm slow (T_SLOW), hai min-jerk cộng chồng (đoạn chậm bắt đầu ở 70%
    đoạn nhanh). Trả (x(t), tổng thời lượng)."""
    d1 = max(D - slow, 0.0)
    T1 = max(T_FAST_MIN, 1.875 * d1 / V_ARM, math.sqrt(5.774 * max(d1, 1e-6) / A_ARM), (60 * d1 / J_APP) ** (1 / 3)) if d1 > 1e-4 else 0.0
    d2 = D - d1
    T2 = t_slow if d2 > 1e-4 else 0.0
    t2s = 0.7 * T1 if T1 > 0 else 0.0
    Ttot = max(T1, t2s + T2)
    return (lambda t: (d1 * mj(t / T1) if T1 > 0 else d1) + (d2 * mj((t - t2s) / T2) if T2 > 0 else d2)), Ttot


def approach(E, side, pre, goal, pitch, yaw, roll_mode=False, close_to=None, t_close=T_CLOSE, slow=SLOW, v_end=V_END, close_tail=0.0,
             grips=()):
    """Tiếp cận chạm: đoạn nhanh pre -> (goal lùi slow) kết thúc ở v_end, rồi đoạn chậm tới goal (v về 0), hai quintic nối nhau.
    close_to: nếu có, bắt đầu khép kẹp tới giá trị này khi TCP còn cách đích CLOSE_AHEAD; generator kết thúc khi kẹp khép xong,
    hoặc sớm close_tail giây (bước sau khép nốt bằng hàm trả về: t -> đích kẹp, để tay đi tiếp ngay khi kẹp đã khép gần hết)."""
    pre, goal = np.asarray(pre, float), np.asarray(goal, float)
    D = float(np.linalg.norm(goal - pre))
    u = (goal - pre) / D if D > 1e-9 else np.zeros(3)
    d1 = max(D - slow, 0.0)
    T1 = max(T_FAST_MIN, 1.875 * d1 / V_ARM, math.sqrt(5.774 * max(d1, 1e-6) / A_ARM), (60 * d1 / J_APP) ** (1 / 3)) if d1 > 1e-4 else 0.0
    d2 = D - d1
    T2 = T_SLOW if d2 > 1e-4 else 0.0
    # hai đoạn min-jerk cộng chồng: đoạn chậm (2 cm cuối) bắt đầu ở 70% đoạn nhanh (vận tốc, gia tốc liên tục; trước 25/11 tối:
    # hai Hermite nối ở v_end, gia tốc liên tục nhưng jerk nhảy ở chỗ nối)
    t2s = 0.7 * T1 if T1 > 0 else 0.0
    Ttot = max(T1, t2s + T2)
    n1, n2 = int(round(Ttot / E.dt)), 0
    gk = spec.joint_name(side, "gripper")
    g0 = E.r._targets[gk]
    t_start_close = None
    i_all = 0
    last_q = None

    def q_at(x):
        p = pre + u * x
        return E.ik(side, p, pitch, roll=yaw) if roll_mode else E.ik(side, p, pitch, yaw)
    for k in range(n1 + n2):
        i_all += 1
        t = (k + 1) * E.dt
        x = (d1 * mj(t / T1) if T1 > 0 else d1) + (d2 * mj((t - t2s) / T2) if T2 > 0 else d2)
        if k + 1 == n1:
            x = D
        q = q_at(x)
        d = {}
        if q is not None:
            last_q = q
            d = E.jd(side, q)
        else:
            E.ik_miss += 1
        for gf in grips:            # lịch kẹp của bước trước chạy nốt (O&C); phần khép bên dưới ghi đè khi bắt đầu khép
            d.update(gf(t))
        if close_to is not None:
            if t_start_close is None and D - x <= CLOSE_AHEAD:
                t_start_close = i_all * E.dt
                if grips:
                    g0 = E.r._targets[gk]
            if t_start_close is not None:
                d[gk] = g0 + (close_to - g0) * mj((i_all * E.dt - t_start_close) / t_close)
        yield d
    if last_q is None and (n1 + n2) > 0:
        raise ValueError(f"tay {side}: tiếp cận {list(np.round(goal, 3))} không có nghiệm IK")
    if close_to is not None:
        if t_start_close is None:
            t_start_close = i_all * E.dt
        while i_all * E.dt < t_start_close + t_close - close_tail - 1e-9:
            i_all += 1
            yield {gk: g0 + (close_to - g0) * mj((i_all * E.dt - t_start_close) / t_close)}
        t_done = i_all * E.dt - t_start_close
        return lambda t: {gk: g0 + (close_to - g0) * mj((t + t_done) / t_close)}
    return lambda t: {}


def drive(E, x, y, lift_to=None, t_lift=None, via=None, tag=None, overlap=LEG_OVERLAP, lift_delay=0.0, acc=BASE_ACC):
    """Đế tới (x, y) thế giới (yaw 0) theo vận tốc min-jerk, gia tốc đỉnh BASE_ACC (< 0,05 m/s²). lift_to: trục nâng chạy cùng lúc.
    via: các điểm giữa (thế giới); chặng sau bắt đầu khi chặng trước đi được `overlap` thời lượng (cộng chồng vận tốc: không dừng
    giữa chặng, góc bo vài cm). tag: ghi E.st["rem_<tag>"] (giây còn lại) và E.st["end_<tag>"] (tư thế cuối dự đoán = tư thế thật +
    quãng còn lại theo hồ sơ) mỗi chu kỳ, để tay bắt đầu bước sau trước khi đế dừng hẳn (đế còn đi < 0,1 mm)."""
    bx, by, _ = E.r.backend.base_pose()
    pts = [(bx, by)] + [tuple(v) for v in (via or [])] + [(x, y)]
    legs = []
    t0 = 0.0
    for (ax, ay), (cx, cy) in zip(pts, pts[1:]):
        dx, dy = cx - ax, cy - ay
        d = math.hypot(dx, dy)
        T = max(0.8, math.sqrt(5.774 * d / acc), 1.875 * d / 0.35) if d > 1e-4 else 0.0
        legs.append((t0, T, dx, dy))
        t0 += T * overlap
    Ttot = max([a + T for a, T, _, _ in legs] + [0.0])
    n = int(round(Ttot / E.dt))
    l0 = E.r._targets["lift"]
    tl = t_lift or max(Ttot * 0.45, 1.0)
    nl = int(round(tl / E.dt)) if lift_to is not None else 0
    nd = int(round(lift_delay / E.dt))     # lift bắt đầu sau lift_delay giây (P&P: đế rời mép bàn trước khi hạ lift)

    def rem_disp(t):
        rx = ry = 0.0
        for a, T, dx, dy in legs:
            if T > 0:
                f = 1.0 - mj((t - a) / T)
                rx += dx * f
                ry += dy * f
        return rx, ry
    for i in range(max(n, nd + nl)):
        out = {}
        t = (i + 0.5) * E.dt
        if i < n:
            vx = vy = 0.0
            for a, T, dx, dy in legs:
                if T > 0 and a <= t <= a + T:
                    v = mj_d((t - a) / T) / T
                    vx += dx * v
                    vy += dy * v
            out["_base"] = (vx, vy)
        elif i == n:
            out["_base"] = (0.0, 0.0)
        if lift_to is not None and nd <= i < nd + nl:
            out["lift"] = l0 + (lift_to - l0) * mj((i - nd + 1) / nl)
        if tag:
            b = E.r.backend.base_pose()
            rx, ry = rem_disp(i * E.dt)
            E.st[f"rem_{tag}"] = max(0.0, Ttot - i * E.dt)
            E.st[f"end_{tag}"] = (b[0] + rx, b[1] + ry)
        yield out
    if tag:
        E.st[f"rem_{tag}"] = 0.0
    yield {"_base": (0.0, 0.0)}
    # chờ dốc về 0 (đế đã gần đứng: hồ sơ min-jerk kết thúc ở v 0)
    for _ in range(int(math.ceil(0.02 / E.dt))):
        yield {"_base": (0.0, 0.0)}


def base_near_end(E, tag, lead=BASE_LEAD):
    """Chờ tới khi đế (drive có tag) còn <= lead giây; trả tư thế cuối dự đoán (x, y)."""
    yield from wait_until(E, lambda: E.st.get(f"rem_{tag}", 1e9) <= lead, timeout=60)
    b = E.st.get(f"end_{tag}")
    if b is None:
        b = tuple(E.r.backend.base_pose()[:2])
    return b


def flag_at(E, key, t0):
    """Dùng trong grips của cpath/jmove: đặt cờ E.st[key] = True từ giờ t0 của đoạn."""
    def f(t):
        if t >= t0:
            E.st[key] = True
        return {}
    return f


def wait_until(E, cond, timeout=30.0):
    n = 0
    while not cond():
        n += 1
        if n * E.dt > timeout:
            raise TimeoutError("chờ cờ đồng bộ quá lâu")
        yield {}


# ---------------- gắp đặt chĩa xuống ----------------

def down_grasp(E, side, center_w, half_w, half_h, gy, z_grasp, base=None):
    """(TCP kẹp, TCP trên vật) khung robot cho vật tâm center_w (thế giới), dụng cụ chĩa xuống, trục đóng theo gy."""
    c = E.w2r([center_w[0], center_w[1], z_grasp], base)
    t = E.r.grasp_point(side, c, half_w, grip_yaw=gy)
    g = c if t is None else t
    top = L.TABLE_H + 2 * half_h
    return np.asarray(g, float), np.array([g[0], g[1], max(top + ABOVE, z_grasp + ABOVE)])


def pick_down(E, side, oid, open_to=100, via=None, cands=PITCH_DOWN, alt=False, grips=(), close_tail=0.0, base=None):
    """Từ chỗ hiện tại tới trên vật (kẹp mở dần từ 50% đoạn), tiếp cận, khép. Trả (g, above, pitch, gy).
    grips: lịch kẹp thêm cho đoạn tới trên vật (vd. nhả nốt vật trước). close_tail: kết thúc sớm khi kẹp còn close_tail giây;
    phần khép còn lại ở E.st["close_cont_<tay>"] (t -> đích kẹp) cho bước sau."""
    p = LC.obj_pos(E.r, oid)
    gy = LC.obj_yaw(E.r, oid) + (math.pi / 2 if alt else 0.0)       # alt: khép theo cặp mặt kia (lần gắp lại)
    hw = LC.obj_half_w(oid) if not alt else _half_y(oid)
    # hạ kẹp với khe PICK_MARGIN giữa ngón cố định và vật (ngón cố định sát mép vật thì hạ xuống chạm vành vật trụ, làm đổ vật);
    # hàm động đẩy vật về ngón cố định khi khép
    g, above = down_grasp(E, side, p, hw + PICK_MARGIN, LC.obj_half_h(oid), gy, LC.grasp_z(oid), base)
    pd = E.pick_pitch(side, [above, g], gy, cands)
    cur = E.tcp_now(side)
    pts = [cur] + ([np.asarray(via, float)] if via is not None else []) + [above]
    p0 = _pitch_of(E, side)
    gk = spec.joint_name(side, "gripper")
    y0 = _gy_of(E, side)
    if via is not None:     # đoạn rút lên qua `via` giữ góc cũ, xoay sang góc gắp mới ở đoạn sau (xoay trên đoạn ngắn làm IK nhảy nhánh)
        pit, yws = [p0, p0, pd], [y0, y0, gy]
    else:
        pit, yws = [p0] + [pd] * (len(pts) - 1), [y0] + [gy] * (len(pts) - 1)
    bl = 0.6 if via is not None else BLEND      # qua điểm rút: đoạn sau bắt đầu sớm hơn (đoạn xoay dài khởi động chậm)
    T = path_sched(pts, pit, yws, blend=bl)[2]
    if grips:
        gl = list(grips)
    else:
        gl = [grip_sched(side, E.r._targets[gk], open_to, 0.5 * T, 0.5 * T)]
    # tiếp cận (trên vật -> chỗ kẹp) bắt đầu khi đường tới trên vật còn APP_LEAD giây (cộng chồng, không dừng ở điểm trên vật);
    # khép khi TCP còn cách chỗ kẹp CLOSE_AHEAD; generator kết thúc sớm close_tail giây (phần khép còn lại ở E.st["close_cont_<tay>"])
    D = float(np.linalg.norm(g - above))
    xf, Ta = app_profile(D)
    u = (g - above) / max(D, 1e-9)
    t_then = max(0.0, T - APP_LEAD)
    tc = next((k * E.dt for k in range(int(Ta / E.dt) + 2) if D - xf(k * E.dt) <= CLOSE_AHEAD), Ta)
    g_open = float(open_to)
    t_c0 = t_then + tc
    gl2 = gl + [grip_sched(side, g_open, 0.0, t_c0, T_CLOSE)]
    T_all = max(T, t_then + Ta)
    stop = max(T_all, t_c0 + T_CLOSE - close_tail)
    Tpath = T
    gen = cpath(E, side, pts, pit, yws, Tpath, grips=gl2, blend=bl, then=(lambda tau: u * xf(tau), Ta, APP_LEAD))
    k = 0
    for d in gen:
        k += 1
        yield d
    while k * E.dt < stop - 1e-9:      # khép tiếp tại chỗ kẹp
        k += 1
        yield {gk: g_open + (0.0 - g_open) * mj((k * E.dt - t_c0) / T_CLOSE)}
    t_done = k * E.dt - t_c0
    E.st[f"close_cont_{side}"] = lambda t: {gk: g_open + (0.0 - g_open) * mj((t + t_done) / T_CLOSE)}
    E.log(f"  > {side} kẹp {oid} alt={alt} pitch {pd:.0f} gy {gy:.2f} kẹp {E.r.backend.read_joints()[gk]:.1f} TCP {np.round(E.r.tcp_position(side), 3).tolist()} đích {np.round(g, 3).tolist()} vật {np.round(E.r.world_to_robot(LC.obj_pos(E.r, oid)), 3).tolist()}")
    if E.r.backend.read_joints()[gk] < 3.0:      # kẹp khép hết: trượt vật (phản hồi vị trí servo kẹp); mở, hạ lại theo vị trí vật mới
        E.retries += 1
        p = LC.obj_pos(E.r, oid)
        gy = LC.obj_yaw(E.r, oid)
        g, above = down_grasp(E, side, p, LC.obj_half_w(oid), LC.obj_half_h(oid), gy, LC.grasp_z(oid))
        pd = E.pick_pitch(side, [above, g], gy, cands)
        up = E.tcp_now(side) + [0, 0, 0.03]
        yield from cpath(E, side, [E.tcp_now(side), up, above], [pd, pd, pd], [gy, gy, gy],
                         grips=[grip_sched(side, E.r._targets[gk], 100, 0.0, 0.3)])
        E.st[f"close_cont_{side}"] = yield from approach(E, side, above, g, pd, gy, close_to=0, close_tail=close_tail)
    return g, above, pd, gy


def _half_y(oid):
    for i, sh, size, _ in L.OBJECTS + [L.FRIDGE_OBJ]:
        if i == oid:
            return size[1] / 2 if sh == "box" else size[0]
    raise KeyError(oid)


def _pitch_of(E, side):
    """Góc ngẩng dụng cụ (độ) của đích hiện tại (FK SDK)."""
    return math.degrees(float(K.arm_fk(E.q_now(side), side)[1]))


def _gy_of(E, side):
    return E.st.get(f"gy_{side}", 0.0)


def place_down(E, side, oid, xy_w, gy, drop_h, open_to, via=None, T=None, wait_flag=None, wait_pt=None, cands=PITCH_DOWN,
               grips=(), merge=False, release_lead=0.0, t_open=None, base=None, above_z=None):
    """Mang vật đang kẹp tới trên chỗ đặt (xy thế giới, hoặc hàm trả xy), tiếp cận; trả (TCP đặt, TCP trên, góc ngẩng).
    wait_flag: chưa được đặt thì tới wait_pt (khung robot) chờ cờ; trong lúc chờ E.st["<tay>_idle"] = True (đế được phép dời)."""
    if wait_flag is not None and not wait_flag():
        wp = np.asarray(wait_pt, float)
        yield from cpath(E, side, [E.tcp_now(side), wp], [_pitch_of(E, side), cands[0]], [_gy_of(E, side), gy])
        E.st[f"{side}_idle"] = True
        yield from wait_until(E, wait_flag)
        E.st[f"{side}_idle"] = False
    if callable(xy_w):
        xy_w = xy_w()
    z = LC.grasp_z(oid) + drop_h
    top = z + LC.obj_half_h(oid)
    pd = None
    # sát trần gập: thử trục khép xoay 90 độ (lệch ngón cố định sang trục kia), rồi dời chỗ đặt sâu thêm tối đa 2 cm
    for gy_t, sh in ((gy, 0.0), (gy + math.pi / 2, 0.0), (gy, 0.01), (gy + math.pi / 2, 0.01), (gy, 0.02), (gy + math.pi / 2, 0.02)):
        c = E.w2r([xy_w[0] + sh, xy_w[1], z], base)
        t = E.r.grasp_point(side, c, LC.obj_half_w(oid), grip_yaw=gy_t)
        g = np.asarray(c if t is None else t, float)
        above = np.array([g[0], g[1], max(top + ABOVE, LC.CLEAR_Z) if above_z is None else above_z])
        try:
            pd = E.pick_pitch(side, [above, g], gy_t, cands)
            gy = gy_t
            break
        except ValueError:
            continue
    if pd is None:
        raise ValueError(f"tay {side}: chỗ đặt {list(np.round(xy_w, 3))} ngoài tầm")
    cur = E.tcp_now(side)
    p0, y0 = _pitch_of(E, side), _gy_of(E, side)
    pts = [cur] + ([np.asarray(via, float)] if via is not None else []) + [above]
    if merge:
        # một đường: (nhấc) -> trên chỗ đặt -> hạ đặt, không dừng ở điểm giữa; kẹp bắt đầu mở release_lead giây trước cuối đoạn hạ
        pts.append(g)
        pit, yws = [p0] + [pd] * (len(pts) - 1), [y0] + [gy] * (len(pts) - 1)
        Tp = path_sched(pts, pit, yws)[2]
        gk = spec.joint_name(side, "gripper")
        gl = list(grips) + [grip_sched(side, 0.0, open_to, Tp - release_lead, t_open or T_OPEN)]
        E.st[f"open_t0_{side}"] = release_lead
        yield from cpath(E, side, pts, pit, yws, grips=gl, blend=PLACE_BLEND)
    else:
        yield from cpath(E, side, pts, [p0] + [pd] * (len(pts) - 1), [y0] + [gy] * (len(pts) - 1), T, grips=grips)
        yield from approach(E, side, above, g, pd, gy)
    E.st[f"gy_{side}"] = gy
    return g, above, pd


def release_retract(E, side, g, pd, gy, open_to, up=0.06, close_after=None, on_clear=None, t_open=None):
    """Nhả trong T_OPEN; khi kẹp mở 60% thì bắt đầu rút thẳng lên `up` (0,4 s), kẹp tiếp tục mở trong lúc rút."""
    gk = spec.joint_name(side, "gripper")
    g0 = E.r._targets[gk]
    To = t_open or T_OPEN
    sched = grip_sched(side, g0, open_to, 0.0, To)
    k60 = 0
    while k60 * E.dt < To and mj(k60 * E.dt / To) < 0.6:
        k60 += 1
    for i in range(1, k60 + 1):
        yield sched(i * E.dt)
    up_pt = np.array([g[0], g[1], g[2] + up])
    off = k60 * E.dt
    yield from cpath(E, side, [g, up_pt], [pd, pd], [gy, gy], grips=[lambda t: sched(t + off)])
    if on_clear:
        on_clear()


def release_open(E, side, open_to, t_open=None, frac=0.6):
    """Nhả tiếp (kẹp đã bắt đầu mở E.st["open_t0_<tay>"] giây trước) tới khi mở `frac`; trả hàm t -> đích kẹp cho phần mở còn lại
    (bước sau, vd. đường tới vật kế tiếp, dùng làm grips)."""
    To = t_open or T_OPEN
    t0 = E.st.pop(f"open_t0_{side}", 0.0)
    gk = spec.joint_name(side, "gripper")
    g0 = 0.0 if t0 > 0 else E.r._targets[gk]
    sched = grip_sched(side, g0, open_to, -t0, To)
    k = 0
    while (k * E.dt + t0) < To and mj((k * E.dt + t0) / To) < frac:
        k += 1
        yield sched(k * E.dt)
    off = k * E.dt
    return lambda t: sched(t + off)


# ---------------- Sorting ----------------

def _set(E, k, v):
    def f():
        E.st[k] = v
    return f


def sorting_pad(robot, jobs, cells, slots, open_to, snap=None, log=print):
    """Bản vòng 1-2 (không còn dùng mặc định, giữ để so sánh): đế đứng yên ở S. Tay trái gắp vật trên bàn, đặt lên đệm đổi tay M (giữa thân); tay phải gắp từ M, đặt vào ngăn.
    Hai tay gối nhau: tay phải mang vật k vào khay trong lúc tay trái gắp vật k+1. Trả danh sách (oid, màu) đã làm."""
    snap = snap or (lambda *a, **k: None)
    E = Engine(robot)
    E.log = log
    lift = L.LIFT_WORK
    snap("1-bat-dau")
    # vào chỗ đứng: đế, trục nâng, hai tay chuẩn bị cùng lúc
    sx, sy = S_SORT
    ready_l = E.ik_down("left", [0.33, 0.15, 0.86], lift=lift)
    wait_r_robot = np.array([PAD[0] + 0.02, PAD[1] - 0.12, 0.87])
    wait_r = E.ik_down("right", wait_r_robot, lift=lift)
    if ready_l is None or wait_r is None:
        raise ValueError("tư thế chuẩn bị ngoài tầm")
    E.run(drive(E, sx, sy, lift_to=lift, t_lift=2.5), jmove(E, "left", ready_l, 2.5, t_start=1.5),
          jmove(E, "right", wait_r, 2.5, t_start=1.5))
    E.st.update({"pad_obj": None, "pad_free": True, "left_clear": True, "gy_left": 0.0, "gy_right": 0.0, "skipped": set(),
                 "right_idle": False, "base_moving": False, "left_idle": False})
    used = {c: 0 for c in cells}
    plan = []
    for oid, color in jobs:
        dx, dy = slots[used[color] % len(slots)]
        used[color] += 1
        cx, cy, _ = cells[color]["center"]
        plan.append((oid, (cx + dx, cy + dy)))
    pad_world = [sx + PAD[0], sy + PAD[1]]
    bx, by, _ = robot.backend.base_pose()
    pad_world = np.array([bx + PAD[0], by + PAD[1]])
    home_y = by
    wait_l = np.array([PAD[0] + 0.02, PAD[1] + 0.12, 0.86])     # chỗ tay trái chờ đệm trống (cao, lệch về phía trái)

    def left_loop():
        nonlocal pad_world
        for k, (oid, _) in enumerate(plan):
            b = E.r.backend.base_pose()
            yr = LC.obj_pos(E.r, oid)[1] - b[1]
            if yr > REACH_Y:
                # vật ngoài tầm với thật của tay trái (IK có nghiệm nhưng cẳng tay chạm mép bàn khi hạ, sim): dời đế sang +y vừa đủ,
                # lúc tay phải đang chờ trên đệm trống (không cầm vật); đệm dời theo đế (khung robot giữ nguyên)
                E.st["left_idle"] = True
                yield from wait_until(E, lambda: E.st["pad_free"] and E.st["right_idle"] and not E.st["base_moving"], timeout=60)
                E.st["left_idle"] = False
                E.st["base_moving"] = True
                b = E.r.backend.base_pose()
                yr = LC.obj_pos(E.r, oid)[1] - b[1]
                yield from drive(E, b[0], b[1] + yr - REACH_Y + 0.005)
                b = E.r.backend.base_pose()
                pad_world = np.array([b[0] + PAD[0], b[1] + PAD[1]])
                E.st["base_moving"] = False
                log(f"  đế dời tới y {b[1]:.3f}")
            held = False
            for _try in range(3):
                g, above, pd, gy = yield from pick_down(E, "left", oid, alt=_try == 1)
                E.st["gy_left"] = gy
                lift_pt = np.array([g[0], g[1], g[2] + 0.04])
                yield from cpath(E, "left", [g, lift_pt], [pd, pd], [gy, gy], T=0.4)
                held = LC.obj_pos(E.r, oid)[2] > L.TABLE_H + LC.obj_half_h(oid) + 0.02
                if held:
                    break
                E.retries += 1
                yield from gripper_to(E, "left", 100, 0.3)
            log(f"  trái gắp {oid} t={E.r.backend.data.time:.1f} giữ {held}")
            if not held:
                E.st["skipped"].add(oid)
                continue
            E.st["left_clear"] = False
            gp, ab, pd2 = yield from place_down(E, "left", oid, lambda: pad_world, gy, 0.004, PAD_OPEN,
                                                wait_flag=lambda: E.st["pad_free"] and not E.st["base_moving"], wait_pt=wait_l, cands=PAD_PITCH)
            E.st["pad_free"] = False
            E.st["pad_obj"] = oid
            log(f"  trái đặt {oid} lên đệm t={E.r.backend.data.time:.1f} vật {np.round(LC.obj_pos(E.r, oid), 3).tolist()}")
            yield from release_retract(E, "left", gp, pd2, gy, PAD_OPEN, up=0.06, on_clear=_set(E, "left_clear", True), t_open=PAD_T_OPEN)
            log(f"  trái rút xong t={E.r.backend.data.time:.1f} vật {np.round(LC.obj_pos(E.r, oid), 3).tolist()}")
            if k == len(plan) // 2:
                snap("2-dang-kep")
        E.st["left_idle"] = True
        home = np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS])
        yield from jmove(E, "left", home, 1.5)

    def right_loop():
        nonlocal pad_world
        for k, (oid, dst) in enumerate(plan):
            E.st["right_idle"] = True
            yield from wait_until(E, lambda: ((E.st["pad_obj"] == oid and E.st["left_clear"]) or oid in E.st["skipped"])
                                  and not E.st["base_moving"], timeout=60)
            E.st["right_idle"] = False
            if oid in E.st["skipped"]:
                continue
            log(f"  phải tới đệm {oid} t={E.r.backend.data.time:.1f} vật {np.round(LC.obj_pos(E.r, oid), 3).tolist()} v {LC.obj_speed(E.r, oid):.3f}")
            held = False
            for _try in range(3):
                if np.linalg.norm(LC.obj_pos(E.r, oid)[:2] - pad_world) > 0.05:
                    break                                   # vật không nằm trên đệm (rơi): bỏ qua
                g, above, pd, gy = yield from pick_down(E, "right", oid, open_to=100, cands=PAD_PITCH, alt=_try == 1)
                E.st["gy_right"] = gy
                lift_pt = np.array([g[0], g[1], g[2] + 0.05])
                yield from cpath(E, "right", [g, lift_pt], [pd, pd], [gy, gy], T=0.4)
                held = LC.obj_pos(E.r, oid)[2] > L.TABLE_H + LC.obj_half_h(oid) + 0.02
                if held:
                    break
                E.retries += 1
                yield from gripper_to(E, "right", 100, 0.3)
            if not held:
                E.st["pad_free"] = True
                E.st["pad_obj"] = None
                continue
            E.st["pad_obj"] = None
            log(f"  phải nhấc {oid} t={E.r.backend.data.time:.1f} vật z {LC.obj_pos(E.r, oid)[2]:.3f}")
            b = E.r.backend.base_pose()
            if abs(b[1] - home_y) > 0.03:     # chỉ khi đế đã dời cho vật xa (không tính trôi vài mm)
                # đế đã dời cho vật xa: về lại S (khay trong tầm tay phải) khi tay trái không hạ/không cầm vật trên bàn
                yield from wait_until(E, lambda: E.st["left_idle"] and not E.st["base_moving"], timeout=60)
                E.st["base_moving"] = True
                yield from drive(E, b[0], home_y)
                b = E.r.backend.base_pose()
                pad_world = np.array([b[0] + PAD[0], b[1] + PAD[1]])
                E.st["base_moving"] = False
                log(f"  đế về y {E.r.backend.base_pose()[1]:.3f}")
            E.st["pad_free"] = True             # đệm trống sau khi đế đã về (đế dời thì đệm dời theo)
            gp, ab, pd2 = yield from place_down(E, "right", oid, dst, 0.0, 0.008, open_to)
            yield from release_retract(E, "right", gp, pd2, E.st["gy_right"], open_to, up=0.06, t_open=TRAY_T_OPEN)
            if k + 1 < len(plan):
                yield from cpath(E, "right", [E.tcp_now("right"), wait_r_robot], [pd2, -90.0], [0.0, 0.0])
        home = np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS])
        yield from jmove(E, "right", home, 1.5, grips=[grip_sched("right", open_to, 0, 0.0, 0.5)])

    E.run(left_loop(), right_loop())
    snap("3-xong")
    return [(oid, color) for oid, color in jobs]


# ---------------- Sorting vòng 3: hai tay cùng gắp, đế chở sang khay ----------------
# Vì sao không dùng đệm đổi tay (sorting_pad): mỗi vật 4 lần chạm (gắp, đặt đệm, gắp đệm, đặt khay) thay vì 2, vật trụ đổ trên đệm,
# và hai tay chờ nhau qua cờ đệm thì kẹt (8/20 seed hết giờ ở vòng 2). Vật trên bàn ở y +0,12..+0,54, khay ở y -0,21..-0,43 (thế giới):
# một chỗ đứng không có tay nào với tới cả hai, nên mỗi chuyến hai tay gắp hai vật cách nhau 18-24 cm, đế chở sang khay, hai tay đặt.
# Cặp vật chọn để tay trái (phía +y) luôn đặt vào ngăn ở phía +y so với tay phải (hai tay không bắt chéo); cặp cùng ngăn thì tay phải
# đặt trước, tay trái đặt khi tay phải đã rút. Đế đứng yên khi tay chạm vật; dốc đế 0,05 m/s² giữ nguyên.
SORT_X = 0.30           # x đế (thế giới) suốt bài: vật và khay ở x robot 0,33
SORT_CARRY_Z = 0.85     # độ cao TCP khi mang vật / chờ (đáy vật cao hơn đỉnh vật trên bàn và vách khay >= 3 cm)
SORT_EARLY = 0.8        # tay bắt đầu đường xuống vật/ô khi chặng đế còn 0,8 s (đích tính theo tư thế cuối dự đoán, đế còn đi < 1 cm)
SORT_SIDE_Y = 0.09      # chuyến một vật: vật ở |y| 0,09 khung robot
SORT_LIFT_T = 1.2       # nhấc vật lên độ cao mang (đế bắt đầu đi khi đã nhấc 4 cm và vật còn trong kẹp)
SORT_DROP = 0.008       # đặt cao hơn đáy khay
SORT_ROT_T = 2.0        # xoay cổ tay 90 độ tại chỗ mang (trong lúc đế chạy)
SORT_BASE_ACC = BASE_ACC
SORT_OPEN_FRAC = 0.4     # rút khi kẹp đã mở 40% quãng (đo 5 seed: 0,6 -> 0,4 giảm jerk p95 ~0,1)
SORT_OPEN_ADD = 6.0     # nhả: mở tới độ kẹp đang giữ + 6 (tối đa RELEASE_OPEN). Mở rộng hơn thì hàm động đập vào vách/vật bên cạnh
                        # (seed 5: +30 -> jerk p95 9,4; +15 -> 6,0; +8 -> 4,6; +6 -> 4,2-4,5); vật vẫn rời ngón khi rút
# (vật tay trái, chỗ), (vật tay phải, chỗ); chỗ (dx, dy, trục khép) so với tâm ngăn. Hai chỗ trước dx -0,02 (trục khép theo x: ngón
# trước cách vách trước >= 1 cm; dx -0,03 thì ngón trước tì lên đỉnh vách, vật rơi 2 cm khi nhả), chỗ sau dx +0,035 với trục khép
# theo y (ngón hai bên, không chen vào khe giữa vật sau và hai vật trước; trục khép theo x thì ngón sau tì vách sau, kẹt khi rút)
F_L, F_R, BACK = (-0.02, 0.025, 0.0), (-0.02, -0.025, 0.0), (0.035, 0.0, math.pi / 2)
SORT_TRIPS = [      # (trái), (phải), tay đặt trước khi hai vật cùng ngăn
    (("obj_red_cyl", F_L), ("obj_red_cube", F_R), "right"),
    (("obj_green_cyl", F_L), ("obj_blue_cyl", F_R), None),
    (("obj_red_block", BACK), ("obj_green_cube", F_R), None),
    (("obj_blue_block", F_L), ("obj_blue_cube", BACK), "left"),     # khối 4,5 cm ở chỗ sau (rộng); chỗ trước đặt trước
]


def _sort_plan(E, jobs, cells, slots):
    color = dict(jobs)
    used = {c: 0 for c in cells}
    trips, seen = [], set()

    def dst(oid, sl):
        cx, cy, _ = cells[color[oid]]["center"]
        return (cx + sl[0], cy + sl[1])
    for (ol, sl), (orr, sr), first in SORT_TRIPS:
        tr = {"left": ol if ol in color else None, "right": orr if orr in color else None, "slot": {"left": sl, "right": sr},
              "first": first or "right"}
        if tr["left"] or tr["right"]:
            trips.append(tr)
            seen.update(o for o in (tr["left"], tr["right"]) if o)
    for oid, c in jobs:              # vật ngoài bảng chuyến (bố cục khác): chuyến một vật bằng tay trái, chỗ trong ngăn theo slots
        if oid not in seen:
            trips.append({"left": oid, "right": None, "slot": {"left": tuple(slots[used[c] % len(slots)]) + (0.0,)}})
            used[c] += 1
    for tr in trips:
        sides = [s for s in ("left", "right") if tr[s]]
        tr["sides"] = sides
        tr["dst"] = {s: dst(tr[s], tr["slot"][s]) for s in sides}
        oy = {s: float(LC.obj_pos(E.r, tr[s])[1]) for s in sides}
        sg = {"left": 1.0, "right": -1.0}
        if len(sides) == 2:
            tr["pick_y"] = (oy["left"] + oy["right"]) / 2
            tr["place_y"] = (tr["dst"]["left"][1] + tr["dst"]["right"][1]) / 2
        else:
            s = sides[0]
            tr["pick_y"] = oy[s] - sg[s] * SORT_SIDE_Y
            tr["place_y"] = tr["dst"][s][1] - sg[s] * SORT_SIDE_Y
        tr["same"] = len(sides) == 2 and color[tr["left"]] == color[tr["right"]]
    return trips


def sorting(robot, jobs, cells, slots, open_to, snap=None, log=print):
    """Sorting vòng 3: mỗi chuyến hai tay gắp hai vật trên bàn cùng lúc (đế đứng), đế chở sang khay (tay giữ vật ở độ cao mang),
    hai tay đặt vào ngăn. Tay bắt đầu xuống khi chặng đế còn SORT_EARLY giây; đế đi tiếp khi hai tay đã nhấc vật / đã nhả.
    Trả danh sách (oid, màu) đã làm."""
    snap = snap or (lambda *a, **k: None)
    E = Engine(robot)
    E.log = log
    lift = L.LIFT_WORK
    snap("1-bat-dau")
    trips = _sort_plan(E, jobs, cells, slots)
    sg = {"left": 1.0, "right": -1.0}
    carry = {s: np.array([0.33, sg[s] * 0.12, SORT_CARRY_Z]) for s in sg}
    ready = {s: E.ik_down(s, carry[s], lift=lift) for s in sg}
    if ready["left"] is None or ready["right"] is None:
        raise ValueError("tư thế mang ngoài tầm")
    E.st.update({"gy_left": 0.0, "gy_right": 0.0, "skipped": set()})
    home = np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS])

    def base_loop():
        for n, tr in enumerate(trips):
            yield from drive(E, SORT_X, tr["pick_y"], lift_to=lift if n == 0 else None, t_lift=2.5, tag=f"p{n}", acc=SORT_BASE_ACC)
            yield from wait_until(E, lambda: all(E.st.get(f"up_{s}_{n}") for s in ("left", "right")), timeout=60)
            yield from drive(E, SORT_X, tr["place_y"], tag=f"q{n}", acc=SORT_BASE_ACC)
            yield from wait_until(E, lambda: all(E.st.get(f"rel_{s}_{n}") for s in ("left", "right")), timeout=60)

    def arm_loop(side):
        yield from jmove(E, side, ready[side], 2.5, t_start=1.5)
        gk = spec.joint_name(side, "gripper")
        for n, tr in enumerate(trips):
            oid = tr[side]
            if oid is None:
                E.st[f"up_{side}_{n}"] = E.st[f"rel_{side}_{n}"] = E.st[f"clr_{side}_{n}"] = True
                continue
            # gắp: đích theo tư thế cuối dự đoán của chặng đế
            yield from wait_until(E, lambda: E.st.get(f"rem_p{n}", 1e9) <= SORT_EARLY, timeout=90)
            held = False
            for _try in range(3):
                base = E.st.get(f"end_p{n}") if _try == 0 and E.st.get(f"rem_p{n}", 0.0) > 0 else None
                g, above, pd, gy = yield from pick_down(E, side, oid, alt=_try == 1, close_tail=CLOSE_TAIL, base=base)
                E.st[f"gy_{side}"] = gy
                cont = E.st.pop(f"close_cont_{side}")
                st = {"held": False}

                def chk(t, oid=oid, st=st):
                    if t >= 0.45 and not st["held"] and LC.obj_pos(E.r, oid)[2] > L.TABLE_H + LC.obj_half_h(oid) + 0.02:
                        st["held"] = True
                        E.st[f"up_{side}_{n}"] = True      # đế bắt đầu đi trong lúc tay còn nhấc tiếp
                    return {}
                cz = np.array([g[0], g[1], SORT_CARRY_Z])
                yield from cpath(E, side, [g, g + [0, 0, 0.04], cz], [pd] * 3, [gy] * 3, T=SORT_LIFT_T, grips=[cont, chk])
                held = st["held"]
                if held:
                    break
                E.retries += 1
                yield from gripper_to(E, side, 100, 0.3)
            log(f"  {side} gắp {oid} t={E.r.backend.data.time:.1f} giữ {held}")
            if not held:
                E.st["skipped"].add(oid)
                E.st[f"up_{side}_{n}"] = E.st[f"rel_{side}_{n}"] = E.st[f"clr_{side}_{n}"] = True
                continue
            if n == len(trips) // 2 and side == "left":
                snap("2-dang-kep")
            gyp = tr["slot"][side][2]
            if abs(gyp - gy) > 0.3:      # chỗ sau: xoay cổ tay sang trục khép theo y chậm trong lúc đế chạy (xoay nhanh lúc hạ: vật lắc)
                yield from cpath(E, side, [cz, cz], [pd, pd], [gy, gyp], T=SORT_ROT_T)
                E.st[f"gy_{side}"] = gyp
            # đặt
            yield from wait_until(E, lambda: E.st.get(f"rem_q{n}", 1e9) <= SORT_EARLY, timeout=90)
            base = E.st.get(f"end_q{n}") if E.st.get(f"rem_q{n}", 0.0) > 0 else None
            if tr["same"] and side != tr["first"]:       # cùng ngăn: chờ tay kia rút khỏi ngăn
                yield from wait_until(E, lambda: E.st.get(f"clr_{tr['first']}_{n}"), timeout=60)
                base = None
            # nhả vừa đủ: mở thêm SORT_OPEN_ADD so với độ kẹp đang giữ (mở tới 65 thì hàm động đập vào vách ngăn/vật bên cạnh, kẹt khi rút)
            op = min(open_to, E.r.backend.read_joints()[gk] + SORT_OPEN_ADD)
            gp, ab, pd2 = yield from place_down(E, side, oid, tr["dst"][side], tr["slot"][side][2], SORT_DROP, op, merge=True,
                                                release_lead=RELEASE_LEAD, t_open=TRAY_T_OPEN, base=base)
            log(f"  {side} hạ {oid} t={E.r.backend.data.time:.1f} TCP {np.round(E.r.tcp_position(side), 3).tolist()} đích {np.round(gp, 3).tolist()} kẹp {E.r.backend.read_joints()[gk]:.1f} mở {op:.0f}")
            opn = yield from release_open(E, side, op, t_open=TRAY_T_OPEN, frac=SORT_OPEN_FRAC)
            E.st[f"rel_{side}_{n}"] = True
            log(f"  {side} đặt {oid} t={E.r.backend.data.time:.1f}")
            # rút lên rồi lùi về phía tay mình (cặp cùng ngăn: tay sau bắt đầu xuống khi tay trước đã rút 60% đường)
            up = gp + [0, 0, 0.06]
            out = np.array([gp[0], gp[1] + sg[side] * 0.05, SORT_CARRY_Z])
            gyp = E.st[f"gy_{side}"]
            Tr = path_sched([gp, up, out], [pd2] * 3, [gyp] * 3)[2]
            last = n == len(trips) - 1 or all(t[side] is None for t in trips[n + 1:])
            yield from cpath(E, side, [gp, up, out], [pd2] * 3, [gyp] * 3, grips=[opn, flag_at(E, f"clr_{side}_{n}", 0.6 * Tr)])
            if not last and abs(gyp) > 0.3:      # chỗ sau (trục khép theo y): xoay cổ tay về 0 chậm trong lúc đế chạy
                yield from cpath(E, side, [out, out], [pd2, pd2], [gyp, 0.0], T=SORT_ROT_T)
                E.st[f"gy_{side}"] = 0.0
            E.st[f"clr_{side}_{n}"] = True
            if last:
                yield from jmove(E, side, home, 1.5, grips=[grip_sched(side, E.r._targets[gk], 0, 0.0, 0.5)])

    E.run(base_loop(), arm_loop("left"), arm_loop("right"))
    snap("3-xong")
    return [(oid, color) for oid, color in jobs]


# ---------------- Pick & place (vùng đích) ----------------

def pick_place(robot, objs, targets, snap=None, log=print):
    """Pick & place lab SDK: 2 vật trên bàn vào vùng đích (thảm giữa thân ở chỗ đứng S) và lon trong tủ lạnh.
    Vùng đích của lab SDK nằm giữa thân robot ở S, nên tay trái gắp và đặt thẳng (không cần đổi tay); tay phải mở cửa tủ lạnh."""
    snap = snap or (lambda *a, **k: None)
    E = Engine(robot)
    lift = L.LIFT_WORK
    sx, sy = S_SORT
    can = L.FRIDGE_OBJ[0]
    table = [(o, t) for o, t in zip(objs, targets) if o != can]
    ready_l = E.ik_down("left", [0.33, 0.15, 0.86], lift=lift)
    snap("1-bat-dau")
    E.st.update({"gy_left": 0.0, "rem_s": 1e9})

    def left_loop():
        # vào tư thế sẵn sàng trong lúc đế tới S; gắp vật đầu bắt đầu khi đế còn BASE_LEAD
        yield from jmove(E, "left", ready_l, 2.5, t_start=1.5)
        yield from base_near_end(E, "s")
        # mỗi vật: tới trên vật (qua điểm rút của vật trước, kẹp mở nốt), tiếp cận, khép; một đường nhấc -> trên thảm -> hạ đặt (kẹp khép
        # nốt lúc bắt đầu nhấc, mở từ RELEASE_LEAD s trước cuối đoạn hạ); mở tới 60% rồi đi tiếp. Không có chỗ tay dừng giữa các bước.
        via, extra = None, ()
        for k, (oid, dst) in enumerate(table):
            g, above, pd, gy = yield from pick_down(E, "left", oid, via=via, grips=extra, close_tail=CLOSE_TAIL)
            E.st["gy_left"] = gy
            cont = E.st.pop("close_cont_left")
            gp, ab, pd2 = yield from place_down(E, "left", oid, dst, gy, PP_DROP, 100, via=g + [0, 0, 0.04], grips=[cont], merge=True,
                                                release_lead=RELEASE_LEAD, t_open=PP_T_OPEN)
            opn = yield from release_open(E, "left", 100, t_open=PP_T_OPEN)
            via = gp + [0, 0, 0.06]
            extra = (opn,)
            E.st["last_place"] = (gp, via, pd2, gy, opn)
            if k == 0:
                snap("2-dang-kep")
    E.run(drive(E, sx, sy, lift_to=lift, t_lift=2.5, tag="s"), left_loop())

    def left_leave():
        # rút lên khỏi vật cuối rồi về HOME trước khi đế xa bàn (chạy cùng lúc đế bắt đầu đi tới F1; đế đi < 1 cm trong 1 s đầu).
        # Để tay duỗi trên bàn khi đế đi, hoặc hạ lift khi đế còn sát bàn, thì kẹp/cẳng tay quét qua vật trên thảm và mép bàn
        # (va chạm, vật bị hất, gia tốc đế 0,47 m/s² ở lần đo trước)
        gp, via, pd2, gy, opn = E.st.pop("last_place")
        yield from cpath(E, "left", [gp, via], [pd2, pd2], [gy, gy], grips=[opn])
        yield from jmove(E, "left", np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS]), T_HOME,
                         grips=[grip_sched("left", 100, GRIP_AIR, 0.0, 1.0)])
    E.st["left_pre"] = left_leave
    E.st["lift_delay"] = PP_LIFT_DELAY
    if can in objs:
        tgt = targets[objs.index(can)]
        # lon đặt bằng tay trái ngang, hai ngón kẹp theo phương y: dời chỗ đặt thêm PP_CAN_DY theo -y để ngón cố định có khe với vật trụ
        # ở ô 2; tâm lon vẫn cách mép thảm >= 2 cm
        tgt = (tgt[0] + (PP_CAN_DX if FR5_PULL else 0.0), tgt[1] + PP_CAN_DY)
        fridge_fetch(E.r, can, tgt, snap=snap, log=log, engine=E, place_station=(tgt[0] - PLACE_REACH[0], tgt[1] - PLACE_REACH[1]))
    snap("3-xong")


# ---------------- Tủ lạnh: phải mở cửa, trái lấy lon ----------------

def _door(robot):
    door = L.FRIDGE
    hinge = np.array([door["front_x"] - door["door_t"] / 2, door["center_y"] - door["size"][1] / 2 + 0.005])
    return hinge


def door_pt(robot, th, u, v, z):
    """Điểm thế giới trong khung cửa: u theo pháp tuyến mặt trong (+x khi đóng), v dọc cánh từ bản lề (+y khi đóng)."""
    h = _door(robot)
    c, s = math.cos(th), math.sin(th)
    return np.array([h[0] + c * u - s * v, h[1] + s * u + c * v, z])


def level_offset(E, side, half_w, margin=0.002):
    """Độ lệch (khung robot) của TCP so với tâm vật để vật tựa ngón cố định, dụng cụ ngang, cuộn 0."""
    q = E.q_now(side)
    try:
        d = K.fixed_side_dir(q, side)
    except Exception:
        d = np.array([0.0, 1.0 if side == "left" else -1.0, 0.0])
    return np.asarray(d, float) * (spec.TCP_DY - spec.FIXED_FACE_Y + half_w + margin)


def fridge_fetch(robot, oid, target_xy, snap=None, log=print, engine=None, place_station=None):
    """CHOREO-M1 4.3 (bản đo ở docs/CHOREO-M1-ketqua.md mục 5). F1: tay phải nắm tay nắm, kéo cửa tới PULL_TH, rồi một chuyển động cộng
    chồng: nhả, lùi, vòng qua mép cửa, đẩy mặt trong tới PUSH_TH, rút ra (đế đứng yên). Tay trái vào tư thế ngang chuẩn bị. F2: tay trái
    tiếp cận lon (dụng cụ ngang), khép, nhấc 4 cm; đế lùi tới F2b (lùi thẳng rồi sang +y) trong lúc tay giữ yên; rồi một đường ra khỏi
    tủ, qua góc tủ, tới trên chỗ đặt và hạ đặt; nhả, rút. Bước sau luôn bắt đầu trước khi bước trước dừng hẳn (không đứng hình).
    place_station: None = đặt từ F2b (thiết kế, chỗ đặt (0,56; -0,95)); (x, y) = đế mang lon tới chỗ đứng đó rồi đặt (P&P vào vùng đích)."""
    snap = snap or (lambda *a, **k: None)
    E = engine or Engine(robot)
    hw = 0.008
    z_h = LC.site_world(robot, "fridge_door_handle")[2]
    home = np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS])
    # 1. đế tới F1, lift 0,27; tay phải tới trước tay nắm 8 cm (ngang, cuộn 0) sau khi đế gần tới; tay trái tư thế sẵn sàng ngang
    hp = LC.site_world(robot, "fridge_door_handle")
    F1_ = (hp[0] - FR5_REL[0], hp[1] - FR5_REL[1]) if FR5_PULL else F1
    pre_w = hp + (FR5_PRE if FR5_PULL else np.array([-0.08, 0, 0]))     # điểm chờ trước tay nắm 8 cm
    pre_r = np.array([pre_w[0] - F1_[0], pre_w[1] - F1_[1], pre_w[2]])
    q_pre = E.ik("right", pre_r, 0.0, roll=0.0, lift=LIFT_FRIDGE)
    if FR5_PULL and q_pre is not None:
        # điểm chờ lệch như điểm kẹp (thanh tựa ngón cố định), để khi hạ thẳng xuống thanh nằm giữa hai ngón
        for _ in range(2):
            o5 = np.asarray(K.fixed_side_dir(q_pre, "right"), float) * (spec.TCP_DY - spec.FIXED_FACE_Y + hw + 0.002)
            q5 = E.ik("right", pre_r + [o5[0], o5[1], 0.0], 0.0, roll=0.0, lift=LIFT_FRIDGE)
            if q5 is None:
                break
            q_pre = q5
    can_w = LC.obj_pos(robot, oid)
    ready_l_r = np.array([0.475, 0.10, can_w[2] + 0.012])
    q_ready_l = E.ik("left", ready_l_r, 0.0, roll=0.0, lift=LIFT_FRIDGE)
    if q_pre is None or q_ready_l is None:
        raise ValueError("tủ lạnh: tư thế chuẩn bị ngoài tầm")
    bx, by, _ = robot.backend.base_pose()
    dT = math.sqrt(5.774 * math.hypot(F1_[0] - bx, F1_[1] - by) / BASE_ACC)
    gk_r = spec.joint_name("right", "gripper")
    E.st["rem_f1"] = 1e9
    st_f1 = {}

    # 3 (chuẩn bị). kéo θ 0 -> PULL_TH theo cung, đế đứng yên; tay nắm trong khung cửa (u, v) khi cửa đóng
    th0 = LC.joint_q(robot, "fridge_hinge")
    h = _door(robot)
    rel = hp[:2] - h
    c0, s0 = math.cos(-th0), math.sin(-th0)
    rel = np.array([c0 * rel[0] - s0 * rel[1], s0 * rel[0] + c0 * rel[1]])

    def handle_r(th):
        bb = st_f1["bf"]
        c, s = math.cos(th), math.sin(th)
        w = np.array([h[0] + c * rel[0] - s * rel[1], h[1] + s * rel[0] + c * rel[1]])
        return np.array([w[0] - bb[0], w[1] - bb[1], z_h])

    def pull():
        T = 1.4
        n = int(round(T / E.dt))
        for i in range(1, n + 1):
            th = PULL_TH * mj(i / n)
            o = level_offset(E, "right", hw)
            q = E.ik("right", handle_r(th) + [o[0], o[1], 0.0], 0.0, roll=0.0)
            d = E.jd("right", q) if q is not None else {}
            d.update(st_f1["close"](i * E.dt))       # khép nốt tay nắm trong lúc bắt đầu kéo
            yield d

    def pull_arc():
        # cổng 4: tay phải giữ nguyên khớp (kẹp ngang ôm thanh đứng), đế tịnh tiến theo cung tay nắm từ góc hiện tại tới FR5_TH; mở kẹp
        # khi cung còn FR5_REL_LEAD (đuôi đế < 5 mm/s nằm trong lúc kẹp mở). Cửa dừng ở FR5_TH, không trôi.
        b0 = robot.backend.base_pose()
        th_a = LC.joint_q(robot, "fridge_hinge")
        hw_ = np.array(handle_w(th_a)[:2])
        rel_b = hw_ - b0[:2]
        R_h = float(np.linalg.norm(hw_ - h))
        Lp = R_h * abs(FR5_TH - th_a)
        T = max(3.0, math.sqrt(5.774 * Lp / (0.8 * BASE_ACC)), 1.875 * Lp / 0.3)
        n = int(round(T / E.dt))
        rel_g = grip_sched("right", 0, 60, T - FR5_REL_LEAD, T_OPEN)
        E.st["fr5_T"] = T
        k_ret = n - int(round(FR5_RET_LEAD / E.dt))
        ret = None
        for i in range(n):
            t = (i + 0.5) * E.dt
            th = th_a + (FR5_TH - th_a) * mj(t / T)
            thd = (FR5_TH - th_a) * mj_d(t / T) / T
            bd = np.array(handle_w(th)[:2]) - rel_b
            db = (np.array(handle_w(th + 1e-4)[:2]) - np.array(handle_w(th - 1e-4)[:2])) / 2e-4 * thd
            b = robot.backend.base_pose()
            v = db + FR5_K * (bd - b[:2])
            d = {"_base": (float(v[0]), float(v[1]))}
            d.update(st_f1["close"]((i + 1) * E.dt))
            d.update(rel_g((i + 1) * E.dt))
            if i == k_ret:
                # rút tay bắt đầu FR5_RET_LEAD trước cuối cung (kẹp đã mở hết), chạy tiếp ở door_release
                ret = door_retract(rel_g, (i + 1) * E.dt)
                st_f1["ret"] = ret
            if ret is not None:
                d.update(next(ret, {}))
            yield d
        yield {"_base": (0.0, 0.0)}

    def door_retract(rel_g, t0):
        # rút tay theo -x khung robot FR5_BACK (dọc mặt ngoài cửa, qua khỏi mép tự do; tay đang ở phía ngoài cánh cửa, đế đi sang +y
        # trước khi tay qua mép thì cẳng tay quét vào cửa). Đế và tay trái bắt đầu bước sau khi tay đã rút xong (cờ door_out)
        p0 = E.tcp_now("right")
        Tb = dur_for(FR5_BACK)
        yield from cpath(E, "right", [p0, p0 + [-FR5_BACK, 0.0, 0.02]], [0.0, 0.0], [0.0, 0.0], T=Tb, roll_mode=True,
                         grips=[lambda t: rel_g(t + t0), flag_at(E, "door_out", 0.4 * Tb)])

    def handle_w(th):
        c, s = math.cos(th), math.sin(th)
        return (h[0] + c * rel[0] - s * rel[1], h[1] + s * rel[0] + c * rel[1])

    def right_f1():
        # tay phải vào tư thế trước tay nắm, xong 0,3 s trước khi đế dừng; tiếp cận bắt đầu khi đế còn BASE_LEAD (theo tư thế cuối dự đoán)
        yield from jmove(E, "right", q_pre, T_PREP, t_start=max(0.0, dT - T_PREP - 0.3),
                         grips=[grip_sched("right", robot._targets[gk_r], 100, 1.0, 1.0)])
        bf = yield from base_near_end(E, "f1")
        st_f1["bf"] = bf
        hr = np.array([hp[0] - bf[0], hp[1] - bf[1], hp[2]])
        off = level_offset(E, "right", hw)
        tc5 = dict(t_close=FR_T_CLOSE, close_tail=FR_CLOSE_TAIL) if FR5_PULL else dict(close_tail=CLOSE_TAIL)   # kéo: đế bắt đầu khi kẹp còn 1 s khép
        st_f1["close"] = yield from approach(E, "right", E.tcp_now("right"), hr + [off[0], off[1], 0.0], 0.0, 0.0, roll_mode=True,
                                             close_to=0, **tc5)
        log("tủ lạnh: F1, tay phải nắm tay nắm")
        yield from (pull_arc() if FR5_PULL else pull())      # 3. kéo ngay khi kẹp gần khép xong
    # 1-3. đế tới F1 (lift cùng lúc), tay phải vào trước tay nắm, tiếp cận, khép, kéo; tay trái vào tư thế sẵn sàng chậm, chạy qua lúc
    # đế dừng và lúc tay phải bắt đầu tiếp cận (không có lúc cả robot đứng yên)
    ld = E.st.get("lift_delay", 0.0)
    left_pre = E.st.pop("left_pre", None)

    def left_f1():
        t0 = robot.backend.data.time
        if left_pre is not None:
            yield from left_pre()
        el = robot.backend.data.time - t0
        yield from jmove(E, "left", q_ready_l, T_PREP_L, t_start=max(0.0, dT - T_PREP_L + 1.5 - el))
    E.run(drive(E, *F1_, lift_to=LIFT_FRIDGE, t_lift=min(3.0, dT - ld), tag="f1", lift_delay=ld), right_f1(), left_f1())
    b = st_f1["bf"]     # tư thế đế dùng khi tiếp cận tay nắm (dự đoán), giữ cho vòng/đẩy: không nhảy đích khi đổi bước
    # 4. nhả, lùi, vòng qua mép cửa, đẩy mặt trong tới 1,30 (tay trái vẫn chờ)
    th = LC.joint_q(robot, "fridge_hinge")
    wr = lambda w: np.array([w[0] - b[0], w[1] - b[1], w[2]])
    p_back = wr(door_pt(robot, th, rel[0] - 0.035, rel[1], Z_PUSH))
    p_edge_o = wr(door_pt(robot, th, -0.08, 0.56, Z_PUSH))
    p_edge_i = wr(door_pt(robot, th, 0.06, 0.56, Z_PUSH))
    p_in = wr(door_pt(robot, th, U_PUSH, V_PUSH, Z_PUSH))

    # một chuyển động cộng chồng (superpose): mở kẹp, lùi khỏi tay nắm, vòng qua mép cửa, đẩy, rút ra. Đoạn sau bắt đầu khi đoạn trước
    # còn LEAD_DOOR giây, nên không có lúc tay dừng giữa các bước
    p_out = wr(door_pt(robot, PUSH_TH, 0.10, V_PUSH2, z_h))

    def door_moves():
        yield from gripper_to(E, "right", 60, T_OPEN)
        p0 = E.tcp_now("right")
        pts = [p0, p_back, p_edge_o, p_edge_i, p_in]
        st, du, Ta = path_sched(pts, [0.0] * 5, [0.0] * 5, blend=AROUND_BLEND)
        moves = [(t0, Tk, _lin(pts[k + 1] - pts[k], Tk)) for k, (t0, Tk) in enumerate(zip(st, du))]
        t_push = Ta - LEAD_PUSH
        dp = {}

        def push_disp(tau):
            # đẩy theo góc cửa (min-jerk từ góc lúc bắt đầu đẩy tới PUSH_TH); điểm đẩy trượt từ mép cửa (U_PUSH, V_PUSH) vào
            # (U_PUSH2, V_PUSH2) khi cửa mở rộng; z nâng dần về z tay nắm (tránh vùng cấm bệ trạm ở góc lớn); cửa trôi trong lúc vòng:
            # lệch giữa p_in và cửa lúc bắt đầu đẩy xoá dần trong 0,6 s
            if "th_a" not in dp:
                dp["th_a"] = LC.joint_q(robot, "fridge_hinge")
                dp["a0"] = wr(door_pt(robot, dp["th_a"], U_PUSH, V_PUSH, Z_PUSH))
                log(f"tủ lạnh: bắt đầu đẩy, cửa {dp['th_a']:.3f} rad (lúc nhả {th:.3f})")
            th_a = dp["th_a"]
            t2 = th_a + (PUSH_TH - th_a) * mj(tau / T_PUSH)
            z = Z_PUSH + (z_h - Z_PUSH) * mj((t2 - th_a) / 0.3)
            g = mj((t2 - TH_SLIDE[0]) / (TH_SLIDE[1] - TH_SLIDE[0]))
            uu, vv = U_PUSH + (U_PUSH2 - U_PUSH) * g, V_PUSH + (V_PUSH2 - V_PUSH) * g
            off0 = p_in - dp["a0"]
            return wr(door_pt(robot, t2, uu, vv, z)) - dp["a0"] - off0 * mj(tau / 0.6)
        moves.append((t_push, T_PUSH, push_disp))
        p_end = wr(door_pt(robot, PUSH_TH, U_PUSH2, V_PUSH2, z_h))
        T_out = dur_for(float(np.linalg.norm(p_out - p_end)))
        t_out = t_push + T_PUSH - LEAD_DOOR
        moves.append((t_out, T_out, _lin(p_out - p_end, T_out)))
        # khép (về GRIP_AIR, ngón không chạm nhau) khi hàm đã lùi khỏi thanh tay nắm, trước khi vòng qua mép cửa
        # cờ door_out: đế và tay trái bắt đầu bước sau khi tay phải bắt đầu rút khỏi cửa
        yield from superpose(E, "right", p0, moves, grips=[grip_sched("right", 60, GRIP_AIR, GC0 * du[0], 0.4), flag_at(E, "door_out", t_out)])
        snap("2-dang-kep", "lab_fridge")
        log(f"tủ lạnh: cửa mở {LC.joint_q(robot, 'fridge_hinge'):.2f} rad")
    # 5. tay phải rút khỏi cửa rồi về HOME; đế F1 -> F2 (bắt đầu lúc tay phải rút); tay trái tới trước lon 10 cm
    can_w = LC.obj_pos(robot, oid)
    hwc = LC.obj_half_w(oid)
    # cổng 4: lon xáo lệch +y thì servo cổ tay trái sát vách trái tủ: dời F2 theo -y thêm FR5_F2K x độ lệch (tối đa 1 cm)
    dy_can = min(0.01, max(0.0, can_w[1] - L.FRIDGE_OBJ_XY[1])) * FR5_F2K
    F2_ = (F2[0], F2[1] + FR5_F2DY - dy_can) if FR5_PULL else F2
    dirv = np.array([can_w[0] - (F2_[0] + 0.06), can_w[1] - (F2_[1] + spec.SHOULDER_Y)])
    dirv /= np.linalg.norm(dirv)
    zc = can_w[2] + (FR5_ZC if FR5_PULL else 0.012)
    can_lift = FR5_CAN_LIFT if FR5_PULL else CAN_LIFT
    out_x = FR5_OUT_X if FR5_PULL else 0.45     # x (thế giới) điểm ra khỏi tủ và góc tủ khi mang lon
    corner_y = FR5_CORNER_Y if FR5_PULL else -1.08
    pre_can_r = np.array([can_w[0] - 0.10 * dirv[0] - F2_[0], can_w[1] - 0.10 * dirv[1] - F2_[1], zc])
    q_pre_can = E.ik("left", pre_can_r, 0.0, roll=0.0, lift=LIFT_FRIDGE)
    if q_pre_can is None:
        raise ValueError("tủ lạnh: tư thế trước lon ngoài tầm")
    gk_l = spec.joint_name("left", "gripper")
    E.st.update({"rem_f2": 1e9, "rem_f2b": 1e9, "rem_st": 1e9, "can_held": False, "door_out": False, "tuck": False})
    zt = L.TABLE_H + LC.obj_half_h(oid) + 0.012
    out = {}

    def door_release():
        # cổng 4: kẹp mở trong đuôi cung kéo, tay bắt đầu rút trước cuối cung (pull_arc); ở đây chạy nốt đoạn rút rồi về HOME
        yield from st_f1["ret"]
        snap("2-dang-kep", "lab_fridge")
        log(f"tủ lạnh: cửa mở {LC.joint_q(robot, 'fridge_hinge'):.2f} rad")

    def right_door():
        yield from (door_release() if FR5_PULL else door_moves())
        yield from jmove(E, "right", home, T_HOME)

    def base_f2():
        yield from wait_until(E, lambda: E.st["door_out"], timeout=60)
        # tới trước F2 10 cm (tay trái đã ở tư thế trước lon, còn ngoài miệng tủ), rồi tiến thẳng 10 cm: kẹp vào tủ theo +x,
        # không quét ngang qua vách tủ (đi chéo một mạch thì hàm kẹp chạm góc vách trái miệng tủ). Hai chặng cộng chồng vận tốc
        # (chặng sau bắt đầu ở 85% chặng trước): không dừng giữa chặng, góc bo ~1 cm
        # cổng 4 (kéo cửa bằng đế): đế đang sát đường cánh cửa (tay phải còn phía ngoài cửa): sang +y trước rồi mới tiến +x
        b_ = robot.backend.base_pose()
        pre_v = [(b_[0], F2_[1])] if FR5_PULL else []
        yield from drive(E, *F2_, via=pre_v + [(F2_[0] - 0.10, F2_[1])], tag="f2")
        # 7. đế lùi F2 -> F2b khi lon đã nhấc được nửa đường (tay giữ yên trong khung robot: lon trượt thẳng ra)
        yield from wait_until(E, lambda: E.st["can_held"], timeout=60)
        if FR5_PULL:
            # cổng 4: F2b dời theo F2 (FR5_F2DY) và bớt FR5_F2B_DY: sang +y khi tay còn trong tủ thì lòng kẹp quẹt vách trái tủ
            yield from drive(E, F2B[0], F2B[1] + FR5_F2DY + FR5_F2B_DY, tag="f2b", via=[(F2B[0], F2_[1])])
        else:
            yield from drive(E, *F2B, tag="f2b", via=([(F2B[0], F2_[1])] if F2B_BACK_FIRST else None))
        if place_station is not None:
            # P&P: mang lon bằng đế tới chỗ đứng đặt, bắt đầu khi tay trái đã ra khỏi tủ và đang thu về gần thân
            yield from wait_until(E, lambda: E.st.get("tuck", False), timeout=60)
            yield from drive(E, *place_station, tag="st")

    def left_can():
        yield from wait_until(E, lambda: E.st["door_out"], timeout=60)
        yield from jmove(E, "left", q_pre_can, 2.0, t_start=0.6, grips=[grip_sched("left", robot._targets[gk_l], CAN_OPEN, 1.0, 1.0)])
        bf = yield from base_near_end(E, "f2")
        # 6. tiếp cận lon (bắt đầu khi đế còn BASE_LEAD, theo tư thế cuối dự đoán), khép, nhấc 2,5 cm
        cw = LC.obj_pos(robot, oid)
        o = level_offset(E, "left", hwc)
        grab = np.array([cw[0] - bf[0], cw[1] - bf[1], zc]) + [o[0], o[1], 0.0]
        cont = yield from approach(E, "left", E.tcp_now("left"), grab, 0.0, 0.0, roll_mode=True, close_to=0, close_tail=CLOSE_TAIL)
        log(f"tủ lạnh: kẹp lon, kẹp trái {robot.backend.read_joints()['left_gripper']:.1f}, TCP {np.round(robot.tcp_position('left'), 3).tolist()} đích {np.round(grab, 3).tolist()}")
        yield from cpath(E, "left", [grab, grab + [0, 0, can_lift]], [0.0, 0.0], [0.0, 0.0], T=T_LIFT_OBJ, roll_mode=True,
                         grips=[cont, flag_at(E, "can_held", 0.2 * T_LIFT_OBJ)])
        log(f"tủ lạnh: lon sau khi nhấc {np.round(LC.obj_pos(robot, oid), 3).tolist()}")
        if place_station is not None:
            # P&P: ra khỏi tủ qua góc tủ, thu tay về gần thân (đế bắt đầu đi khi tay bắt đầu thu), rồi khi đế còn BASE_LEAD tới chỗ
            # đứng đặt: một đường tới trên chỗ đặt và hạ đặt, kẹp mở từ RELEASE_LEAD s trước cuối đoạn hạ
            b2 = yield from base_near_end(E, "f2b")
            tw = lambda w: np.array([w[0] - b2[0], w[1] - b2[1], w[2]])
            out_pt = tw([out_x, can_w[1] + 0.04, zc + can_lift])
            corner = tw([out_x, corner_y, zc + 0.05])
            pc = E.level_pitch("left", [corner])
            po = [E.tcp_now("left"), out_pt, corner]
            pio = [0.0, E.level_pitch("left", [out_pt]), pc]
            To = path_sched(po, pio, [0.0] * 3)[2]
            # đế bắt đầu đi tới chỗ đứng đặt 0,6 s trước khi tay tới góc tủ (đế đi < 1 mm trong 0,6 s đầu)
            yield from cpath(E, "left", po, pio, [0.0] * 3, roll_mode=True, grips=[flag_at(E, "tuck", max(0.0, To - 0.6))])
            cr2 = E.tcp_now("left")
            tuck = np.array([0.32, cr2[1], cr2[2]])
            tuck = tuck if E.ik("left", tuck, pc, roll=0.0) is not None else cr2
            yield from cpath(E, "left", [cr2, tuck], [pc, pc], [0.0, 0.0], T=1.2, roll_mode=True)
            b3 = yield from base_near_end(E, "st", lead=0.5)
            o = level_offset(E, "left", hwc)
            place = np.array([target_xy[0] - b3[0], target_xy[1] - b3[1], zt + 0.004 + PP_PLACE_DZ]) + [o[0], o[1], 0.0]
            above = place + [0, 0, 0.05]
            pp = E.level_pitch("left", [above, place])
            out["pp"] = pp
            pts = [E.tcp_now("left"), above, place]
            pits = [pc, pp, pp]
            Tp = path_sched(pts, pits, [0.0] * 3)[2]
            out["open_t0"] = RELEASE_LEAD
            yield from cpath(E, "left", pts, pits, [0.0] * 3, roll_mode=True,
                             grips=[grip_sched("left", robot._targets[gk_l], 100, Tp - RELEASE_LEAD, T_OPEN)])
            return
        # 8. ra khỏi tủ, qua góc tủ, tới trên chỗ đặt và hạ đặt: một đường, góc bo; bắt đầu khi đế còn BASE_LEAD tới F2b
        b2 = yield from base_near_end(E, "f2b")
        tw = lambda w: np.array([w[0] - b2[0], w[1] - b2[1], w[2]])
        out_pt = tw([out_x, can_w[1] + 0.04, zc + can_lift])
        corner = tw([out_x, corner_y, zc + 0.05])
        o = level_offset(E, "left", hwc)
        place = tw([target_xy[0], target_xy[1], zt + 0.004]) + [o[0], o[1], 0.0]
        above = place + [0, 0, 0.05]
        pp = E.level_pitch("left", [above, place])
        out["pp"] = pp
        pts = [E.tcp_now("left"), out_pt, corner, above, place]
        pits = [0.0, E.level_pitch("left", [out_pt]), E.level_pitch("left", [corner]), pp, pp]
        Tp = path_sched(pts, pits, [0.0] * 5)[2]
        # nhả bắt đầu RELEASE_LEAD s trước cuối đoạn hạ (lon còn cách chỗ đặt < 1 mm; chỗ đặt cao hơn mặt bàn 4 mm)
        out["open_t0"] = RELEASE_LEAD
        yield from cpath(E, "left", pts, pits, [0.0] * 5, roll_mode=True,
                         grips=[grip_sched("left", robot._targets[gk_l], 100, Tp - RELEASE_LEAD, T_OPEN)])
    E.run(right_door(), left_can(), base_f2())
    snap("2b-lay-tu-lanh", "lab_fridge_in")
    pp = out["pp"]

    def rel_l():
        # nhả (T_OPEN), rút ngược trục dụng cụ 6 cm khi kẹp đã mở 60% (kẹp mở tiếp trong lúc rút), rồi về HOME
        t0 = out.get("open_t0", 0.0)       # kẹp đã bắt đầu mở t0 giây trước (cuối đoạn hạ)
        g0 = robot._targets[gk_l] if t0 == 0.0 else 0.0
        sched = grip_sched("left", g0, 100, -t0, T_OPEN)
        k60 = 0
        while k60 * E.dt < T_OPEN and mj((k60 * E.dt + t0) / T_OPEN) < 0.6:
            k60 += 1
        for i in range(1, k60 + 1):
            yield sched(i * E.dt)
        p = E.tcp_now("left")
        pan = E.q_now("left")[0]
        back = p - np.array([math.cos(pan), math.sin(pan), 0.0]) * RETRACT[0] + [0, 0, RETRACT[1]]
        off = k60 * E.dt
        if place_station is not None and FR5_PULL:
            # cổng 4 (P&P): đặt lon sát vách trước khay; lùi chéo lên thì lòng kẹp quẹt đỉnh vách khay: nâng thẳng trước rồi mới lùi
            up = p + [0.0, 0.0, RETRACT[1]]
            yield from cpath(E, "left", [p, up, back + [0.0, 0.0, 0.01]], [pp, pp, 0.0], [0.0, 0.0, 0.0], roll_mode=True,
                             grips=[lambda t: sched(t + off)])
        else:
            yield from cpath(E, "left", [p, back], [pp, 0.0], [0.0, 0.0], roll_mode=True, grips=[lambda t: sched(t + off)])
        yield from jmove(E, "left", home, T_HOME, grips=[grip_sched("left", 100, GRIP_AIR, 0.0, 1.0)] + lift_up)
    # P&P: chỗ đứng đặt sát bàn, lift thấp (0,27): về HOME thì cẳng tay quét xuống mép bàn; nâng lift về LIFT_WORK cùng lúc
    lift_up = []
    if place_station is not None:
        l0 = robot._targets["lift"]
        lift_up = [lambda t: {"lift": l0 + (L.LIFT_WORK - l0) * mj(t / T_HOME)}]
    E.run(rel_l())


# ---------------- O&C: mở và đóng ngăn kéo, nắp hộp, cửa tủ lạnh (docs/CHOREO-M1-ketqua.md mục 6) ----------------
OC_LIFT = 0.45          # lift khi làm ngăn kéo và nắp (như demo cũ)
OC_PITCH = (-85.0, -80.0, -90.0)   # dụng cụ ngẩng 5 độ (trần gập 70: chĩa thẳng xuống không tới z tay nắm + 3 cm)
DR_IN = 0.05            # đế đứng ở x = WORK_X + 0,05, y = y tay nắm - ARM_Y (tay trái thẳng trước tay nắm)
DR_ABOVE = 0.03         # điểm chờ 3 cm trên tâm thanh tay nắm
DR_OPEN, DR_OPEN_PUSH = 40.0, 30.0   # mở kẹp trước khi hạ: 40% (khe 34 mm) khi kéo, 30% khi đẩy
DR_HW = 0.006           # bán kính thanh tay nắm
DR_PULL = 0.12          # đế lùi 0,12 m theo -x thế giới trong lúc tay giữ đích khớp (TCP cố định trong khung robot)
DR_PUSH_END = 0.004     # đẩy tới x tay nắm lúc đóng + 4 mm
DR_LEAD = 0.35          # bước sau bắt đầu khi đế còn 0,35 s
DR_CLOSE_TAIL = 0.45    # đế bắt đầu kéo/đẩy khi kẹp còn 0,45 s khép (đế 0,3 s đầu đi < 1 mm, kẹp khép nốt trong lúc đó)


OC_VIA_OVERLAP = 0.6    # chặng lùi khỏi bàn -> chặng ngang: chặng sau bắt đầu ở 60% chặng trước (0,85: đế < 5 mm/s ở góc, đứng hình)


def drive_T(E, x, y, via=None, overlap=LEG_OVERLAP, acc=BASE_ACC):
    """Tổng thời lượng của drive(E, x, y, via, overlap) (cùng công thức), để tay vào tư thế chờ xong đúng lúc đế còn DR_LEAD."""
    bx, by, _ = E.r.backend.base_pose()
    pts = [(bx, by)] + [tuple(v) for v in (via or [])] + [(x, y)]
    t0, Tt = 0.0, 0.0
    for (ax, ay), (cx, cy) in zip(pts, pts[1:]):
        d = math.hypot(cx - ax, cy - ay)
        T = max(0.8, math.sqrt(5.774 * d / acc), 1.875 * d / 0.35) if d > 1e-4 else 0.0
        Tt = max(Tt, t0 + T)
        t0 += T * overlap
    return Tt


def _oc_ik(E, p, gy=0.0, lift=OC_LIFT):
    for pd in OC_PITCH:
        q = E.ik("left", p, pd, gy, lift=lift)
        if q is not None:
            return np.asarray(q, float), pd
    return None, None


def _oc_roll_for_fixed(E, p_r, half_w, q_ref, want_dir=(1.0, 0.0), margin=0.002):
    """Chọn góc cuộn (q_ref hoặc lật 180 độ) sao cho vật tâm p_r (khung robot) tựa ngón cố định nằm phía want_dir.
    Trả (TCP kẹp khung robot, góc cuộn). TCP = tâm vật + d * (TCP_DY - FIXED_FACE_Y + half_w + margin), d = hướng ngón cố định."""
    lo, hi = spec.ARM_LIMITS["wrist_roll"]
    for roll in (q_ref[4], q_ref[4] + math.pi, q_ref[4] - math.pi):
        if not lo <= roll <= hi:
            continue
        q = np.array(q_ref, float)
        q[4] = roll
        d = np.asarray(K.fixed_side_dir(q, "left"), float)
        off = d * (spec.TCP_DY - spec.FIXED_FACE_Y + half_w + margin)
        if float(np.dot(off[:2], want_dir)) >= 0:
            return np.asarray(p_r, float) + off, roll
    raise ValueError("không có nghiệm kẹp đúng phía ngón cố định")


def _hold(E, cont, cond):
    """Giữ đích tay (TCP cố định trong khung robot), khép nốt kẹp theo cont, tới khi cond()."""
    k = 0
    while not cond():
        k += 1
        yield cont(k * E.dt)
        if k * E.dt > 60:
            raise TimeoutError("chờ đế quá lâu")


def oc_drawer(E, log, i=1, last=True):
    """Ngăn kéo i: kẹp thanh tay nắm (ngón cố định phía mặt ngăn), đế lùi DR_PULL kéo ngăn ra, nhả, kẹp lại, đế tiến đẩy ngăn vào."""
    r = E.r
    jn = f"drawer_{i}_slide"
    gk = spec.joint_name("left", "gripper")
    jr = spec.joint_name("left", "wrist_roll")
    h = LC.site_world(r, f"drawer_{i}_handle")
    log(f"ngăn kéo {i}: tay nắm {np.round(h, 3).tolist()}")
    bx0, by0 = LC.WORK_X + DR_IN, h[1] - LC.ARM_Y
    hr = np.array([h[0] - bx0, h[1] - by0, h[2]])
    q_pre, pd = _oc_ik(E, hr + [0, 0, DR_ABOVE])
    if q_pre is None:
        raise ValueError("ngăn kéo: điểm chờ ngoài tầm")
    g, roll = _oc_roll_for_fixed(E, hr, DR_HW, q_pre)
    off = g - hr                         # TCP so với tâm thanh tay nắm (lệch về phía ngón cố định)
    keep = r._targets[jr]
    r._targets[jr] = roll
    q_pre2, pd = _oc_ik(E, g + [0, 0, DR_ABOVE])
    r._targets[jr] = keep
    st = {}
    bxy = r.backend.base_pose()
    dT = drive_T(E, bx0, by0)

    def arm():
        # 1. tay vào tư thế chờ trên tay nắm trong lúc đế đi tới (kẹp mở 40%, chạy qua chỗ nối), tiếp cận khi đế còn DR_LEAD
        Tp = max(T_PREP, dT - DR_LEAD)
        g_op = grip_sched("left", GRIP_AIR, DR_OPEN, Tp - 0.5, 1.0)
        yield from jmove(E, "left", q_pre2, Tp, grips=[grip_sched("left", r._targets[gk], GRIP_AIR, 0.0, 1.0), g_op])
        bf = yield from base_near_end(E, "dr0", lead=DR_LEAD)
        g1 = np.array([h[0] - bf[0], h[1] - bf[1], h[2]]) + off
        cont = yield from approach(E, "left", E.tcp_now("left"), g1, pd, 0.0, close_to=0, close_tail=DR_CLOSE_TAIL,
                                   grips=[lambda t: g_op(t + Tp)])
        E.st["dr_pull"] = True
        # 2. kéo: đế lùi DR_PULL, tay giữ đích (khép nốt trong lúc đế bắt đầu đi)
        yield from _hold(E, cont, lambda: E.st.get("rem_dr1", 1e9) <= DR_LEAD)
        # 3. nhả tới 30% và nâng 3 cm khi đế còn DR_LEAD, rồi tới trên tay nắm ở chỗ mới, tiếp cận, khép
        p0 = E.tcp_now("left")
        yield from cpath(E, "left", [p0, p0 + [0, 0, DR_ABOVE]], [pd, pd], [0.0, 0.0], T=0.6,
                         grips=[grip_sched("left", r._targets[gk], DR_OPEN_PUSH, 0.0, 0.5)])
        h2 = LC.site_world(r, f"drawer_{i}_handle")
        st["q_open"] = LC.joint_q(r, jn)
        b = E.st["end_dr1"]
        g2 = np.array([h2[0] - b[0], h2[1] - b[1], h2[2]]) + off
        cont2 = yield from approach(E, "left", E.tcp_now("left"), g2, pd, 0.0, close_to=0, close_tail=DR_CLOSE_TAIL)
        st["push_dx"] = (h[0] + DR_PUSH_END) - h2[0]
        E.st["dr_push"] = True
        yield from _hold(E, cont2, lambda: E.st.get("rem_dr2", 1e9) <= DR_LEAD)
        # 4. nhả, nâng, về HOME (đế lùi khỏi bàn cùng lúc)
        p0 = E.tcp_now("left")
        yield from cpath(E, "left", [p0, p0 + [0, 0, DR_ABOVE]], [pd, pd], [0.0, 0.0], T=0.6,
                         grips=[grip_sched("left", r._targets[gk], 100, 0.0, 0.5)])
        E.st["dr_out"] = True
        if last:
            yield from oc_home(E)

    def base():
        E.st.update({"dr_pull": False, "dr_push": False, "dr_out": False, "rem_dr0": 1e9, "rem_dr1": 1e9, "rem_dr2": 1e9})
        yield from drive(E, bx0, by0, lift_to=OC_LIFT, t_lift=min(3.0, max(1.0, dT * 0.6)), tag="dr0")
        yield from wait_until(E, lambda: E.st["dr_pull"], timeout=60)
        b = E.st["end_dr0"]
        yield from drive(E, b[0] - DR_PULL, b[1], tag="dr1")
        yield from wait_until(E, lambda: E.st["dr_push"], timeout=60)
        b = E.st["end_dr1"]
        yield from drive(E, b[0] + st["push_dx"], b[1], tag="dr2")
        if last:
            yield from wait_until(E, lambda: E.st["dr_out"], timeout=60)
            b = E.st["end_dr2"]
            yield from drive(E, LC.WORK_X - 0.05, b[1], tag="dr3")
    E.run(arm(), base())
    return jn, st["q_open"]


def oc_home(E):
    home = np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS])
    gk = spec.joint_name("left", "gripper")
    yield from jmove(E, "left", home, T_HOME, grips=[grip_sched("left", E.r._targets[gk], GRIP_AIR, 0.0, 1.0)])


# ---------------- O&C ngăn kéo có đặt vật (docs/CHOREO-M1-ketqua.md mục 8.8) ----------------
# Trình tự theo bản web "Opening & closing hai tay" (open-close.ts, hàm drawer): tay trái nắm tay nắm ngăn trên, kéo ngăn ra bằng tay
# (đế đứng yên) và giữ; cùng lúc tay phải gắp vật thấp nhất gần tủ trên bàn, mang lên trên miệng ngăn, xoay kẹp 90 độ, đặt vào lòng ngăn;
# tay trái đẩy ngăn vào trong lúc tay phải thu về. Chuyển động theo kỹ thuật SDK (tiếp cận hai đoạn, khép 0,6 s, đoạn cộng chồng).
DP_OBJ = _os.environ.get("M1_OC_DP_OBJ", "obj_blue_block")   # vật cao 35 mm (lòng ngăn 52 mm), gần tủ nhất trong các vật <= 4 cm
DP_STAND = (float(_os.environ.get("M1_OC_DP_BX", "0.34")), float(_os.environ.get("M1_OC_DP_BY", "0.74")))   # đế (thế giới), lift 0,45
DP_PICK_BACK = float(_os.environ.get("M1_OC_DP_PICK_BACK", "0.04"))   # đế đứng lùi chừng này khi tay phải gắp vật, rồi tiến tới DP_STAND
DP_PULL = float(_os.environ.get("M1_OC_DP_PULL", "0.11"))     # tay trái kéo 10 cm theo -x (đế đứng yên)
DP_T_PULL = float(_os.environ.get("M1_OC_DP_T_PULL", "2.5"))  # thời lượng kéo / đẩy (web 2,5 s)
DP_SLOT_IN = float(_os.environ.get("M1_OC_DP_SLOT_IN", "0.058"))   # tâm vật cách mặt trong vách trước (web 0,045: lòng bàn tay phải quệt vách trước)
DP_SLOT_SIDE = 0.065      # tâm vật cách mép -y của đáy ngăn (web)
DP_OUT_DY = float(_os.environ.get("M1_OC_DP_OUT_DY", "0.12"))   # rút tay phải ra phía -y khỏi bề ngang vách trước trước khi đẩy ngăn
DP_HOLD = _os.environ.get("M1_OC_DP_HOLD", "1") == "1"   # 1: giữ tay nắm suốt như web (trượt cổng 4), 0: nhả sau khi kéo, đẩy bằng đầu ngón
DP_REL_UP = 0.025         # nhả sau khi kéo: nâng 2,5 cm (đầu ngón qua đỉnh thanh 0,871)
DP_BACK2 = float(_os.environ.get("M1_OC_DP_BACK2", "0.08"))   # đế lùi chừng này sau khi đặt vật để tay trái với tới sau thanh
DP_BEHIND = 0.028         # TCP lùi sau tâm thanh (phía robot) khi hạ để đẩy
DP_FINGER_FACE = 0.0236   # TCP tới mặt trước khối ngón khép (ngón cố định dày 17,7 mm, TCP ở mặt trong) + bán kính thanh 6 mm
DP_PUSH_END = float(_os.environ.get("M1_OC_DP_PUSH_END", "0.006"))   # đẩy TCP tới x tay nắm lúc đóng + chừng này (1,2 cm: lòng bàn tay trái chạm nóc tủ)
DP_HANDLE_DZ = float(_os.environ.get("M1_OC_DP_HANDLE_DZ", "0.0"))   # TCP tay trái cao hơn tâm thanh tay nắm: lòng bàn tay cách mép vách trước thêm 4 mm
DP_CARRY_Z = float(_os.environ.get("M1_OC_DP_CARRY", "0.915"))   # TCP khi mang vật: đáy vật cao hơn mép vách bên ngăn (0,8755) ~1,2 cm
DP_SIDE_DY = float(_os.environ.get("M1_OC_DP_SIDE_DY", "0.08"))   # vào chỗ đặt từ phía -y (qua vách bên thấp), không qua vách trước (0,8955: TCP mang phải ~0,925, ngoài tầm)
DP_DROP = 0.004           # đáy vật cao hơn đáy ngăn khi nhả
DP_ALT = _os.environ.get("M1_OC_DP_ALT", "0") == "1"   # gắp theo cặp mặt 5 cm (trục khép theo y): hai ngón dọc y ngay từ bàn, không xoay khi mang
DP_ROT = float(_os.environ.get("M1_OC_DP_ROT", "0" if DP_ALT else str(math.pi / 2)))   # xoay trục khép khi mang (web: π/2, hai ngón dọc y, lòng bàn tay không chạm mép tủ)
DP_UP = 0.06              # rút thẳng lên sau khi nhả
DP_REL_FRAC = float(_os.environ.get("M1_OC_DP_REL_FRAC", "0.3"))   # rút khi kẹp đã mở chừng này quãng (0,6: tay đứng ~0,3 s, đứng hình)
DP_OPEN_ADD = float(_os.environ.get("M1_OC_DP_OPEN_ADD", "14"))   # nhả: mở thêm chừng này % so với độ kẹp đang giữ
DP_PUSH_LEAD = float(_os.environ.get("M1_OC_DP_PUSH_LEAD", "0.3"))   # đẩy bắt đầu khi đoạn rút của tay phải còn chừng này giây
DP_PLACE_LEAD = float(_os.environ.get("M1_OC_DP_PLACE_LEAD", "0.35"))  # đường đặt bắt đầu khi kéo còn chừng này giây
L_CANDS = tuple(float(v) for v in _os.environ.get("M1_OC_DP_LCANDS", "-80,-85,-90,-75,-70").split(","))   # -80 trước: lòng bàn tay lùi về phía robot, xa mép nóc tủ
# đặt trong ngăn sát trần tầm với (TCP 0,87-0,92 m ở x 0,34-0,36 khung robot): chĩa xuống tới -70 thì chỉ vừa đủ, tay võng ~1 cm
# và vật quệt mép vách bên; cho phép ngẩng thêm tới -60 (dụng cụ nghiêng về phía tủ)
DP_PLACE_G_CANDS = tuple(float(v) for v in _os.environ.get("M1_OC_DP_GCANDS", "-85,-80,-75,-70,-65,-60").split(","))   # điểm đặt: chĩa xuống nhất có thể (lòng bàn tay nghiêng về phía robot thì quệt vách trước)
DP_PLACE_CANDS = tuple(float(v) for v in _os.environ.get("M1_OC_DP_PCANDS", "-70,-65,-75,-60").split(","))


def drawer_floor(r, i=1):
    """Đáy lòng ngăn i (geom `drawer_<i>_g1`): tâm (thế giới), nửa kích thước, z mặt đáy."""
    b = r.backend
    import mujoco
    gid = mujoco.mj_name2id(b.model, mujoco.mjtObj.mjOBJ_GEOM, f"drawer_{i}_g1")
    c = np.array(b.data.geom_xpos[gid], float)
    h = np.array(b.model.geom_size[gid], float)
    return c, h, float(c[2] + h[2])


def obj_in_drawer(r, oid, i=1, settle=0.02):
    """Vật nằm trong lòng ngăn: tâm trong hộp đáy theo x, y; đáy vật trên đáy ngăn ±1,5 cm; đứng yên (như web objInDrawer)."""
    c, h, floor = drawer_floor(r, i)
    p = LC.obj_pos(r, oid)
    ok = abs(p[0] - c[0]) <= h[0] and abs(p[1] - c[1]) <= h[1] and abs(p[2] - LC.obj_half_h(oid) - floor) <= 0.015 \
        and LC.obj_speed(r, oid) <= settle
    return {"ok": bool(ok), "pos": [round(float(v), 4) for v in p]}


def _dp_place(E, oid, xy, gy, z_tcp, op, base):
    """Một đường không dừng: (độ cao mang) -> trên chỗ đặt trong ngăn -> hạ đặt; kẹp bắt đầu mở RELEASE_LEAD s trước cuối đoạn hạ.
    Sát đỉnh tầm (TCP 0,925 m) góc ngẩng ở điểm trên và điểm đặt khác nhau 5-10 độ: mỗi điểm một góc, cpath nội suy góc theo đoạn
    (place_down đòi một góc chung cho cả hai điểm, không có nghiệm ở đây)."""
    r = E.r
    hw = _half_y(oid) if DP_ALT else LC.obj_half_w(oid)
    for gy_t in (gy, gy - math.pi, gy + math.pi / 2, gy - math.pi / 2):
        c = E.w2r([xy[0], xy[1], z_tcp], base)
        t = r.grasp_point("right", c, hw, grip_yaw=gy_t)
        g = np.asarray(c if t is None else t, float)
        above = np.array([g[0], g[1], DP_CARRY_Z])
        side = above - [0.0, DP_SIDE_DY, 0.0]
        try:
            ps = E.pick_pitch("right", [side], gy_t, DP_PLACE_CANDS)
            pa = E.pick_pitch("right", [above], gy_t, DP_PLACE_CANDS)
            pg = E.pick_pitch("right", [g], gy_t, DP_PLACE_G_CANDS)
        except ValueError:
            continue
        break
    else:
        raise ValueError(f"tay phải: chỗ đặt trong ngăn {list(np.round(xy, 3))} ngoài tầm (z {z_tcp:.3f}, gy {gy:.2f}, đế {np.round(base, 3).tolist()}, "
                         f"lift {r._targets['lift']:.3f}, g {np.round(g, 3).tolist()})")
    cur = E.tcp_now("right")
    pts = [cur, side, above, g]
    pit, yws = [_pitch_of(E, "right"), ps, pa, pg], [_gy_of(E, "right"), gy_t, gy_t, gy_t]
    Tp = path_sched(pts, pit, yws, blend=PLACE_BLEND)[2]
    E.st["open_t0_right"] = RELEASE_LEAD
    yield from cpath(E, "right", pts, pit, yws, blend=PLACE_BLEND, grips=[grip_sched("right", 0.0, op, Tp - RELEASE_LEAD, T_OPEN)])
    E.st["gy_right"] = gy_t
    return g, pg, gy_t


def oc_drawer_place(E, log, i=1, last=True):
    """Ngăn kéo i có đặt vật: tay trái kéo ngăn bằng tay và giữ, tay phải gắp DP_OBJ trên bàn đặt vào lòng ngăn, tay trái đẩy ngăn vào."""
    r = E.r
    jn = f"drawer_{i}_slide"
    gk = spec.joint_name("left", "gripper")
    gkr = spec.joint_name("right", "gripper")
    jr = spec.joint_name("left", "wrist_roll")
    oid = DP_OBJ
    h = LC.site_world(r, f"drawer_{i}_handle")
    bx0, by0 = DP_STAND
    log(f"ngăn kéo {i} có đặt vật {oid}: tay nắm {np.round(h, 3).tolist()}, đế {DP_STAND}")
    hr = np.array([h[0] - bx0, h[1] - by0, h[2]])
    q_pre, pd = _oc_ik(E, hr + [0, 0, DR_ABOVE])
    if q_pre is None:
        raise ValueError("ngăn kéo: điểm chờ ngoài tầm")
    g, roll = _oc_roll_for_fixed(E, hr, DR_HW, q_pre)
    g = g + [0.0, 0.0, DP_HANDLE_DZ]
    off = g - hr
    keep = r._targets[jr]
    r._targets[jr] = roll
    try:
        # tiếp cận một góc chung (điểm chờ, tay nắm); điểm kéo ra (TCP lùi 10 cm, sát trần gập) có góc riêng, cpath nội suy góc
        # ưu tiên một góc cho cả tay nắm lúc đóng, lúc kéo ra và điểm chờ (đổi góc khi đang kẹp làm lòng bàn tay quệt mép vách
        # trước ngăn); điểm chờ hạ dần 3 -> 1,5 cm nếu cần. Không có thì: góc chung điểm chờ/tay nắm, điểm kéo ra góc riêng.
        pd = pp = None
        for dab in (DR_ABOVE, 0.025, 0.02, 0.015):
            try:
                pd = pp = E.pick_pitch("left", [g + [0, 0, dab], g, g - [DP_PULL, 0, 0]], 0.0, L_CANDS, lift=OC_LIFT)
                break
            except ValueError:
                continue
        if pd is None:
            dab = DR_ABOVE
            pd = E.pick_pitch("left", [g + [0, 0, dab], g], 0.0, L_CANDS, lift=OC_LIFT)
            pp = E.pick_pitch("left", [g - [DP_PULL, 0, 0]], 0.0, (pd,) + L_CANDS, lift=OC_LIFT)
        q_pre2 = E.ik("left", g + [0, 0, dab], pd, 0.0, lift=OC_LIFT)
    finally:
        r._targets[jr] = keep
    po = LC.obj_pos(r, oid)
    bxp = bx0 - DP_PICK_BACK      # chỗ gắp: đế lùi DP_PICK_BACK (vật ở x 0,27 khung robot quá sát thân, tay phải không hạ thẳng được)
    ready_r = E.ik_down("right", [po[0] - bxp, po[1] - by0, LC.grasp_z(oid) + ABOVE + 0.02], lift=OC_LIFT)
    if ready_r is None:
        raise ValueError("ngăn kéo: tư thế chờ tay phải ngoài tầm")
    st = {}
    dT = drive_T(E, bxp, by0)
    E.st.update({"rem_dp1": 1e9, "rem_dp2": 1e9, "rem_dp3": 1e9, "end_dp3": None, "dp_back": False, "dp_push": False, "dp_grasp": False, "dp_pulled_soon": False, "dp_placed": False, "dp_out": False, "rem_dp0": 1e9})

    def left():
        Tp = max(T_PREP, dT - DR_LEAD)
        g_op = grip_sched("left", GRIP_AIR, DR_OPEN, Tp - 0.5, 1.0)
        yield from jmove(E, "left", q_pre2, Tp, grips=[grip_sched("left", r._targets[gk], GRIP_AIR, 0.0, 1.0), g_op])
        bf = yield from base_near_end(E, "dp1", lead=DR_LEAD)
        g1 = np.array([h[0] - bf[0], h[1] - bf[1], h[2]]) + off
        cont = yield from approach(E, "left", E.tcp_now("left"), g1, pd, 0.0, close_to=0, close_tail=DR_CLOSE_TAIL,
                                   grips=[lambda t: g_op(t + Tp)])
        # kéo bằng tay: TCP lùi DP_PULL theo -x (min-jerk), kẹp khép nốt lúc đầu đoạn; báo tay phải khi còn DP_PLACE_LEAD s
        T = DP_T_PULL
        soon = lambda t: (E.st.__setitem__("dp_pulled_soon", True) or {}) if t >= T - DP_PLACE_LEAD else {}
        yield from cpath(E, "left", [g1, g1 - [DP_PULL, 0, 0]], [pd, pp], [0.0, 0.0], T=T, grips=[cont, soon])
        E.st["dp_pulled_soon"] = True
        st["q_open"] = LC.joint_q(r, jn)
        if DP_HOLD:
            # bản giữ tay nắm suốt (web): đẩy vào khi kẹp còn khép. Lòng bàn tay chạm vách trước lúc dừng kéo và nóc tủ lúc ngăn
            # đóng (cổng 4), nên mặc định là bản nhả-đẩy bên dưới
            yield from wait_until(E, lambda: E.st["dp_placed"], timeout=60)
            p0 = E.tcp_now("left")
            g2 = np.array([g1[0] + DP_PUSH_END, g1[1], g1[2]])
            st["q_open"] = max(st["q_open"], LC.joint_q(r, jn))
            up = lambda tau: np.array([0.0, 0.0, dab * mj(tau / 0.6)])
            yield from cpath(E, "left", [p0, g2], [pp, pd], [0.0, 0.0], T=T,
                             grips=[grip_sched("left", 0.0, DR_OPEN, T - DR_LEAD, 0.4)], then=(up, 0.6, DR_LEAD))
        else:
            # nhả ngay khi kéo xong (ngăn đứng yên nhờ ma sát): mở 40% tại chỗ (TCP ở x ~0,29 khung robot, sát thân: không nâng được).
            # Chờ tay phải ra khỏi miệng ngăn; đế lùi DP_BACK2 trong lúc tay trái giữ đích thế giới (tay ra xa thân), rồi nâng qua
            # đỉnh thanh, sang phía robot của thanh, hạ; đế tiến đẩy ngăn vào, tay giữ đích khớp (như mục 8.3 bước 5). Lòng bàn tay
            # ở phía robot của thanh: không chạm vách trước, nóc tủ
            p0 = E.tcp_now("left")
            yield from cpath(E, "left", [p0, p0 + [0, 0, 0.001]], [pp, pp], [0.0, 0.0], T=0.6,
                             grips=[grip_sched("left", 0.0, DR_OPEN, 0.0, 0.5)])
            st["q_open"] = max(st["q_open"], LC.joint_q(r, jn))
            yield from wait_until(E, lambda: E.st["dp_placed"], timeout=60)
            bb = r.backend.base_pose()
            pw = E.tcp_now("left") + [bb[0], bb[1], 0.0]          # TCP thế giới lúc nhả
            hb = LC.site_world(r, f"drawer_{i}_handle")
            b2 = (bx0 - DP_BACK2, by0)
            E.st["dp_back"] = True
            k = 0
            while E.st.get("rem_dp2", 1e9) > 0.0 or k < 3:
                k += 1
                b = r.backend.base_pose()
                q = E.ik("left", [pw[0] - b[0], pw[1] - b[1], pw[2]], pp, 0.0)
                yield (E.jd("left", q) if q is not None else {})
                if k * E.dt > 30:
                    raise TimeoutError("chờ đế lùi quá lâu")
            c0 = np.array([pw[0] - b2[0], pw[1] - b2[1], pw[2]])
            up = c0 + [0.0, 0.0, DP_REL_UP]
            lo = np.array([hb[0] - DP_BEHIND - b2[0], c0[1], hb[2]])
            hi = lo + [0.0, 0.0, DP_REL_UP]
            PP = (-80.0, -85.0, -90.0, -75.0, -95.0, -70.0)
            pu, ph = E.pick_pitch("left", [up], 0.0, (pp,) + PP), E.pick_pitch("left", [hi], 0.0, PP)
            pl = E.pick_pitch("left", [lo, lo + [0, 0, DR_ABOVE]], 0.0, (ph,) + PP)
            yield from cpath(E, "left", [E.tcp_now("left"), up, hi, lo], [pp, pu, ph, pl], [0.0, 0.0, 0.0, 0.0], blend=0.4,
                             grips=[grip_sched("left", DR_OPEN, GRIP_AIR, 0.6, 0.6)])
            E.st["dp_bx_end"] = b2[0] + (h[0] - hb[0]) + DP_BEHIND - DP_FINGER_FACE + DP_PUSH_END
            E.st["dp_push"] = True
            yield from wait_until(E, lambda: E.st.get("rem_dp3", 1e9) <= DR_LEAD, timeout=60)
            p0 = E.tcp_now("left")
            yield from cpath(E, "left", [p0, p0 + [0, 0, DR_ABOVE]], [pl, pl], [0.0, 0.0], T=0.6)
        E.st["dp_out"] = True
        if last:
            yield from oc_home(E)

    def right():
        yield from jmove(E, "right", ready_r, max(T_PREP, dT - DR_LEAD - 0.3),
                         grips=[grip_sched("right", r._targets[gkr], GRIP_AIR, 0.0, 1.0)])
        bf = yield from base_near_end(E, "dp0", lead=DR_LEAD)
        E.st["gy_right"] = _gy_of(E, "right")
        gp, above, pdr, gy = yield from pick_down(E, "right", oid, alt=DP_ALT, close_tail=CLOSE_TAIL, base=bf)
        E.st["gy_right"] = gy
        cont = E.st.pop("close_cont_right")
        E.st["dp_grasp"] = True       # đế tiến tới chỗ đứng ngăn kéo trong lúc tay phải nhấc vật
        # nhấc thẳng lên độ cao mang (kẹp khép nốt), chờ ngăn ra gần hết, rồi một đường: sang trên chỗ đặt (xoay DP_ROT) -> hạ đặt
        lift_pt = np.array([gp[0], gp[1], DP_CARRY_Z])
        yield from cpath(E, "right", [gp, lift_pt], [pdr, pdr], [gy, gy], grips=[cont])
        yield from wait_until(E, lambda: E.st["dp_pulled_soon"], timeout=60)
        dz = float(r.tcp_position("right")[2] - LC.obj_pos(r, oid)[2])     # lệch TCP - tâm vật đo được (vật có thể trượt trong kẹp)
        b = r.backend.base_pose()

        def slot():
            c, hh, floor = drawer_floor(r, i)
            # đọc ngăn lúc kéo xong (tay trái võng: ngăn ra 0,08-0,10 m tuỳ seed, không đoán theo quãng đặt)
            x_front = c[0] - hh[0]
            st["floor"] = floor
            return (x_front + DP_SLOT_IN, c[1] - hh[1] + DP_SLOT_SIDE)
        xy = slot()
        z_tcp = st["floor"] + LC.obj_half_h(oid) + DP_DROP + dz
        drop_h = z_tcp - LC.grasp_z(oid)
        # nhả hẹp: mở tới độ kẹp đang giữ + DP_OPEN_ADD (mở hết thì hàm động quệt vách ngăn)
        op = min(100.0, float(r.backend.read_joints()[gkr]) + DP_OPEN_ADD)
        g_pl, pd2, gy2 = yield from _dp_place(E, oid, xy, gy + DP_ROT, z_tcp, op, (b[0], b[1]))
        # kẹp đã bắt đầu mở RELEASE_LEAD s trước cuối đoạn hạ; mở tới 60% quãng rồi rút thẳng lên DP_UP (mở nốt trong lúc rút)
        opn = yield from release_open(E, "right", op, frac=DP_REL_FRAC)
        # rút: lên độ cao mang rồi ra phía -y (ngược đường vào), khỏi bề ngang vách trước ngăn trước khi tay trái đẩy
        up_pt = np.array([g_pl[0], g_pl[1], DP_CARRY_Z])
        out_pt = up_pt - [0.0, DP_OUT_DY, 0.0]
        po_ = E.pick_pitch("right", [out_pt], gy2, DP_PLACE_CANDS)
        pts_r, pit_r, yw_r = [g_pl, up_pt, out_pt], [pd2, pd2, po_], [gy2, gy2, gy2]
        Tu = path_sched(pts_r, pit_r, yw_r)[2]
        flag = lambda t: (E.st.__setitem__("dp_placed", True) or {}) if t >= Tu - DP_PUSH_LEAD else {}
        yield from cpath(E, "right", pts_r, pit_r, yw_r, grips=[opn, flag])
        E.st["dp_placed"] = True
        home = np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS])
        yield from jmove(E, "right", home, T_HOME, grips=[grip_sched("right", r._targets[gkr], GRIP_AIR, 0.0, 1.0)])

    def base():
        yield from drive(E, bxp, by0, lift_to=OC_LIFT, t_lift=min(3.0, max(1.0, dT * 0.6)), tag="dp0")
        yield from wait_until(E, lambda: E.st["dp_grasp"], timeout=60)
        yield from drive(E, bx0, by0, tag="dp1")
        if not DP_HOLD:
            yield from wait_until(E, lambda: E.st["dp_back"], timeout=90)
            b = E.st["end_dp1"]
            yield from drive(E, b[0] - DP_BACK2, b[1], tag="dp2")
            yield from wait_until(E, lambda: E.st["dp_push"], timeout=90)
            b = E.st["end_dp2"]
            yield from drive(E, E.st["dp_bx_end"], b[1], tag="dp3")
        if last:
            yield from wait_until(E, lambda: E.st["dp_out"], timeout=90)
            b = E.st.get("end_dp3") or E.st["end_dp1"]
            yield from drive(E, LC.WORK_X - 0.05, b[1], tag="dp3")
    E.run(left(), right(), base())
    return jn, st.get("q_open", 0.0)


LID_IN, LID_DY = 0.04, 0.07   # đế: x = WORK_X + 0,04; y = y núm - (ARM_Y - 0,07) (núm lệch vào trong trục pan tay trái 7 cm)
LID_ABOVE = 0.04        # điểm chờ 4 cm trên núm
LID_UP = float(_os.environ.get("M1_OC_LID_UP", "0.01"))   # nhả ở góc mở: nâng 1 cm rồi kẹp lại
LID_HW = 0.012          # bán kính núm cầu
LID_WANT_Y = float(_os.environ.get("M1_OC_LID_WANT_Y", "0"))   # -1: ngón cố định phía -y của núm (đẩy núm theo +y khi mở)
LID_HOOK = float(_os.environ.get("M1_OC_LID_HOOK", "0"))        # >0: móc cổ núm, TCP lùi dọc pháp tuyến nắp từ tâm núm cầu (m)
LID_HOOK_GAP = float(_os.environ.get("M1_OC_LID_HOOK_GAP", "0.015"))   # khe ngón khi móc (m)
LID_GY = float(_os.environ.get("M1_OC_LID_GY", "1.5708"))      # grip_yaw khi làm nắp (pi/2: trục khép theo y, lồng núm; 0: trục khép song song bản lề, kẹp chặt)
LID_GRIP = float(_os.environ.get("M1_OC_LID_GRIP", "30"))   # đích khép trên núm (%)
LID_OPEN_TH = float(_os.environ.get("M1_OC_LID_OPEN_TH", "1.75"))     # góc đích khi mở
LID_CLOSE_TH = float(_os.environ.get("M1_OC_LID_CLOSE_TH", "0.12"))   # góc đích khi đóng (nắp tự rơi về 0 sau khi nhả)
LID_SW = float(_os.environ.get("M1_OC_LID_SW", "0.7"))       # >0: từ góc này, TCP nâng dần LID_DZ và lùi LID_DYP (chuyển từ lồng núm sang đẩy)
LID_DZ = float(_os.environ.get("M1_OC_LID_DZ", "0"))
LID_DYP = float(_os.environ.get("M1_OC_LID_DYP", "-0.0425"))
LID_DZC = float(_os.environ.get("M1_OC_LID_DZC", "0"))    # khi đóng: TCP hạ LID_DZC, lệch LID_DYC theo -y ở góc > LID_SW (núm cao hơn tầm với)
LID_DYC = float(_os.environ.get("M1_OC_LID_DYC", "0"))
LID_UPC = float(_os.environ.get("M1_OC_LID_UPC", "0.02"))   # nâng sau khi nhả ở cuối cung đóng khi còn bước sau (nâng 4 cm rồi sang tủ lạnh: rung đế, jerk > 5); bài riêng nắp nâng 4 cm (về HOME từ 2 cm kéo nắp lên)
LID_RELX = float(_os.environ.get("M1_OC_LID_RELX", "0.322"))  # >0: đế x = x núm - LID_RELX
T_LID = float(_os.environ.get("M1_OC_T_LID", "3.0"))             # thời lượng tối thiểu của cung (kéo dài nếu đế cần, gia tốc <= BASE_ACC)
ARC_LEAD = 0.35         # bắt đầu nhả khi cung còn 0,35 s
LID_LEAD_O = float(_os.environ.get("M1_OC_LID_LEAD_O", "0.35"))   # nắp: nhả khi cung mở còn chừng này giây
LID_LEAD_C = float(_os.environ.get("M1_OC_LID_LEAD_C", "0.7"))   # nắp: nhả khi cung đóng còn chừng này giây
LID_TRIES, LID_OK_TH = 2, 1.25   # nắp mở chưa tới 1,25 rad (núm trượt) thì kẹp lại và mở lại, tối đa 2 lần


def _lid_geom(r):
    import mujoco
    b = r.backend
    hinge = np.array(b.data.xpos[mujoco.mj_name2id(b.model, mujoco.mjtObj.mjOBJ_BODY, "lid_box_lid")], float)
    th0 = LC.joint_q(r, "lid_hinge")
    k = LC.site_world(r, "lid_box_lid_knob") - hinge
    c, s_ = math.cos(-th0), math.sin(-th0)          # đưa về nắp đóng (trục bản lề -x: y' = y cos + z sin, z' = -y sin + z cos)
    k0 = np.array([k[0], k[1] * c + k[2] * s_, -k[1] * s_ + k[2] * c])

    def knob(th):
        return hinge + [k0[0], k0[1] * math.cos(th) + k0[2] * math.sin(th), -k0[1] * math.sin(th) + k0[2] * math.cos(th)]
    return knob


def _arm_only_pitch(E, pts_r, lift=OC_LIFT, first=None, gy=0.0):
    """Góc ngẩng chung để tay một mình đi hết các điểm pts_r (khung robot), None nếu không có. first: thử trước (góc lúc kẹp:
    đổi góc ngay sau khi khép làm núm trượt khỏi kẹp)."""
    for c in ([first] if first is not None else []) + [-85.0, -80.0, -90.0, -75.0, -70.0]:
        if all(E.ik("left", p, c, gy, lift=lift) is not None for p in pts_r):
            return c
    return None


def track_arm(E, path, T, off, base_xy, pfix, grips=(), gy=0.0):
    """Đế đứng yên ở base_xy; tay đưa TCP theo path(mj(t/T)) + off (khung robot), một góc ngẩng pfix."""
    n = max(2, int(round(T / E.dt)))
    for i in range(1, n + 1):
        t = i * E.dt
        w = np.asarray(path(mj(t / T)), float)
        pt = [w[0] - base_xy[0] + off[0], w[1] - base_xy[1] + off[1], w[2] + off[2]]
        q = E.ik("left", pt, pfix, gy)
        if q is None:
            E.ik_miss += 1
            q = _oc_ik(E, pt, gy, lift=None)[0]
        out = E.jd("left", q) if q is not None else {}
        for g in grips:
            out.update(g(t))
        yield out


def track(E, path, T_min, off, pd, grips=(), tag=None):
    """Tay giữ tay nắm trên đường path(s) (thế giới, s 0..1 theo min-jerk), đế mang phần x, y của đường (TCP cố định x, y trong khung
    robot), tay mang phần z. T kéo dài để gia tốc đế <= BASE_ACC. off: TCP so với tay nắm (khung robot)."""
    m = 400
    xy = np.array([np.asarray(path(mj(i / m)), float)[:2] for i in range(m + 1)])     # theo thời gian chuẩn hoá u = t / T
    a_u = float(np.max(np.linalg.norm(np.diff(xy, n=2, axis=0), axis=1))) * m * m         # max |d²xy/du²|
    j_u = float(np.max(np.linalg.norm(np.diff(xy, n=3, axis=0), axis=1))) * m ** 3
    T = max(T_min, math.sqrt(a_u / (0.8 * BASE_ACC)), (j_u / 0.25) ** (1 / 3))          # gia tốc đế <= 0,8 BASE_ACC, jerk đế <= 0,25 m/s³
    n = max(2, int(round(T / E.dt)))
    pr = E.tcp_now("left") - off                    # tay nắm trong khung robot lúc bắt đầu
    # một góc ngẩng cho cả cung (đổi góc giữa chừng làm khớp nhảy): góc đầu tiên có nghiệm ở mọi điểm
    zs = [float(np.asarray(path(i / 20), float)[2]) for i in range(21)]
    pdc = [pd] + [c for c in (-85.0, -80.0, -90.0, -75.0, -70.0) if c != pd]
    pfix = next((c for c in pdc if all(E.ik("left", [pr[0] + off[0], pr[1] + off[1], z + off[2]], c, 0.0) is not None for z in zs)), None)
    for i in range(1, n + 1):
        t = i * E.dt
        s = mj(t / T)
        sv = mj_d((t - 0.5 * E.dt) / T) / T
        w = np.asarray(path(s), float)
        # vận tốc đế = đạo hàm phần x, y của đường (sai phân trung tâm theo s)
        ds = 1e-4
        dw = (np.asarray(path(min(1.0, s + ds)), float) - np.asarray(path(max(0.0, s - ds)), float)) / (min(1.0, s + ds) - max(0.0, s - ds))
        vb = dw[:2] * sv
        pt = [pr[0] + off[0], pr[1] + off[1], w[2] + off[2]]
        q = E.ik("left", pt, pfix, 0.0) if pfix is not None else _oc_ik(E, pt, lift=None)[0]
        out = E.jd("left", q) if q is not None else {}
        if q is None:
            E.ik_miss += 1
        out["_base"] = (float(vb[0]), float(vb[1]))
        for g in grips:
            out.update(g(t))
        if tag:
            E.st[f"rem_{tag}"] = T - t
        yield out
    yield {"_base": (0.0, 0.0)}


def _dbg(E, what):
    if _os.environ.get("M1_OC_DBG") == "1":
        E.log(f"DBG {what} t={float(E.r.backend.data.time):.2f} lid={LC.joint_q(E.r, 'lid_hinge'):.3f}")


def oc_lid(E, log, last=True):
    """Nắp hộp: kẹp núm cầu (dụng cụ ngẩng 5 độ), mở theo cung bản lề tới 1,70 rad (đế đi theo y, tay theo z), nhả ở cuối cung,
    nâng 1 cm, kẹp lại, đóng theo cung tới 0,12 rad, nhả (nắp rơi về 0)."""
    r = E.r
    jn = "lid_hinge"
    gk = spec.joint_name("left", "gripper")
    jr = spec.joint_name("left", "wrist_roll")
    knob = _lid_geom(r)
    p = knob(LC.joint_q(r, jn))
    log(f"nắp hộp: núm {np.round(p, 3).tolist()}")
    bx0, by0 = LC.WORK_X + LID_IN, p[1] - (LC.ARM_Y - LID_DY)
    if LID_RELX > 0:            # đế theo x của núm: núm luôn ở cùng chỗ trong khung robot (hộp lệch ±1 cm)
        bx0 = p[0] - LID_RELX
    pr = np.array([p[0] - bx0, p[1] - by0, p[2]])
    q_pre, pd = _oc_ik(E, pr + [0, 0, LID_ABOVE], LID_GY)
    if q_pre is None:
        raise ValueError("nắp: điểm chờ ngoài tầm")
    g, roll = _oc_roll_for_fixed(E, pr, LID_HW, q_pre, want_dir=(0.0, LID_WANT_Y))
    off = g - pr
    knob_h = knob
    if LID_HOOK > 0:
        # móc: hai ngón ôm cổ núm (trụ r 5 mm) dưới núm cầu, khe hẹp hơn núm cầu; núm tựa trên mép ngón, xoay tự do trong khe
        dd = off / max(np.linalg.norm(off), 1e-9)
        off = dd * (LID_HOOK_GAP / 2)
        knob_h = lambda th: knob(th) - LID_HOOK * np.array([0.0, math.sin(th), math.cos(th)])
    keep = r._targets[jr]
    r._targets[jr] = roll
    q_pre2, pd = _oc_ik(E, g + [0, 0, LID_ABOVE], LID_GY)
    r._targets[jr] = keep
    if q_pre2 is None:
        raise ValueError("nắp: điểm chờ ngoài tầm")
    st = {}
    bxy = r.backend.base_pose()
    via = [(LC.WORK_X - 0.05, bxy[1])] if bxy[0] > LC.WORK_X - 0.02 else None     # lùi khỏi bàn trước khi đi ngang
    dT = drive_T(E, bx0, by0, via=via, overlap=OC_VIA_OVERLAP)
    E.st.update({"lid_go": False, "rem_ld0": 1e9})

    def arm():
        Tp = max(T_PREP, dT - DR_LEAD)
        g_op = grip_sched("left", GRIP_AIR, 100, Tp - 0.5, 1.0)
        yield from jmove(E, "left", q_pre2, Tp, grips=[grip_sched("left", r._targets[gk], GRIP_AIR, 0.0, 1.0), g_op])
        bf = yield from base_near_end(E, "ld0", lead=DR_LEAD)
        ph = knob_h(LC.joint_q(r, jn))
        g1 = np.array([ph[0] - bf[0], ph[1] - bf[1], ph[2]]) + off
        cont = yield from approach(E, "left", E.tcp_now("left"), g1, pd, LID_GY, close_to=LID_GRIP, close_tail=DR_CLOSE_TAIL,
                                   grips=[lambda t: g_op(t + Tp)])
        E.st["lid_go"] = True
        _dbg(E, "grasped")
        bfx = E.st["end_ld0"]
        for attempt in range(LID_TRIES):
            th_a = LC.joint_q(r, jn)
            def path_o(s, th_a=th_a):
                th = th_a + (LID_OPEN_TH - th_a) * s
                w = mj((th - LID_SW) / 0.3) if LID_SW > 0 else 0.0
                return knob_h(th) + np.array([0.0, -LID_DYP * w, LID_DZ * w])
            pts = [np.asarray(path_o(i / 20)) - [bfx[0], bfx[1], 0] + off for i in range(21)]
            pf = _arm_only_pitch(E, pts, first=pd, gy=LID_GY)
            st["arm_only"] = pf is not None
            # nhả bắt đầu khi cung còn ARC_LEAD, mở chậm 1,3 s (chạy qua đoạn nâng 1 cm và đầu đoạn tiếp cận lại: không đứng hình)
            rel = grip_sched("left", LID_GRIP, 100, T_LID - LID_LEAD_O, 1.3)
            if pf is not None:          # đế đứng yên, tay đi hết cung (đế bám theo làm tay giằng với núm: rung, jerk cao)
                yield from track_arm(E, path_o, T_LID, off, bfx, pf, grips=[cont, rel], gy=LID_GY)
            else:
                yield from track(E, path_o, T_LID, off, pd, grips=[cont, rel])
            # nâng 1 cm (kẹp mở nốt trong lúc nâng), kẹp lại núm ở chỗ hiện tại
            p0 = E.tcp_now("left")
            yield from cpath(E, "left", [p0, p0 + [0, 0, LID_UP]], [pd, pd], [LID_GY, LID_GY], T=0.6,
                             grips=[lambda t, rel=rel: rel(t + T_LID)])
            st["q_open"] = LC.joint_q(r, jn)
            _dbg(E, "opened")
            if st["q_open"] >= LID_OK_TH or attempt + 1 == LID_TRIES:
                break
            # núm trượt khỏi kẹp (nắp chưa mở đủ): về trên núm ở góc hiện tại 4 cm, kẹp lại, mở lại
            log(f"nắp: lần {attempt + 1} chỉ mở {st['q_open']:.2f} rad, kẹp lại")
            k1 = knob(st["q_open"])
            g1 = np.array([k1[0] - bfx[0], k1[1] - bfx[1], k1[2]]) + off
            p0 = E.tcp_now("left")
            yield from cpath(E, "left", [p0, g1 + [0, 0, LID_ABOVE]], [pd, pd], [LID_GY, LID_GY])
            cont = yield from approach(E, "left", E.tcp_now("left"), g1, pd, LID_GY, close_to=LID_GRIP, close_tail=DR_CLOSE_TAIL)
        b = r.backend.base_pose()
        k2 = knob(st["q_open"])
        g2 = np.array([k2[0] - b[0], k2[1] - b[1], k2[2]]) + off
        cont2 = yield from approach(E, "left", E.tcp_now("left"), g2, pd, LID_GY, close_to=LID_GRIP, close_tail=DR_CLOSE_TAIL,
                                    grips=[lambda t: rel(t + T_LID + 0.6)])     # kẹp mở nốt trong lúc hạ lại
        th_b = LC.joint_q(r, jn)
        _dbg(E, "regrasp")
        def path_c(s):
            th = th_b + (LID_CLOSE_TH - th_b) * s
            w = mj((th - LID_SW) / 0.3) if LID_SW > 0 else 0.0
            return knob(th) + np.array([0.0, -LID_DYC * w, -LID_DZC * w])
        pts = [np.asarray(path_c(i / 20)) - [b[0], b[1], 0] + off for i in range(21)]
        pf = _arm_only_pitch(E, pts, first=pd, gy=LID_GY)
        rel2 = grip_sched("left", LID_GRIP, 100, T_LID - LID_LEAD_C, 0.8)
        if pf is not None:
            yield from track_arm(E, path_c, T_LID, off, b[:2], pf, grips=[cont2, rel2], gy=LID_GY)
        else:
            yield from track(E, path_c, T_LID, off, pd, grips=[cont2, rel2])
        p0 = E.tcp_now("left")
        yield from cpath(E, "left", [p0, p0 + [0, 0, LID_ABOVE if last else LID_UPC]], [pd, pd], [LID_GY, LID_GY], T=0.6,
                         grips=[lambda t: rel2(t + T_LID)])
        E.st["lid_out"] = True
        _dbg(E, "closed")
        if last:
            yield from oc_home(E)

    def base():
        E.st["lid_out"] = False
        yield from drive(E, bx0, by0, lift_to=OC_LIFT, t_lift=min(3.0, max(1.0, dT * 0.6)), tag="ld0", via=via,
                         overlap=OC_VIA_OVERLAP)
        if last:
            yield from wait_until(E, lambda: E.st["lid_out"], timeout=120)
            if _os.environ.get("M1_OC_LID_NOBACK", "1") != "1":      # đo riêng nắp: đế đứng yên khi tay về HOME (đế lùi ngắn làm rung)
                b = r.backend.base_pose()
                yield from drive(E, LC.WORK_X - 0.05, b[1], tag="ld3")
    E.run(arm(), base())
    return jn, st["q_open"]


FR_BACK = 0.36          # đế: x = x tay nắm - 0,36; y = y tay nắm - ARM_Y (như demo cũ)
FR_ABOVE = 0.07         # điểm chờ 7 cm trên tay nắm
FR_HW = 0.008           # bán kính thanh tay nắm tủ lạnh
FR_OPEN_TH, FR_CLOSE_TH = 1.45, -0.02
T_FR = 5.0              # thời lượng tối thiểu của cung cửa
FR_T_CLOSE, FR_CLOSE_TAIL = 1.1, 1.0   # cung cửa dài (đế 0,55 s đầu < 5 mm/s): khép chậm 1,1 s, cung bắt đầu ngay khi tay tới tay nắm, kẹp khép nốt trong 0,6-0,8 s đầu của cung
FR_ACC = float(_os.environ.get("M1_OC_FR_ACC", str(BASE_ACC)))   # gia tốc đế khi đi tới chỗ đứng tủ lạnh
FR_UP = 0.05            # nhả ở cửa mở: nâng 5 cm rồi hạ lại kẹp (đế đứng yên)


def track_yaw(E, path, T_min, gy0, pd, grips=()):
    """Như track nhưng path(s) -> (điểm TCP thế giới, grip_yaw): đế mang x, y, tay giữ x, y trong khung robot, đổi z và grip_yaw.
    T kéo dài để gia tốc đế <= 0,8 BASE_ACC."""
    m = 400
    xy = np.array([np.asarray(path(mj(i / m))[0], float)[:2] for i in range(m + 1)])
    a_u = float(np.max(np.linalg.norm(np.diff(xy, n=2, axis=0), axis=1))) * m * m
    T = max(T_min, math.sqrt(a_u / (0.8 * BASE_ACC)))
    n = max(2, int(round(T / E.dt)))
    pr = E.tcp_now("left")
    for i in range(1, n + 1):
        t = i * E.dt
        s = mj(t / T)
        sv = mj_d((t - 0.5 * E.dt) / T) / T
        w, yw = path(s)
        s1, s0 = min(1.0, s + 1e-4), max(0.0, s - 1e-4)
        dw = (np.asarray(path(s1)[0], float) - np.asarray(path(s0)[0], float)) / (s1 - s0)
        q = E.ik("left", [pr[0], pr[1], float(w[2])], pd, yw)
        out = E.jd("left", q) if q is not None else {}
        if q is None:
            E.ik_miss += 1
        out["_base"] = (float(dw[0] * sv), float(dw[1] * sv))
        for g in grips:
            out.update(g(t))
        yield out
    yield {"_base": (0.0, 0.0)}
    return T


def oc_fridge(E, log, last=True):
    """Cửa tủ lạnh: kẹp chĩa xuống ôm thanh tay nắm đứng (trục đóng theo cửa), đế mang tay theo cung cửa tới 1,45 rad, nhả,
    đế tới tay nắm ở cửa mở, kẹp lại, đóng theo cung về -0,02, nhả. Các bước nối gối nhau như ngăn kéo và nắp."""
    import lab_open_close as LO
    r = E.r
    jn = "fridge_hinge"
    gk = spec.joint_name("left", "gripper")
    handle, arc = LO._fridge_geom(r)
    st = {}

    def station(th):
        p = handle(th)
        return p, (p[0] - FR_BACK, p[1] - LC.ARM_Y)

    def plan(th, bxy):
        p, _ = station(th)
        gy = math.pi / 2 + th
        pr = np.array([p[0] - bxy[0], p[1] - bxy[1], p[2]])
        g = r.grasp_point("left", pr, FR_HW, grip_yaw=gy, lift=OC_LIFT)
        g = pr if g is None else np.asarray(g, float)
        pd = None
        for c in PITCH_DOWN:
            if E.ik("left", g, c, gy, lift=OC_LIFT) is not None and E.ik("left", g + [0, 0, FR_ABOVE], c, gy, lift=OC_LIFT) is not None:
                pd = c
                break
        if pd is None:
            raise ValueError("tủ lạnh: tay nắm ngoài tầm")
        return g, g - pr, gy, pd

    def leg(th_from, th_to, first):
        p, (bx, by) = station(th_from)
        g, off, gy, pd = plan(th_from, (bx, by))
        q_pre = E.ik("left", g + [0, 0, FR_ABOVE], pd, gy, lift=OC_LIFT)
        b = r.backend.base_pose()
        via = [(min(b[0], bx), b[1])] if abs(b[1] - by) > 0.1 else None     # lùi trước rồi mới đi ngang
        dT = drive_T(E, bx, by, via=via, overlap=OC_VIA_OVERLAP, acc=FR_ACC)
        tag = f"fr{int(first)}"
        E.st[f"rem_{tag}"] = 1e9
        E.st["fr_leg_done"] = False

        def arm():
            Tp = max(T_PREP, dT - DR_LEAD)
            g_op = grip_sched("left", GRIP_AIR, 100, Tp - 0.5, 1.1)       # mở chạy qua chỗ nối tư thế chờ -> tiếp cận
            yield from jmove(E, "left", q_pre, Tp, grips=[grip_sched("left", r._targets[gk], GRIP_AIR, 0.0, 1.0), g_op])
            bf = yield from base_near_end(E, tag, lead=DR_LEAD)
            g1 = np.array([p[0] - bf[0], p[1] - bf[1], p[2]]) + off
            cont = yield from approach(E, "left", E.tcp_now("left"), g1, pd, gy, close_to=0, t_close=FR_T_CLOSE, close_tail=FR_CLOSE_TAIL,
                                       grips=[lambda t: g_op(t + Tp)])
            yield from arc_part(cont)

        def arm2():
            # cửa đã mở: đế đứng yên (đã ở chỗ đứng theo cung), tay hạ thẳng xuống tay nắm, kẹp mở nốt trong lúc hạ
            bf = r.backend.base_pose()[:2]
            g1, off2, _, _ = plan(LC.joint_q(r, jn), bf)
            st["off"] = off2
            rel0, t0 = st["rel"]
            cont = yield from approach(E, "left", E.tcp_now("left"), g1, pd, gy, close_to=0, t_close=FR_T_CLOSE, close_tail=FR_CLOSE_TAIL,
                                       grips=[lambda t: rel0(t + t0)])
            yield from arc_part(cont)

        def arc_part(cont):
            off_ = st.get("off", off) if not first else off
            th_a = LC.joint_q(r, jn)
            yaw0 = math.pi / 2 + th_a

            def path(s, th_a=th_a):
                w, yw = arc(th_a, th_to)(s)
                c, sn = math.cos(yw - yaw0), math.sin(yw - yaw0)
                return np.asarray(w, float) + [c * off_[0] - sn * off_[1], sn * off_[0] + c * off_[1], off_[2]], yw
            # T của cung tính trước để lịch nhả bắt đầu khi cung còn ARC_LEAD
            m = 400
            xy = np.array([np.asarray(path(mj(i / m))[0], float)[:2] for i in range(m + 1)])
            a_u = float(np.max(np.linalg.norm(np.diff(xy, n=2, axis=0), axis=1))) * m * m
            Ta = max(T_FR, math.sqrt(a_u / (0.8 * BASE_ACC)))
            rel = grip_sched("left", 0, 100, Ta - ARC_LEAD, 1.6 if first else 0.8)
            yield from track_yaw(E, path, Ta, yaw0, pd, grips=[cont, rel])
            p0 = E.tcp_now("left")
            yw_end = math.pi / 2 + th_to
            up = FR_UP if first else 0.06
            yield from cpath(E, "left", [p0, p0 + [0, 0, up]], [pd, pd], [yw_end, yw_end], T=0.6,
                             grips=[lambda t: rel(t + Ta)])
            st["rel"] = (rel, Ta + 0.6)
            st[f"q_{int(first)}"] = LC.joint_q(r, jn)
            E.st["fr_leg_done"] = True
            if not first and last:
                yield from oc_home(E)

        def base():
            yield from drive(E, bx, by, lift_to=OC_LIFT, t_lift=min(3.0, max(1.0, dT * 0.6)), tag=tag, via=via,
                             overlap=OC_VIA_OVERLAP, acc=FR_ACC)
            if not first and last:
                yield from wait_until(E, lambda: E.st["fr_leg_done"], timeout=120)
                b = r.backend.base_pose()
                yield from drive(E, b[0] - 0.10, b[1], tag="fr9")
        def base2():
            if last:
                yield from wait_until(E, lambda: E.st["fr_leg_done"], timeout=120)
                b = r.backend.base_pose()
                yield from drive(E, b[0] - 0.10, b[1], tag="fr9")
        if first:
            E.run(arm(), base())
        else:
            E.run(arm2(), base2())
    log(f"tủ lạnh: tay nắm {np.round(handle(0.0), 3).tolist()}")
    leg(LC.joint_q(r, jn), FR_OPEN_TH, True)
    leg(LC.joint_q(r, jn), FR_CLOSE_TH, False)
    return jn, st["q_1"]


# ---------------- O&C cửa tủ lạnh, bản cổng 4 "không xuyên vật" (docs/CHOREO-M1-ketqua.md mục 9) ----------------
# Bản 8.4 (oc_fridge): kẹp chĩa thẳng xuống, trục đóng dọc cánh cửa, đế tịnh tiến theo cung: camera cổ tay + giá camera và servo cổ tay
# (khớp yaw, nhô ~6 cm về phía trước theo hướng với của tay) lún vào cửa tới 58 mm. Bản này:
# - kẹp ôm thanh tay nắm đứng từ cạnh: dụng cụ chúc 70 độ (ngẩng 20 độ khỏi phương thẳng đứng), trục đóng ngón vuông góc mặt cửa
#   (grip_yaw = θ trong khung robot), hai ngón dọc thanh; camera cổ tay quay dọc cánh cửa, không quay vào mặt cửa;
# - đế "ghim" vào cửa: trong lúc kéo/đẩy cửa, đế vừa tịnh tiến vừa quay đúng bằng góc cửa quanh bản lề, nên tư thế tay so với cửa
#   không đổi suốt cung (khớp tay đứng yên, chỉ kẹp đóng/mở), khe tay - cửa giữ như lúc kẹp (dò tĩnh: >= 1,9 cm mọi khâu kể cả camera);
# - quay đế theo trần SDK (80%): |ω| <= BASE_MAX_W_TIP, |v|·|ω| <= BASE_TURN_ACC_MAX, gia tốc góc <= BASE_MAX_ALPHA, R·α <= BASE_ACC.
FRP_BX = float(_os.environ.get("M1_FRP_BX", "0.36"))      # tay nắm trong khung robot lúc kẹp (x, y)
FRP_BY = float(_os.environ.get("M1_FRP_BY", "0.19"))
FRP_PD = float(_os.environ.get("M1_FRP_PD", "-70"))       # góc chúc dụng cụ (độ)
FRP_GYO = -math.pi / 2                                    # trục đóng ngón vuông góc mặt cửa
FRP_DZ = float(_os.environ.get("M1_FRP_DZ", "0.015"))     # TCP cao hơn tâm tay nắm 1,5 cm: vỏ kẹp không tì đỉnh thanh (0,80)
FRP_K = float(_os.environ.get("M1_FRP_K", "1.0"))         # bám tư thế đế (1/s)
OC_FR_PIN = _os.environ.get("M1_OC_FR_PIN", "1") == "1"
FRP_CLOSE_TH = float(_os.environ.get("M1_FRP_CLOSE_TH", "0.0"))   # đóng về 0 (không -0,02 như 8.4: cửa chạm khung ở 0, đế ghim quay tiếp thì cổ tay ép vào cửa)
FRP_REL_LEAD = float(_os.environ.get("M1_FRP_REL_LEAD", "0.8"))   # nhả khi cung còn 0,8 s (đuôi đế < 5 mm/s ~0,55 s nằm trong lúc kẹp mở)


class _WzEngine(Engine):
    """Engine có quay đế: "_base" = (vx, vy, wz) khung robot. Dùng chung st/log với engine gốc. Chỉ cho bước cửa tủ lạnh (cổng 4)."""

    def __init__(self, E):
        super().__init__(E.r)
        self.st, self.log = E.st, E.log

    def run(self, *gens):
        gens = [g for g in gens if g is not None]
        while gens:
            tg, base, alive = {}, None, []
            for g in gens:
                try:
                    y = next(g)
                except StopIteration:
                    continue
                alive.append(g)
                if y:
                    y = dict(y)
                    b = y.pop("_base", None)
                    if b is not None:
                        base = b
                    tg.update(y)
            gens = alive
            if not gens:
                break
            if base is not None:
                self.r.set_base_velocity(base[0], base[1], base[2] if len(base) > 2 else 0.0)
            if tg:
                self.r.set_joint_positions(tg)
            self.r.step()


def _frp_prof(R, dth):
    """Hồ sơ góc của cung đế ghim cửa: tăng tốc (vận tốc theo đa thức bậc 5, liên tục tới gia tốc) - chạy đều - giảm tốc, trong 80% trần
    quay SDK: ω <= BASE_MAX_W_TIP và sqrt(BASE_TURN_ACC_MAX / R) (|v|·|ω| = R ω²), gia tốc góc <= BASE_MAX_ALPHA và BASE_ACC / R.
    (Min-jerk một mạch dài ~20 s có hai đuôi chậm ~1,5 s đế < 5 mm/s, tay đứng yên: đứng hình.) Trả (T, f) với f(t) -> (Δθ, θ')."""
    D, sg = abs(dth), (1.0 if dth >= 0 else -1.0)
    R = max(R, 1e-3)
    w_c = min(0.8 * spec.BASE_MAX_W_TIP, math.sqrt(0.8 * spec.BASE_TURN_ACC_MAX / R))
    a_l = min(0.8 * spec.BASE_MAX_ALPHA, 0.8 * BASE_ACC / R)
    Tr = 1.875 * w_c / a_l
    if D < w_c * Tr:
        w_c = math.sqrt(max(D, 1e-9) * a_l / 1.875)
        Tr = 1.875 * w_c / a_l
    Tc = max(0.0, D / w_c - Tr)
    T = 2 * Tr + Tc
    P = lambda u: 2.5 * u ** 4 - 3 * u ** 5 + u ** 6
    H = lambda u: 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5

    def f(t):
        t = min(max(t, 0.0), T)
        if t < Tr:
            u = t / Tr
            return sg * w_c * Tr * P(u), sg * w_c * H(u)
        if t < Tr + Tc:
            return sg * (w_c * Tr / 2 + w_c * (t - Tr)), sg * w_c
        u = (T - t) / Tr
        return sg * (D - w_c * Tr * P(u)), sg * w_c * H(u)
    return T, f


def _frp_grasp(c_r, th_r, ref=None):
    """TCP (khung robot) kẹp thanh tay nắm tâm c_r, cửa ở góc th_r (khung robot). ref: nhánh cuộn (None: nghiệm gần cuộn 0).
    Trả (TCP, grip_yaw, q kẹp, q điểm chờ FR_ABOVE trên điểm kẹp)."""
    gy = math.pi / 2 + th_r + FRP_GYO
    mnt = K.arm_mount("left", OC_LIFT)
    pit = FRP_PD * DEG
    c = np.asarray(c_r, float) - mnt
    if ref is None:
        q0 = K.arm_ik(c, pitch=pit, grip_yaw=gy, side="left", ref_roll=0.0)
        if q0 is None:
            raise ValueError("tủ lạnh (cổng 4): tay nắm ngoài tầm")
        ref = float(q0[5])
    t = K.grasp_tcp(c, FR_HW, side="left", grip_yaw=gy, pitch=pit, ref_roll=ref)
    if t is None:
        raise ValueError("tủ lạnh (cổng 4): điểm kẹp ngoài tầm")
    q = K.arm_ik(t, pitch=pit, grip_yaw=gy, side="left", ref_roll=ref)
    q_up = K.arm_ik(t + [0, 0, FR_ABOVE], pitch=pit, grip_yaw=gy, side="left", ref_roll=ref)
    if q is None or q_up is None:
        raise ValueError("tủ lạnh (cổng 4): điểm kẹp ngoài tầm")
    return t + mnt, gy, q, q_up


def oc_fridge_pin(E0, log, last=True):
    """Cửa tủ lạnh O&C theo cổng 4 (xem chú thích trên). Mở tới FR_OPEN_TH, nhả, nâng, hạ kẹp lại, đóng về FR_CLOSE_TH, nhả."""
    import lab_open_close as LO
    E = _WzEngine(E0)
    r = E.r
    jn = "fridge_hinge"
    gk = spec.joint_name("left", "gripper")
    handle, _ = LO._fridge_geom(r)
    hinge = _door(r)
    z_h = handle(0.0)[2]
    st = {}

    def rot(a, v):
        return np.array([math.cos(a) * v[0] - math.sin(a) * v[1], math.sin(a) * v[0] + math.cos(a) * v[1]])

    def to_robot(th):
        b = r.backend.base_pose()
        rel = rot(-b[2], handle(th)[:2] - b[:2])
        return np.array([rel[0], rel[1], z_h + FRP_DZ]), th - b[2]

    def arc(th_to, grips=()):
        """Đế ghim vào cửa: quay + tịnh tiến theo góc cửa (min-jerk), tay giữ nguyên khớp."""
        b0 = r.backend.base_pose()
        th_a = LC.joint_q(r, jn)
        rel = rot(-b0[2], handle(th_a)[:2] - b0[:2])
        yaw_a = float(b0[2])

        def bdes(th):
            yaw = yaw_a + (th - th_a)
            return np.concatenate([handle(th)[:2] - rot(yaw, rel), [yaw]])
        T, prof = _frp_prof(float(np.linalg.norm(b0[:2] - hinge)), th_to - th_a)
        n = max(2, int(round(T / E.dt)))
        for i in range(n):
            t = (i + 0.5) * E.dt
            d_th, thd = prof(t)
            th = th_a + d_th
            bd = bdes(th)
            db = (bdes(th + 1e-4) - bdes(th - 1e-4)) / 2e-4 * thd
            b = r.backend.base_pose()
            err = bd - b
            err[2] = K._wrap(err[2])
            vr = rot(-b[2], db[:2] + FRP_K * err[:2])
            out = {"_base": (float(vr[0]), float(vr[1]), float(db[2] + FRP_K * err[2]))}
            for g in grips:
                out.update(g((i + 1) * E.dt))
            yield out
        for _ in range(3):
            yield {"_base": (0.0, 0.0, 0.0)}

    # đế tới chỗ đứng (tay nắm ở (FRP_BX, FRP_BY) khung robot, yaw 0), tay vào điểm chờ trên tay nắm, tiếp cận, khép
    th0 = LC.joint_q(r, jn)
    h0 = handle(th0)
    bx, by = h0[0] - FRP_BX, h0[1] - FRP_BY
    b = r.backend.base_pose()
    via = [(min(b[0], bx), b[1])] if abs(b[1] - by) > 0.1 else None
    dT = drive_T(E, bx, by, via=via, overlap=OC_VIA_OVERLAP, acc=FR_ACC)
    _, _, _, q_pre = _frp_grasp([FRP_BX, FRP_BY, z_h + FRP_DZ], th0)
    E.st["rem_frp0"] = 1e9
    log(f"tủ lạnh (cổng 4): tay nắm {np.round(h0, 3).tolist()}, chỗ đứng ({bx:.3f}; {by:.3f})")

    def grasp_and_arc(th_to, first):
        if first:
            Tp = max(T_PREP, dT - DR_LEAD)
            g_op = grip_sched("left", GRIP_AIR, 100, Tp - 0.5, 1.1)
            yield from jmove(E, "left", q_pre, Tp, grips=[grip_sched("left", r._targets[gk], GRIP_AIR, 0.0, 1.0), g_op])
            yield from base_near_end(E, "frp0", lead=DR_LEAD)
            gopen = lambda t: g_op(t + Tp)
            ref = None
        else:
            rel0, t0 = st["rel"]
            gopen = lambda t: rel0(t + t0)
            ref = float(r._targets[spec.joint_name("left", "wrist_roll")])
        c_r, th_r = to_robot(LC.joint_q(r, jn))
        g, gy, _, _ = _frp_grasp(c_r, th_r, ref)
        cont = yield from approach(E, "left", E.tcp_now("left"), g, FRP_PD, gy, close_to=0, t_close=FR_T_CLOSE,
                                   close_tail=FR_CLOSE_TAIL, grips=[gopen])
        # cung: kẹp khép nốt, nhả khi cung còn ARC_LEAD (cùng công thức thời lượng với arc)
        Ta = _frp_prof(float(np.linalg.norm(r.backend.base_pose()[:2] - hinge)), th_to - LC.joint_q(r, jn))[0]
        rel = grip_sched("left", 0, 100, Ta - FRP_REL_LEAD, 1.6 if first else 0.8)
        yield from arc(th_to, grips=[cont, rel])
        p0 = E.tcp_now("left")
        up = FR_UP if first else 0.06
        yield from cpath(E, "left", [p0, p0 + [0, 0, up]], [FRP_PD, FRP_PD], [gy, gy], T=0.6, grips=[lambda t: rel(t + Ta)])
        st["rel"] = (rel, Ta + 0.6)
        st[f"q_{int(first)}"] = LC.joint_q(r, jn)
        if not first and last:
            E.st["frp_done"] = True
            yield from oc_home(E)

    def base0():
        yield from drive(E, bx, by, lift_to=OC_LIFT, t_lift=min(3.0, max(1.0, dT * 0.6)), tag="frp0", via=via,
                         overlap=OC_VIA_OVERLAP, acc=FR_ACC)
    E.run(grasp_and_arc(FR_OPEN_TH, True), base0())
    log(f"tủ lạnh (cổng 4): cửa mở {st['q_1']:.3f} rad, đế {np.round(r.backend.base_pose(), 3).tolist()}")
    E.st["frp_done"] = False

    def base1():
        if last:
            yield from wait_until(E, lambda: E.st["frp_done"], timeout=120)
            b = r.backend.base_pose()
            yield from drive(E, b[0] - 0.10, b[1], tag="frp9")
    E.run(grasp_and_arc(FRP_CLOSE_TH, False), base1())
    return jn, st["q_1"]


def _oc_result(item_fn, jn, q_open, q_closed):
    js = next(ln["joint"] for ln in item_fn()["links"] if (ln.get("joint") or {}).get("name") == jn)
    ok_o, ok_c = q_open >= js["open_min"], q_closed <= js["closed_max"]
    return {"joint": jn, "open_q": round(q_open, 4), "closed_q": round(q_closed, 4), "open_ok": bool(ok_o), "closed_ok": bool(ok_c),
            "ok": bool(ok_o and ok_c)}


def open_close(robot, only=("drawer", "lid", "fridge"), snap=None, log=print) -> dict:
    """O&C biên đạo mới: các bước nối gối nhau, kiểm khớp sau khi xong cả bài (chờ 1 s ở cuối, không chờ giữa bài)."""
    E = Engine(robot)
    E.log = log
    done = []
    for k, name in enumerate(only):
        last = k == len(only) - 1 or (only[k + 1] == "fridge" and _os.environ.get("M1_OC_FRIDGE_OLD") == "1")
        if name == "drawer":
            dp = _os.environ.get("M1_OC_DRAWER_PLACE", OC_DRAWER_PLACE_DEFAULT) == "1"
            jn, qo = (oc_drawer_place if dp else oc_drawer)(E, log, last=last)
            done.append((L.cabinet, jn, qo))
        elif name == "lid":
            jn, qo = oc_lid(E, log, last=last)
            done.append((L.lid_box, jn, qo))
        elif _os.environ.get("M1_OC_FRIDGE_OLD") == "1":
            # so sánh: cửa tủ lạnh theo bước cũ (đế bám cung tay nắm, có chờ giữa bước)
            import lab_open_close as LO
            rr = LO.fridge(robot, snap or (lambda *a, **k: None), log)
            done.append((L.fridge, rr["joint"], rr["open_q"]))
        elif _os.environ.get("M1_OC_FRIDGE_CAN", OC_FRIDGE_CAN_DEFAULT) == "1":
            # mục 8.7: tay phải mở cửa, tay trái lấy lon đặt lên bàn (nguyên mục 5), tay phải đẩy mặt ngoài đóng cửa
            can_oid = L.FRIDGE_OBJ[0]
            rr = oc_fridge_can_push(robot, can_oid, OC_CAN_TARGET, snap=snap, log=log)
            done.append((L.fridge, "fridge_hinge", rr["open_q"]))
        else:
            jn, qo = (oc_fridge_pin if OC_FR_PIN else oc_fridge)(E, log, last=last)   # cổng 4: bản ghim đế vào cửa (mục 9)
            done.append((L.fridge, jn, qo))
    robot.wait(1.0)
    out = {n: _oc_result(fn, jn, qo, LC.joint_q(robot, jn)) for n, (fn, jn, qo) in zip(only, done)}
    if "drawer" in out and _os.environ.get("M1_OC_DRAWER_PLACE", OC_DRAWER_PLACE_DEFAULT) == "1":
        oi = obj_in_drawer(robot, DP_OBJ)       # mục 8.8: vật nằm trong lòng ngăn đã đóng
        out["drawer"]["obj_in_drawer"] = oi
        out["drawer"]["ok"] = bool(out["drawer"]["ok"] and oi["ok"])
    if "fridge" in only and _os.environ.get("M1_OC_FRIDGE_CAN", OC_FRIDGE_CAN_DEFAULT) == "1" and _os.environ.get("M1_OC_FRIDGE_OLD") != "1":
        out["fridge_can"] = LC.in_zone(robot, L.FRIDGE_OBJ[0], OC_CAN_TARGET, 0.04)
    return out


OC_DRAWER_PLACE_DEFAULT = "0"   # "1": ngăn kéo có đặt vật (mục 8.8, hai tay); "0": chỉ kéo/đẩy ngăn (mục 8.3, tay trái)
OC_FRIDGE_CAN_DEFAULT = "0"     # "1": phần tủ lạnh của O&C theo mục 8.7 (lấy lon + đẩy đóng); "0": mục 8.4 (chỉ mở/đóng cửa)
OC_CAN_TARGET = (0.56, -0.95)   # như lab_pick_place.CAN_TARGET



# ---------------- O&C tủ lạnh có lon (docs/CHOREO-M1-ketqua.md mục 8.7) ----------------
# Mở cửa + lấy lon chép nguyên mục 5 (fridge_fetch: tay phải kéo rồi đẩy cửa, tay trái lấy lon, đặt ở (0,56; -0,95)).
# Đóng cửa: tay trái kẹp lại tay nắm ở góc cửa đang trôi tới (~1,65 rad), đế mang tay theo cung về -0,02 như bước đóng mục 8.4.

def oc_fridge_close(E, log, th_to=None):
    """Đóng cửa tủ lạnh sau fridge_fetch: đế tới chỗ đứng theo tay nắm ở góc hiện tại, tay trái vào điểm chờ trên tay nắm,
    tiếp cận khi đế còn DR_LEAD, khép, đế mang tay theo cung về FR_CLOSE_TH, nhả, nâng, tay về HOME, đế lùi 0,10 m."""
    import lab_open_close as LO
    r = E.r
    jn = "fridge_hinge"
    gk = spec.joint_name("left", "gripper")
    th_to = FR_CLOSE_TH if th_to is None else th_to
    handle, arc = LO._fridge_geom(r)
    th_from = LC.joint_q(r, jn)
    # cửa đẩy tới 1,30 rồi trôi tới chặn 1,75 (range cảnh) trong lúc đế đi: chọn chỗ đứng theo góc chặn khi cửa đã > 1,5
    th_hi = next(ln["joint"] for ln in L.fridge()["links"] if (ln.get("joint") or {}).get("name") == jn)["range"][1]
    th_plan = max(th_from, th_hi) if th_from > 1.5 else th_from
    p = handle(th_plan)
    bx = p[0] - float(_os.environ.get("M1_OCF_BACK", FR_BACK))
    by = p[1] - LC.ARM_Y + float(_os.environ.get("M1_OCF_DY", "0"))
    # cửa > 90 độ: grip_yaw π/2 + θ vượt π; kẹp hai ngón đối xứng nên dùng π/2 + θ - π (grasp_point tính lại phía ngón cố định)
    YS = -math.pi if (th_plan > math.pi / 2 and _os.environ.get("M1_OCF_YS", "1") != "0") else 0.0
    gy = math.pi / 2 + th_plan + YS
    pr = np.array([p[0] - bx, p[1] - by, p[2]])
    g = r.grasp_point("left", pr, FR_HW, grip_yaw=gy, lift=OC_LIFT)
    g = pr if g is None else np.asarray(g, float)
    off = g - pr
    pd = None
    for c in PITCH_DOWN:
        if E.ik("left", g, c, gy, lift=OC_LIFT) is not None and E.ik("left", g + [0, 0, FR_ABOVE], c, gy, lift=OC_LIFT) is not None:
            pd = c
            break
    if pd is None:
        raise ValueError("tủ lạnh (đóng): tay nắm ngoài tầm")
    q_pre = E.ik("left", g + [0, 0, FR_ABOVE], pd, gy, lift=OC_LIFT)
    b = r.backend.base_pose()
    via = [(min(b[0], bx), b[1])] if abs(b[1] - by) > 0.1 else None
    dT = drive_T(E, bx, by, via=via, overlap=OC_VIA_OVERLAP)
    E.st["rem_frc"] = 1e9
    E.st["frc_done"] = False
    log(f"tủ lạnh (đóng): cửa {th_from:.3f} rad, chỗ đứng ({bx:.3f}; {by:.3f})")

    def arm():
        Tp = max(T_PREP, dT - DR_LEAD)
        g_op = grip_sched("left", GRIP_AIR, 100, Tp - 0.5, 1.1)
        yield from jmove(E, "left", q_pre, Tp, grips=[grip_sched("left", r._targets[gk], GRIP_AIR, 0.0, 1.0), g_op])
        bf = yield from base_near_end(E, "frc", lead=DR_LEAD)
        # cửa còn trôi trong lúc đế đi (1,65 -> ~1,75 rad): tính lại tay nắm, grip_yaw và độ lệch TCP theo góc lúc bắt đầu tiếp cận
        th_a = LC.joint_q(r, jn)
        pa = handle(th_a)
        yaw0 = math.pi / 2 + th_a
        c0, s0 = math.cos(th_a - th_plan), math.sin(th_a - th_plan)
        off_a = np.array([c0 * off[0] - s0 * off[1], s0 * off[0] + c0 * off[1], off[2]])
        g1 = np.array([pa[0] - bf[0], pa[1] - bf[1], pa[2]]) + off_a
        cont = yield from approach(E, "left", E.tcp_now("left"), g1, pd, yaw0 + YS, close_to=0, t_close=FR_T_CLOSE, close_tail=FR_CLOSE_TAIL,
                                   grips=[lambda t: g_op(t + Tp)])
        off0 = off_a

        def path(s):
            w, yw = arc(th_a, th_to)(s)
            c, sn = math.cos(yw - yaw0), math.sin(yw - yaw0)
            return np.asarray(w, float) + [c * off0[0] - sn * off0[1], sn * off0[0] + c * off0[1], off0[2]], yw + YS
        m = 400
        xy = np.array([np.asarray(path(mj(i / m))[0], float)[:2] for i in range(m + 1)])
        a_u = float(np.max(np.linalg.norm(np.diff(xy, n=2, axis=0), axis=1))) * m * m
        Ta = max(T_FR, math.sqrt(a_u / (0.8 * BASE_ACC)))
        rel = grip_sched("left", 0, 100, Ta - ARC_LEAD, 0.8)
        yield from track_yaw(E, path, Ta, yaw0 + YS, pd, grips=[cont, rel])
        p0 = E.tcp_now("left")
        yw_end = math.pi / 2 + th_to + YS
        yield from cpath(E, "left", [p0, p0 + [0, 0, 0.06]], [pd, pd], [yw_end, yw_end], T=0.6, grips=[lambda t: rel(t + Ta)])
        E.st["frc_done"] = True
        yield from oc_home(E)

    def base():
        yield from drive(E, bx, by, lift_to=OC_LIFT, t_lift=min(3.0, max(1.0, dT * 0.6)), tag="frc", via=via, overlap=OC_VIA_OVERLAP)
        yield from wait_until(E, lambda: E.st["frc_done"], timeout=120)
        b2 = r.backend.base_pose()
        yield from drive(E, b2[0] - 0.10, b2[1], tag="frc9")
    E.run(arm(), base())
    return th_from


def oc_fridge_can(robot, oid, target_xy, snap=None, log=print) -> dict:
    """O&C tủ lạnh có lon: fridge_fetch nguyên mục 5, rồi oc_fridge_close. Trả góc cửa lúc bắt đầu đóng."""
    E = Engine(robot)
    E.log = log
    fridge_fetch(robot, oid, target_xy, snap=snap, log=log, engine=E)
    return {"open_q": oc_fridge_close(E, log)}



# O&C tủ lạnh, đóng bằng đẩy mặt ngoài (mục 8.7, vòng 2): tay phải (đã mở cửa, đang rảnh) khép kẹp, chĩa xuống thành một chốt đẩy
# ở z PC_Z (trên đỉnh thanh tay nắm 0,80), TCP giữ cố định trong khung robot; đế mang chốt theo cung quanh bản lề, điểm tì trên mặt
# ngoài cửa trượt từ v PC_V0 (gần mép cửa) về PC_V1 khi cửa từ góc lúc bắt đầu về 0. Không kẹp lại, không dừng giữa bước.
PC_Z = float(_os.environ.get("M1_PC_Z", "0.87"))
PC_X = float(_os.environ.get("M1_PC_X", "0.30"))      # TCP khung robot x (chĩa xuống ở z 0,87 tới x <= 0,33)
PC_V0, PC_V1 = float(_os.environ.get("M1_PC_V0", "0.47")), float(_os.environ.get("M1_PC_V1", "0.32"))
PC_GAP = float(_os.environ.get("M1_PC_GAP", "0.010"))  # tâm chốt cách mặt ngoài cửa (nửa bề dày hai ngón khép)
PC_TH_END = float(_os.environ.get("M1_PC_TH_END", "-0.01"))
PC_K = float(_os.environ.get("M1_PC_K", "0.5"))        # hệ số bám vị trí đế (1/s)


PC_DELAY = float(_os.environ.get("M1_PC_DELAY", "1.2"))   # chạy cùng bước nhả lon của tay trái: đế, tay phải bắt đầu sau 1,2 s
PC_ACC = float(_os.environ.get("M1_PC_ACC", "0.6"))       # gia tốc đế kế hoạch của cung = PC_ACC * BASE_ACC
PC_TAIL = float(_os.environ.get("M1_PC_TAIL", "1.5"))     # tay phải bắt đầu về HOME khi cung còn PC_TAIL s (cửa còn < 0,02 rad)
PC_IN_LEAD = float(_os.environ.get("M1_PC_IN_LEAD", "0.9"))   # tay vào chốt đẩy khi đế còn 0,9 s; cung bắt đầu ở 70% đoạn vào


def oc_fridge_push_close(E, log, gens_only=False):
    r = E.r
    jn = "fridge_hinge"
    door = L.FRIDGE
    u_c = -(door["door_t"] / 2 + PC_GAP)
    th_hi = next(ln["joint"] for ln in L.fridge()["links"] if (ln.get("joint") or {}).get("name") == jn)["range"][1]
    th_now = LC.joint_q(r, jn)
    th0 = max(th_now, th_hi) if th_now > 1.5 else th_now
    pr = np.array([PC_X, -LC.ARM_Y, PC_Z])
    q = None
    for pd in OC_PITCH + (-75.0, -70.0):
        q = E.ik("right", pr, pd, 0.0, lift=OC_LIFT)
        if q is not None:
            break
    if q is None:
        raise ValueError("tủ lạnh (đẩy đóng): chốt đẩy ngoài tầm")
    vv = lambda th: PC_V1 + (PC_V0 - PC_V1) * max(0.0, min(1.0, th / th0))
    cpt = lambda th: door_pt(r, th, u_c, vv(th), PC_Z)
    c0 = cpt(th0)
    b0 = (c0[0] - pr[0], c0[1] - pr[1])
    # chờ ngoài: chốt lùi 6 cm theo pháp tuyến ngoài của cửa (-u) rồi tiến vào khi đế còn 0,35 s
    out = -np.array([math.cos(th0), math.sin(th0)]) * 0.06
    pre_r = pr + [out[0], out[1], 0.0]
    q_pre = None
    for pd in OC_PITCH + (-75.0, -70.0):
        q_pre = E.ik("right", pre_r, pd, 0.0, lift=OC_LIFT)
        if q_pre is not None:
            break
    if q_pre is None:
        raise ValueError("tủ lạnh (đẩy đóng): điểm chờ ngoài tầm")
    bx, by, _ = r.backend.base_pose()
    # tay phải phải sang phía ngoài cửa (-y) mà không cắt qua cánh cửa: đế lùi tới x PC_BX (TCP chờ còn trước mép cửa), sang -y, rồi tiến
    xb = min(float(_os.environ.get("M1_PC_BX", "-0.42")), b0[0])
    via = [(xb, by), (xb, b0[1])]
    dT = drive_T(E, b0[0], b0[1], via=via, overlap=OC_VIA_OVERLAP)
    E.st["rem_pc"] = 1e9
    log(f"tủ lạnh (đẩy đóng): cửa {th_now:.3f} rad, chỗ đứng ({b0[0]:.3f}; {b0[1]:.3f})")
    # cung: min-jerk theo góc, thời lượng theo gia tốc đế
    m = 400
    xy = np.array([cpt(th0 + (PC_TH_END - th0) * mj(i / m))[:2] for i in range(m + 1)])
    a_u = float(np.max(np.linalg.norm(np.diff(xy, n=2, axis=0), axis=1))) * m * m
    v_u = float(np.max(np.linalg.norm(np.diff(xy, axis=0), axis=1))) * m
    Ta = max(5.0, math.sqrt(a_u / (PC_ACC * BASE_ACC)), 1.1 * v_u / 0.35)
    st = {}

    def arm():
        for _ in range(int(round(PC_DELAY / E.dt))):
            yield {}
        Tp = max(T_PREP, dT - PC_IN_LEAD)
        gk = spec.joint_name("right", "gripper")
        if _os.environ.get("M1_PC_SEP") == "1":
            yield from jmove(E, "right", q_pre, Tp, grips=[grip_sched("right", r._targets[gk], 0.0, 0.0, 1.0)])
            yield from base_near_end(E, "pc", lead=PC_IN_LEAD)
            yield from jmove(E, "right", q, 1.0, grips=[flag_at(E, "pc_go", 0.7)])
        else:
            # HOME -> điểm chờ ngoài -> chốt đẩy: hai min-jerk khớp cộng chồng (đoạn vào bắt đầu khi đoạn trước còn 0,3 s), theo giờ
            # kế hoạch của đế (drive_T): đoạn vào xong 0,3 s sau khi đế dừng, cung bắt đầu ở 70% đoạn vào
            Tp = max(T_PREP, dT - 0.4)
            t2 = Tp - 0.3
            q0 = E.q_now("right")
            g0 = grip_sched("right", r._targets[gk], 0.0, 0.0, 1.0)
            nn = int(round((t2 + 1.0) / E.dt))
            for k in range(1, nn + 1):
                t = k * E.dt
                qq = q0 + (np.asarray(q_pre) - q0) * mj(t / Tp) + (np.asarray(q) - np.asarray(q_pre)) * mj((t - t2) / 1.0)
                d = E.jd("right", qq)
                d.update(g0(t))
                if t >= t2 + 0.7:
                    E.st["pc_go"] = True
                yield d
        yield from wait_until(E, lambda: st.get("done", False), timeout=120)
        st["up"] = True
        log(f"tủ lạnh (đẩy đóng): cửa sau cung {LC.joint_q(r, jn):.3f} rad")
        yield from jmove(E, "right", np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS]), T_HOME)

    def base():
        for _ in range(int(round(PC_DELAY / E.dt))):
            yield {}
        yield from drive(E, b0[0], b0[1], lift_to=OC_LIFT, t_lift=min(3.0, max(1.0, dT * 0.6)), tag="pc", via=via,
                         overlap=OC_VIA_OVERLAP)
        yield from wait_until(E, lambda: E.st.get("pc_go", False), timeout=60)
        n = max(2, int(round(Ta / E.dt)))
        for i in range(n):
            s1 = (i + 1) / n
            s0 = i / n
            p1 = cpt(th0 + (PC_TH_END - th0) * mj(s1))
            p0 = cpt(th0 + (PC_TH_END - th0) * mj(s0))
            v = (p1[:2] - p0[:2]) / E.dt
            b = r.backend.base_pose()
            err = np.array([p0[0] - pr[0] - b[0], p0[1] - pr[1] - b[1]])
            v = v + PC_K * err
            if _os.environ.get("M1_PC_DBG") and i % 100 == 0:
                log(f"pc {i}/{n} base {np.round(b, 3).tolist()} tcp_r {np.round(E.tcp_now('right'), 3).tolist()} tcp_w {np.round(r.tcp_position('right'), 3).tolist()} want_c {np.round(p0, 3).tolist()} door {LC.joint_q(r, jn):.3f}")
            if i == n - int(round(PC_TAIL / E.dt)):
                st["done"] = True
            yield {"_base": (float(v[0]), float(v[1]))}
        st["done"] = True
        yield from wait_until(E, lambda: st.get("up", False), timeout=10)
        b = r.backend.base_pose()
        yield from drive(E, b[0] - 0.10, b[1], tag="pc9")
    if gens_only:
        return [arm(), base()], th_now
    E.run(arm(), base())
    return th_now


# O&C tủ lạnh có lon, đóng cửa theo cổng 4 (mục 9): bản đẩy mặt ngoài (oc_fridge_push_close) tì lòng kẹp phải và camera cổ tay vào cửa
# (quy tắc B). Bản này là chiều ngược của bước mở mục 5 (FR5_PULL): đế lùi tới chỗ đứng ứng với góc cửa đang mở (tay nắm ở FR5_REL khung
# robot), tay phải từ HOME vào điểm chờ sau tay nắm 8 cm (dọc mặt ngoài cửa, ngoài mép tự do), tiếp cận ngang, khép, đế tịnh tiến theo
# cung tay nắm về FR5_CLOSE_TH (khớp tay đứng yên), nhả, rút tay theo -x, về HOME.
OCF_PULL = _os.environ.get("M1_OCF_PULL", "1") == "1"
FR5_CLOSE_TH = float(_os.environ.get("M1_FR5_CLOSE_TH", "0.0"))
FR5_CLOSE_SIDE = float(_os.environ.get("M1_FR5_CLOSE_SIDE", "0.012"))
FR5_CLOSE_REL = float(_os.environ.get("M1_FR5_CLOSE_REL", "25"))   # nới kẹp ở cuối cung đóng (%)


def oc_fridge_pull_close(E, log, gens_only=False):
    import lab_open_close as LO
    r = E.r
    jn = "fridge_hinge"
    handle, _ = LO._fridge_geom(r)
    gk = spec.joint_name("right", "gripper")
    home = np.array([spec.HOME_ARM[j] for j in spec.ARM_JOINTS])
    th_now = LC.joint_q(r, jn)
    hw0 = handle(th_now)
    sx, sy = hw0[0] - FR5_REL[0], hw0[1] - FR5_REL[1]
    z_h = hw0[2]
    pre_r = np.array([FR5_REL[0], FR5_REL[1], z_h]) + FR5_PRE
    q_pre = E.ik("right", pre_r, 0.0, roll=0.0, lift=LIFT_FRIDGE)
    if q_pre is not None:
        for _ in range(2):
            o5 = np.asarray(K.fixed_side_dir(q_pre, "right"), float) * (spec.TCP_DY - spec.FIXED_FACE_Y + FR_HW + 0.002)
            q5 = E.ik("right", pre_r + [o5[0], o5[1], 0.0], 0.0, roll=0.0, lift=LIFT_FRIDGE)
            if q5 is None:
                break
            q_pre = q5
    if q_pre is None:
        raise ValueError("tủ lạnh (kéo đóng): điểm chờ ngoài tầm")
    bx, by, _ = r.backend.base_pose()
    via = [(sx, by)]           # lùi theo -x trước (tay phải ở HOME, ngoài mép tự do), rồi sang -y tới chỗ đứng
    dT = drive_T(E, sx, sy, via=via, overlap=OC_VIA_OVERLAP)
    E.st.update({"rem_pl": 1e9, "pl_go": False, "pl_done": False, "pl_up": False, "pl_ret": False})
    st = {}
    log(f"tủ lạnh (kéo đóng): cửa {th_now:.3f} rad, chỗ đứng ({sx:.3f}; {sy:.3f})")

    def arm():
        for _ in range(int(round(PC_DELAY / E.dt))):
            yield {}
        # vào điểm chờ khi đế gần tới chỗ đứng (đi sang -y lúc tay còn duỗi trước thì tay quét qua mép cửa)
        bf = yield from base_near_end(E, "pl")
        yield from jmove(E, "right", q_pre, T_PREP, grips=[grip_sched("right", r._targets[gk], 100, 0.0, 1.0)])
        hr = np.array([handle(LC.joint_q(r, jn))[0] - bf[0], handle(LC.joint_q(r, jn))[1] - bf[1], z_h])
        off = level_offset(E, "right", FR_HW)
        cont = yield from approach(E, "right", E.tcp_now("right"), hr + [off[0], off[1], 0.0], 0.0, 0.0, roll_mode=True,
                                   close_to=0, t_close=FR_T_CLOSE, close_tail=FR_CLOSE_TAIL)
        st["cont"] = cont
        E.st["pl_go"] = True
        yield from wait_until(E, lambda: E.st.get("pl_ret", False), timeout=120)
        rel_g, t0 = st["rel_g"]
        p0 = E.tcp_now("right")
        Tb = dur_for(FR5_BACK)
        # cửa đã đóng: rút -x là kéo cửa mở lại (đầu hàm động cong ôm sau thanh); mở hết kẹp, nâng qua đầu trên thanh tay nắm rồi mới lùi
        fd = np.asarray(K.fixed_side_dir(E.q_now("right"), "right"), float) * FR5_CLOSE_SIDE   # dịch về phía ngón cố định: hàm động rời thanh
        pts = [p0, p0 + [fd[0], fd[1], 0.01], p0 + [fd[0], fd[1], 0.05], p0 + [fd[0] - FR5_BACK, fd[1], 0.055]]
        pits = [0.0, 0.0, 0.0, E.level_pitch("right", [pts[3]])]
        # kẹp chỉ nới tới FR5_CLOSE_REL (mở rộng thì hàm động quét vào thanh, đẩy lệch tay); mở hẳn khi đã lên khỏi đầu thanh
        opn = grip_sched("right", FR5_CLOSE_REL, 60, 0.9, 0.4)
        yield from cpath(E, "right", pts, pits, [0.0] * 4, roll_mode=True, grips=[lambda t: rel_g(t + t0), opn])
        log(f"tủ lạnh (kéo đóng): cửa sau cung {LC.joint_q(r, jn):.3f} rad")
        E.st["pl_up"] = True
        yield from jmove(E, "right", home, T_HOME, grips=[grip_sched("right", 60, GRIP_AIR, 0.0, 1.0)])

    def base():
        for _ in range(int(round(PC_DELAY / E.dt))):
            yield {}
        yield from drive(E, sx, sy, lift_to=LIFT_FRIDGE, t_lift=min(3.0, max(1.0, dT * 0.6)), tag="pl", via=via,
                         overlap=OC_VIA_OVERLAP)
        yield from wait_until(E, lambda: E.st["pl_go"], timeout=60)
        b0 = r.backend.base_pose()
        th_a = LC.joint_q(r, jn)
        rel_b = np.array(handle(th_a)[:2]) - b0[:2]
        R_h = float(np.linalg.norm(np.array(handle(th_a)[:2]) - _door(r)))
        Lp = R_h * abs(FR5_CLOSE_TH - th_a)
        T = max(3.0, math.sqrt(5.774 * Lp / (0.8 * BASE_ACC)), 1.875 * Lp / 0.3)
        n = int(round(T / E.dt))
        rel_g = grip_sched("right", 0, FR5_CLOSE_REL, T - FR5_REL_LEAD, T_OPEN)
        cont = st["cont"]
        for i in range(n):
            t = (i + 0.5) * E.dt
            th = th_a + (FR5_CLOSE_TH - th_a) * mj(t / T)
            thd = (FR5_CLOSE_TH - th_a) * mj_d(t / T) / T
            bd = np.array(handle(th)[:2]) - rel_b
            db = (np.array(handle(th + 1e-4)[:2]) - np.array(handle(th - 1e-4)[:2])) / 2e-4 * thd
            b = r.backend.base_pose()
            v = db + FR5_K * (bd - b[:2])
            d = {"_base": (float(v[0]), float(v[1]))}
            d.update(cont((i + 1) * E.dt))
            d.update(rel_g((i + 1) * E.dt))
            if i == n - int(round(FR5_RET_LEAD / E.dt)):
                st["rel_g"] = (rel_g, (i + 1) * E.dt)
                E.st["pl_ret"] = True
            yield d
        E.st["pl_done"] = True
        yield {"_base": (0.0, 0.0)}
        yield from wait_until(E, lambda: E.st["pl_up"], timeout=20)
        b = r.backend.base_pose()
        yield from drive(E, b[0] - 0.10, b[1], tag="pl9")
    if gens_only:
        return [arm(), base()], th_now
    E.run(arm(), base())
    return th_now


def oc_fridge_can_push(robot, oid, target_xy, snap=None, log=print) -> dict:
    """O&C tủ lạnh có lon, đóng bằng đẩy mặt ngoài: fridge_fetch nguyên mục 5, rồi oc_fridge_push_close."""
    E = _PCEngine(robot)
    E.log = log
    fridge_fetch(robot, oid, target_xy, snap=snap, log=log, engine=E)
    return {"open_q": E.pc_th}


class _PCEngine(Engine):
    """Engine cho oc_fridge_can_push: lần chạy thứ 3 của fridge_fetch (tay trái nhả lon, rút, về HOME) chạy cùng bước đẩy đóng cửa,
    để không có chỗ nối đứng yên giữa hai bước. fridge_fetch giữ nguyên."""

    def __init__(self, robot):
        super().__init__(robot)
        self.pc_n = 0
        self.pc_th = None

    def run(self, *gens):
        self.pc_n += 1
        if self.pc_n == 3:
            extra, self.pc_th = (oc_fridge_pull_close if OCF_PULL else oc_fridge_push_close)(self, self.log, gens_only=True)   # cổng 4: kéo đóng
            gens = tuple(gens) + tuple(extra)
        return super().run(*gens)
