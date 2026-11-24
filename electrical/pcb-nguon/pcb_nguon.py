"""PCB nguồn Faibotic M1, schematic v0.1 bằng SKiDL (không dùng thư viện ký hiệu KiCad).
v0.1 (21/11/2026): áp quyết định D3/D4 (../quyet-dinh-bms-estop.md Phần 3, QC2 vòng 2):
pack 3S5P, contactor ngoài PCB + nạp trước K4/R_pc, chốt reset P1 (S_RST 1NO+1NC, K3, K2, U4),
M2 (POR giữ /CLR của U4), cầu chì nhánh, D1 + Zener 15 V, diode chặn D8 cho nhánh K4
(xả âm K1_FEED), LED3 có diode chặn ngược.
Mỗi linh kiện là template tự khai báo chân + loại chân, nên ERC của SKiDL chạy được.
Chạy: python pcb_nguon.py  ->  pcb-nguon.net, pcb_nguon.erc, pcb_nguon.log
Sau đó: python xuat_kicad.py -> pcb_nguon.kicad_sch + pcb_nguon.kicad_pcb (nháp).
v0.1.1 (21/11/2026): đóng N1 (LATCH_SENSE tách khỏi /CLR U4: U7 đệm một chiều), N3 (F10, F11 cầu chì nhánh K4/R_pc và TVS1),
N4 (Q1 ≥ 60 V, Q5 cùng mã), N5 (J1->J9 và SERVO_PRE->RS1->F3/F4/F5 đi busbar/dây ngoài PCB), N7 (R20 470k), D5 (P+ -> F1 -> D0).
CHƯA được duyệt: xem ../REVIEW.md. Không đặt in PCB khi chưa có chữ ký."""
import os
from skidl import KICAD, Part, Pin, Net, SKIDL, TEMPLATE, ERC, generate_netlist, set_default_tool, POWER

os.chdir(os.path.dirname(os.path.abspath(__file__)))
set_default_tool(SKIDL)
T = Pin.types

def tpl(name, pins, fp, val=""):
    return Part(name=name, tool=SKIDL, dest=TEMPLATE, footprint=fp, value=val,
                pins=[Pin(num=n, name=nm, func=f) for n, nm, f in pins])

P = T.PASSIVE
# ---- template linh kiện ----
BOLT2 = tpl("BOLT_M6_2P", [(1, "POS", T.PWRIN), (2, "NEG", T.PWRIN)], "Terminal_M6_2P", "cực bắt bu-lông M6 (cos 2 AWG, trục chính)")
BOLT_ARM = tpl("BOLT_M6_ARM", [(1, "POS", P), (2, "NEG", P)], "Terminal_M6_2P", "cực bắt bu-lông M6 (cos 4 AWG, nhánh tay, D9)")
BOLT_K1 = tpl("K1_PWR", [(1, "K1_IN", P), (2, "K1_OUT", P)], "Terminal_M6_2P", "cực M6 tới contactor K1")
XT60 = tpl("XT60PW", [(1, "POS", P), (2, "NEG", P)], "XT60PW-M", "XT60PW-M")
XT30 = tpl("XT30PW", [(1, "POS", P), (2, "NEG", P)], "XT30PW-M", "XT30PW-M")
SCREW2 = tpl("SCREW2", [(1, "P1", P), (2, "P2", P)], "TerminalBlock_2P_5.08", "")
RELAY_DPDT = tpl("RELAY_DPDT", [(1, "COIL_P", P), (2, "COIL_N", P), (3, "COM1", P), (4, "NO1", P),
                                (5, "COM2", P), (6, "NO2", P)], "Relay_DPDT_G2R", "Omron G2R-2-DC12")
RELAY_SPST = tpl("RELAY_SPST", [(1, "COIL_P", P), (2, "COIL_N", P), (3, "COM", P), (4, "NO", P)],
                 "Relay_SPST_G2R", "SPST-NO 12 VDC")
FUSEH = tpl("FUSE_PTC", [(1, "A", P), (2, "B", P)], "Fuse_1812", "1.1 A hold")
FUSE = tpl("FUSE", [(1, "A", P), (2, "B", P)], "Fuse_Holder", "")
BTN4 = tpl("BTN_1NO1NC", [(1, "NO_A", P), (2, "NO_B", P), (3, "NC_A", P), (4, "NC_B", P)],
           "TerminalBlock_4P_5.08", "nút Reset 22 mm 1NO+1NC (NC liên kết gương)")
ESTOP4 = tpl("ESTOP_2NC", [(1, "NC1_A", P), (2, "NC1_B", P), (3, "NC2_A", P), (4, "NC2_B", P)],
             "TerminalBlock_4P_5.08", "E-stop 2NC mở trực tiếp")
R = tpl("R", [(1, "1", P), (2, "2", P)], "R_0603")
RP = tpl("R_PWR", [(1, "1", P), (2, "2", P)], "R_Power_5W")
C = tpl("C", [(1, "1", P), (2, "2", P)], "C_0805")
CP = tpl("CP", [(1, "+", P), (2, "-", P)], "CP_Radial")
L = tpl("L", [(1, "1", P), (2, "2", P)], "L_Bourns_SRP1245")
D = tpl("D", [(1, "K", P), (2, "A", P)], "D_SMA")
ZENER = tpl("ZENER", [(1, "K", P), (2, "A", P)], "D_SOD-123")
TVS = tpl("TVS", [(1, "K", P), (2, "A", P)], "D_SMC")
LED = tpl("LED", [(1, "K", P), (2, "A", P)], "LED_0805")
SHUNT = tpl("SHUNT4", [(1, "F_IN", P), (2, "F_OUT", P), (3, "S_P", P), (4, "S_N", P)], "Shunt_4T_2512")
NMOS = tpl("NMOS", [(1, "G", T.INPUT), (2, "S", P), (3, "D", P)], "SOT-23", "AO3400A")
# INA226, VSSOP-10, số chân theo datasheet SBOS547C (tra 22/11/2026)
INA = tpl("INA226", [(1, "A1", T.INPUT), (2, "A0", T.INPUT), (3, "ALERT", T.OPENCOLL),
                     (4, "SDA", T.BIDIR), (5, "SCL", T.INPUT), (6, "VS", T.PWRIN),
                     (7, "GND", T.PWRIN), (8, "VBUS", T.INPUT), (9, "IN-", T.INPUT), (10, "IN+", T.INPUT)],
          "VSSOP-10", "INA226AIDGSR")
# TPS54560B, HSOIC-8 PowerPAD, số chân theo SLVSF00 (tra 22/11/2026)
BUCK = tpl("TPS54560B", [(1, "BOOT", P), (2, "VIN", T.PWRIN), (3, "EN", T.INPUT),
                         (4, "RT_CLK", P), (5, "FB", P), (6, "COMP", P),
                         (7, "GND", T.PWRIN), (8, "SW", P), (9, "PAD", T.PWRIN)],
           "HSOIC-8_PowerPAD", "TPS54560BDDA")
# 74LVC1G74: số chân theo ký hiệu 74AUC1G74 của thư viện KiCad 9 (cùng pinout họ 1G74), chưa đối chiếu datasheet TI
FF74 = tpl("74LVC1G74", [(1, "CLK", T.INPUT), (2, "D", T.INPUT), (3, "QN", T.OUTPUT), (4, "GND", T.PWRIN),
                         (5, "Q", T.OUTPUT), (6, "CLR_N", T.INPUT), (7, "PRE_N", T.INPUT), (8, "VCC", T.PWRIN)],
           "VSSOP-8", "74LVC1G74")
# 74AHCT1G125 (SOT-23-5, TI SCLS378P 03/2024: VCC 4,5-5,5 V, VIH min 2 V, IOH -8 mA): đệm cổng Q1 cấp 5 V; số chân 1 OE, 2 A, 3 GND, 4 Y, 5 VCC theo gói DBV [chưa đối chiếu hình pinout]
BUF5 = tpl("74AHCT1G125", [(1, "OE_N", T.INPUT), (2, "A", T.INPUT), (3, "GND", T.PWRIN), (4, "Y", T.OUTPUT), (5, "VCC", T.PWRIN)],
           "SOT-23-5", "74AHCT1G125")
# 74LVC1G08 (1 A, 2 B, 3 GND, 4 Y, 5 VCC) và 74LVC1G17 (2 A, 3 GND, 4 Y, 5 VCC): thư viện KiCad 9
AND2 = tpl("74LVC1G08", [(1, "A", T.INPUT), (2, "B", T.INPUT), (3, "GND", T.PWRIN), (4, "Y", T.OUTPUT),
                         (5, "VCC", T.PWRIN)], "SOT-23-5", "74LVC1G08")
SCHM = tpl("74LVC1G17", [(2, "A", T.INPUT), (3, "GND", T.PWRIN), (4, "Y", T.OUTPUT), (5, "VCC", T.PWRIN)],
           "SOT-23-5", "74LVC1G17")
HDR = lambda n, nm: tpl(nm, [(i + 1, f"P{i+1}", P) for i in range(n)], f"PinHeader_1x{n:02d}")

def two(t, ref, val, a, b):
    p = t(ref=ref, value=val); p[1] += a; p[2] += b; return p
def r(ref, val, a, b): return two(R, ref, val, a, b)
def c(ref, val, a, b): return two(C, ref, val, a, b)
def dio(ref, val, a, k):
    p = D(ref=ref, value=val); p["A"] += a; p["K"] += k; return p

# ---- net ----
GND = Net("GND"); GND.drive = POWER
BAT_F = Net("BAT_F"); BAT_F.drive = POWER          # sau F1 MEGA 60 A rồi D0 (P+ -> F1 -> D0), trước contactor; đoạn J1->J9 đi busbar ngoài PCB (N5)
SERVO_PRE = Net("SERVO_PRE")                       # sau contactor K1, trước shunt
SERVO_12V = Net("SERVO_12V")                       # bus servo 12 V sau shunt
BUCK_F = Net("BUCK_F")                             # sau F6 7,5 A
VIN_BUCK = Net("VIN_BUCK"); VIN_BUCK.drive = POWER
V5_PI = Net("V5_PI"); V5_PI.drive = POWER
V33_PI = Net("V33_PI"); V33_PI.drive = POWER       # 3V3 từ header Pi
ESTOP_A, ESTOP_OK = Net("ESTOP_A"), Net("ESTOP_OK")
K1_FEED, COIL_N, COIL_RET = Net("K1_FEED"), Net("COIL_N"), Net("COIL_RET")
K2_HOLD, RST_NC, NO_B, ARM = Net("K2_HOLD"), Net("RST_NC"), Net("NO_B"), Net("ARM")
SDA, SCL = Net("I2C_SDA"), Net("I2C_SCL")
RELAY_EN, RUN = Net("RELAY_EN"), Net("RUN")
ESTOP_SENSE, LATCH_SENSE, ARM_SENSE, PRECHARGE_OK = (Net(n) for n in
    ("ESTOP_SENSE", "LATCH_SENSE", "ARM_SENSE", "PRECHARGE_OK"))
LATCH_CLR = Net("LATCH_CLR")   # N1: nhánh nội bộ chỉ tới U6.A; LATCH_SENSE (ra Pi) lấy qua đệm một chiều U7
ALERT1, ALERT2 = Net("ALERT_SERVO"), Net("ALERT_PI")
G_Q1, Q4G, Q4D, K4_FEED, PC_MID = Net("G_Q1"), Net("Q4G"), Net("Q4D"), Net("K4_FEED"), Net("PC_MID")

# ---- đầu vào pin (sau F1 MEGA 80 A ngoài PCB; dây trục 2 AWG, cos M6; D9) ----
J1 = BOLT2(ref="J1"); J1[1] += BAT_F; J1[2] += GND
# N3: TVS1 có cầu chì nhánh F11 riêng (TVS chập không kéo cả F1 60 A)
F11 = FUSE(ref="F11", value="ATOF 5 A 32 V (TVS1)"); TVS1_F = Net("TVS1_F"); F11["A"] += BAT_F; F11["B"] += TVS1_F
TVS1 = TVS(ref="TVS1", value="SMCJ14A"); TVS1["K"] += TVS1_F; TVS1["A"] += GND

# ---- contactor K1 200 A (ngoài PCB, dây W16) + nạp trước K4/R_pc ----
J9 = BOLT_K1(ref="J9"); J9[1] += BAT_F; J9[2] += SERVO_PRE
K4 = RELAY_SPST(ref="K4", value="SPST-NO 12 V (nạp trước)")
# N3: nhánh K4/R_pc có cầu chì F10 riêng (vệt đồng mỏng sau F10)
F10 = FUSE(ref="F10", value="ATOF 2 A 32 V (K4/R_pc)"); K4_IN = Net("K4_IN"); F10["A"] += BAT_F; F10["B"] += K4_IN
K4["COM"] += K4_IN; K4["NO"] += PC_MID
RPC = RP(ref="RPC", value="10R 5W (nạp trước)"); RPC[1] += PC_MID; RPC[2] += SERVO_PRE

# ---- vòng E-stop: BAT_F -> F2 -> J8 NC1 -> ESTOP_OK ----
F2 = FUSEH(ref="F2", value="PTC 1.1 A hold"); F2["A"] += BAT_F; F2["B"] += ESTOP_A
J8 = ESTOP4(ref="J8"); J8["NC1_A"] += ESTOP_A; J8["NC1_B"] += ESTOP_OK
J8["NC2_A"] += COIL_N; J8["NC2_B"] += COIL_RET        # kênh 2 ở đường về cuộn K1 (giảm chập J8)

# ---- chốt reset P1: S_RST 1NO+1NC, K3 giám sát, K2 tự giữ ----
J11 = BTN4(ref="J11")
J11["NO_A"] += ESTOP_OK; J11["NO_B"] += NO_B; J11["NC_A"] += ESTOP_OK; J11["NC_B"] += RST_NC
D4 = dio("D4", "SS14", NO_B, ARM)
CK3 = CP(ref="C14", value="220u 25V (C_K3)"); CK3["+"] += ARM; CK3["-"] += GND
K3 = RELAY_SPST(ref="K3", value="SPST-NO 12 V (giám sát)")
K3["COIL_P"] += ARM; K3["COIL_N"] += GND; K3["COM"] += RST_NC; K3["NO"] += K2_HOLD
K2 = RELAY_DPDT(ref="K2")
K2["COIL_P"] += K2_HOLD; K2["COIL_N"] += GND
K2["COM1"] += ESTOP_OK; K2["NO1"] += K2_HOLD          # pole 1: tự giữ
K2["COM2"] += ESTOP_OK; K2["NO2"] += K1_FEED          # pole 2: cấp cuộn K1
D3 = dio("D3", "SS14", GND, Net("ZM3")); Z3 = ZENER(ref="Z3", value="15V"); Z3["K"] += D3["K"]; Z3["A"] += K2_HOLD

# ---- cuộn K1 (contactor ngoài): K1_FEED -> J10 -> COIL_N -> NC2 -> COIL_RET -> Q1 ----
J10 = SCREW2(ref="J10", value="cuộn K1 contactor"); J10[1] += K1_FEED; J10[2] += COIL_N
Q1 = NMOS(ref="Q1", value="IRLML0060TRPBF", part="IRLML0060TRPBF")  # D8: Vth max 2,5 V, chưa đạt 2,0 V, chờ đo EVT
Q1["D"] += COIL_RET; Q1["S"] += GND; Q1["G"] += G_Q1
D1 = dio("D1", "SS14", COIL_N, Net("ZM1")); Z1 = ZENER(ref="Z1", value="15V >=1W/xung 5 W"); Z1["K"] += D1["K"]; Z1["A"] += K1_FEED
# RC trễ đóng K1 ~0,3 s sau khi K4 đóng; D5 xả nhanh khi RUN xuống
# D8 vòng 2: RC trễ ở nút K1_DLY (miền 3V3, tối đa 3,0 V) -> Schmitt U8 74LVC1G17 (cấp 3V3_PI cùng nguồn U4/RUN)
# -> đệm U9 74AHCT1G125 cấp V5_PI (VIH 2 V nhận mức 3,3 V) -> G_Q1 = 5 V (Vgs Q1 >= 4,5 V, ở đó Rds(on) mới có số). Dừng E-stop không đi qua đây.
K1_DLY = Net("K1_DLY"); U8_Y = Net("U8_Y")
r("R1", "47k", RUN, K1_DLY); r("R2", "470k", K1_DLY, GND); c("C13", "10u 10V", K1_DLY, GND)
D5 = dio("D5", "BAT54", K1_DLY, RUN)
U8 = SCHM(ref="U8"); U8["A"] += K1_DLY; U8["VCC"] += V33_PI; U8["GND"] += GND; U8["Y"] += U8_Y
U9 = BUF5(ref="U9"); U9["A"] += U8_Y; U9["OE_N"] += GND; U9["VCC"] += V5_PI; U9["GND"] += GND; U9["Y"] += G_Q1
r("R40", "100k", G_Q1, GND)           # giữ cổng thấp khi U9 chưa có nguồn
r("R41", "100k", U8_Y, GND)           # QC M1: giữ U9.A thấp khi V33_PI mất mà V5_PI còn (U8 mất nguồn, đầu ra hở)
c("C19", "0.1u", V33_PI, GND); c("C20", "0.1u", V5_PI, GND)
# K4: Q4 đóng cùng RUN; Q5 tắt Q4 khi G_Q1 lên (K1 bắt đầu hút)
Q4 = NMOS(ref="Q4", value="AO3400A"); Q4["D"] += Q4D; Q4["S"] += GND; Q4["G"] += Q4G
Q5 = NMOS(ref="Q5", value="IRLML0060TRPBF", part="IRLML0060TRPBF"); Q5["D"] += Q4G; Q5["S"] += GND; Q5["G"] += G_Q1
r("R11", "1k", RUN, Q4G); r("R12", "100k", Q4G, GND)
D8 = dio("D8", "SS14 (chặn xả âm K1_FEED)", K1_FEED, K4_FEED)
K4["COIL_P"] += K4_FEED; K4["COIL_N"] += Q4D
D6 = dio("D6", "SS14", Q4D, K4_FEED)
r("R22", "1k", K1_DLY, PRECHARGE_OK)   # D8: chuyển từ G_Q1 (nay 5 V) sang K1_DLY (<= 3,0 V) để không đẩy 5 V vào chân Pi

# ---- cảm biến mức cho Pi: chia áp 100k/33k + Zener 3,3 V + 1k nối tiếp ----
def sense(i, src, dst, tag):
    mid = Net(f"{tag}_DIV")
    r(f"R{i}", "100k", src, mid); r(f"R{i+1}", "33k", mid, GND)
    z = ZENER(ref=f"ZD{tag}", value="3V3"); z["K"] += mid; z["A"] += GND
    r(f"R{i+2}", "1k", mid, dst)
    return mid
sense(30, ESTOP_OK, ESTOP_SENSE, "ES")
LA_MID = sense(33, K1_FEED, LATCH_CLR, "LA")
# N1: Pi chỉ ĐỌC. U7 (74LVC1G17, đệm một chiều, cấp 3V3_PI) lấy mức từ LA_MID, ra qua R39 1k tới LATCH_SENSE -> J6.P7.
# Chân Pi dù cấu hình output hay pull-up chỉ đẩy vào đầu ra U7, không có đường tới LATCH_CLR (/CLR của U4).
U7 = SCHM(ref="U7"); U7["A"] += LA_MID; U7["VCC"] += V33_PI; U7["GND"] += GND; U7_Y = Net("U7_Y"); U7["Y"] += U7_Y
r("R39", "1k", U7_Y, LATCH_SENSE)
sense(36, ARM, ARM_SENSE, "AR")

# ---- U4 chốt cạnh RELAY_EN + M2: POR giữ /CLR thấp ~0,2-0,3 s sau khi 3V3 lên ----
U4 = FF74(ref="U4"); U5 = SCHM(ref="U5"); U6 = AND2(ref="U6")
CLR_N, POR_RC, POR_OK = Net("CLR_N"), Net("POR_RC"), Net("POR_OK")
U4["CLK"] += RELAY_EN; U4["D"] += V33_PI; U4["PRE_N"] += V33_PI; U4["CLR_N"] += CLR_N
U4["Q"] += RUN; U4["QN"] += Net("NC_U4_QN"); U4["VCC"] += V33_PI; U4["GND"] += GND
r("R20", "470k", V33_PI, POR_RC)   # N7: 470k x 1u, cận dưới có dung sai R 1%, C -20%: >= 200 ms
c("C15", "1u", POR_RC, GND)
D9 = dio("D9", "BAT54 (xả C15 khi 3V3 sụt)", POR_RC, V33_PI)
U5["A"] += POR_RC; U5["Y"] += POR_OK; U5["VCC"] += V33_PI; U5["GND"] += GND
U6["A"] += LATCH_CLR; U6["B"] += POR_OK; U6["Y"] += CLR_N; U6["VCC"] += V33_PI; U6["GND"] += GND
r("R21", "100k", RELAY_EN, GND)
for ref in ("C16", "C17", "C18"): c(ref, "0.1u", V33_PI, GND)

# ---- đo dòng bus servo: shunt 1 mOhm + INA226 U1 (0x40) + tụ + TVS + 3 nhánh có cầu chì ----
# N5: RS1 và F3/F4/F5 nằm trên busbar/dây ngoài PCB (SERVO_PRE -> RS1 -> F3/F4/F5); PCB chỉ nhận 2 dây sense Kelvin
RS1 = SHUNT(ref="RS1", value="1mR >=20W 4T (ngoài PCB, mã chưa có số liệu)")
RS1["F_IN"] += SERVO_PRE; RS1["F_OUT"] += SERVO_12V
RS1["S_P"] += SERVO_PRE; RS1["S_N"] += SERVO_12V
U1 = INA(ref="U1"); U1["IN+"] += SERVO_PRE; U1["IN-"] += SERVO_12V; U1["VBUS"] += SERVO_12V
U1["VS"] += V33_PI; U1["GND"] += GND; U1["SDA"] += SDA; U1["SCL"] += SCL
U1["A0"] += GND; U1["A1"] += GND; U1["ALERT"] += ALERT1            # 0x40
CB1 = CP(ref="C1", value="470u 25V"); CB1["+"] += SERVO_12V; CB1["-"] += GND
TVS2 = TVS(ref="TVS2", value="SMCJ14A"); TVS2["K"] += SERVO_12V; TVS2["A"] += GND
for ref, fref, fv, JT in [("J2", "F3", "MEGA 298 60 A", BOLT_ARM), ("J3", "F4", "MEGA 298 60 A", BOLT_ARM), ("J4", "F5", "MAXI 20 A", XT60)]:
    f = FUSE(ref=fref, value=fv); br = Net(f"{ref}_P")
    f["A"] += SERVO_12V; f["B"] += br
    j = JT(ref=ref); j[1] += br; j[2] += GND   # D9: nhánh tay 50 A chặn, 4 AWG không vào XT60 -> cực M6

# ---- nhánh Pi: F6 7,5 A + shunt 20 mOhm + INA226 U2 (0x41) + buck 5,1 V 5 A ----
F6 = FUSE(ref="F6", value="ATOF 7.5 A 32 V"); F6["A"] += BAT_F; F6["B"] += BUCK_F
RS2 = SHUNT(ref="RS2", value="20mR 1W 4T")
RS2["F_IN"] += BUCK_F; RS2["F_OUT"] += VIN_BUCK; RS2["S_P"] += BUCK_F; RS2["S_N"] += VIN_BUCK
U2 = INA(ref="U2"); U2["IN+"] += BUCK_F; U2["IN-"] += VIN_BUCK; U2["VBUS"] += VIN_BUCK
U2["VS"] += V33_PI; U2["GND"] += GND; U2["SDA"] += SDA; U2["SCL"] += SCL
U2["A0"] += V33_PI; U2["A1"] += GND; U2["ALERT"] += ALERT2          # 0x41
# buck theo ví dụ TI SLVSF00 mục 8.2 (5 V, 5 A, 400 kHz), R_bot đổi 10,2k -> 10,0k để ra ~5,09 V
U3 = BUCK(ref="U3")
U3["VIN"] += VIN_BUCK; U3["GND"] += GND; U3["PAD"] += GND
SW, BOOT, FB, COMP, RT = Net("SW"), Net("BOOT"), Net("FB"), Net("COMP"), Net("RT")
U3["SW"] += SW; U3["BOOT"] += BOOT; U3["FB"] += FB; U3["COMP"] += COMP; U3["RT_CLK"] += RT
# EN: chia áp 442k/90.9k từ VIN (EN tối đa 8,4 V theo SLVSF00), KHÔNG nối thẳng VIN
r("R5", "442k", VIN_BUCK, U3["EN"]); r("R6", "90.9k", U3["EN"], GND)       # UVLO bật ~6,5 V theo ví dụ TI
for ref in ("C2", "C3", "C4", "C5"): c(ref, "2.2u 25V", VIN_BUCK, GND)
CBulk = CP(ref="C6", value="100u 25V"); CBulk["+"] += VIN_BUCK; CBulk["-"] += GND
c("C7", "0.1u 50V", BOOT, SW)
L1 = L(ref="L1", value="6.8u >=8A Isat"); L1[1] += SW; L1[2] += V5_PI
D2 = D(ref="D2", value="B560C-13-F"); D2["K"] += SW; D2["A"] += GND
for ref in ("C8", "C9", "C10"): c(ref, "47u 10V", V5_PI, GND)
r("R7", "243k", RT, GND); r("R8", "53.6k", V5_PI, FB); r("R9", "10.0k", FB, GND)
CCn = Net("CC_N"); r("R10", "16.9k", COMP, CCn); c("C11", "4700p", CCn, GND); c("C12", "47p", COMP, GND)
J5 = XT30(ref="J5"); J5[1] += V5_PI; J5[2] += GND

# ---- LED trạng thái (LED3 có chặn ngược vì ESTOP_OK xuống âm lúc xả cuộn K1) ----
for ref, net, val in [("LED1", SERVO_12V, "2.2k"), ("LED2", V5_PI, "1k")]:
    rr = R(ref="R" + ref[-1] + "L", value=val); l = LED(ref=ref)
    rr[1] += net; rr[2] += l["A"]; l["K"] += GND
LA3 = Net("LED3_A"); LB3 = Net("LED3_B")
r("R3L", "3.3k", ESTOP_OK, LA3); dio("", "1N4148W", LA3, LB3)
l3 = LED(ref="LED3"); l3["A"] += LB3; l3["K"] += GND

# ---- header tới Pi 5 (9 chân) ----
J6 = HDR(9, "HDR_PI_CTRL")(ref="J6")   # 1 3V3, 2 SDA, 3 SCL, 4 GND, 5 RELAY_EN, 6 ESTOP_SENSE, 7 LATCH_SENSE, 8 ARM_SENSE, 9 PRECHARGE_OK
for i, n in enumerate([V33_PI, SDA, SCL, GND, RELAY_EN, ESTOP_SENSE, LATCH_SENSE, ARM_SENSE, PRECHARGE_OK]):   # LATCH_SENSE qua đệm U7
    J6[i + 1] += n
J7 = HDR(2, "HDR_ALERT")(ref="J7"); J7[1] += ALERT1; J7[2] += ALERT2

if __name__ == "__main__":
    ERC()
    generate_netlist(file_="pcb-nguon.net", tool=KICAD)


# Kiểm nhất quán: mọi ref trong bom-pcb.csv phải có trong netlist (bắt lỗi kiểu C15 bị chú thích mất)
def _kiem_bom_netlist():
    import csv, os, re
    here = os.path.dirname(os.path.abspath(__file__))
    bom = os.path.join(here, "..", "bom-pcb.csv")
    refs_net = {p.ref for p in default_circuit.parts}
    refs_bom = set()
    for row in csv.DictReader(open(bom, encoding="utf-8")):
        if "ngoai PCB" in row["ref"]:
            continue   # linh kiện ngoài PCB (vd contactor K1), chỉ nối qua đầu cắm
        for x in re.split(r"[\s,]+", row["ref"].strip()):
            if re.fullmatch(r"[A-Z]+[0-9]+", x):
                refs_bom.add(x)
    thieu = sorted(refs_bom - refs_net)
    if thieu:
        raise SystemExit("BOM có ref không có trong netlist: " + ", ".join(thieu))
    print("BOM-netlist: khớp", len(refs_bom), "ref")

if __name__ == "__main__":
    _kiem_bom_netlist()
