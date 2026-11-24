"""Ve so-do-khoi va di-day (SVG+PNG) bang matplotlib cho chuoi nguon L v0.5 (cau hinh P-L, chot 23/11; S2 24/11: tram hai tay, giac 21W4). Chay: python ve_so_do.py
So lieu dong, chieu dai, cau chi lay tu bang-day.csv (sinh tu day_L.py qua tinh_toan.py); chan GPIO chot truc nang theo sdk/faibotic/gpio_pi.py (LATCH_DEFAULT_BCM).
Ban v0.3 (PCB nguon) va chuoi B o lich su git."""
import csv, os, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
os.chdir(os.path.dirname(os.path.abspath(__file__)))
COL = dict(p12="#c0392b", p5="#e67e22", sig="#2471a3", es="#7d3c98", box="#f4f6f7", edge="#2c3e50", lat="#16a085")
W = {r["ma"]: r for r in csv.DictReader(open("bang-day.csv", encoding="utf-8"))}
F = {r["cau_chi"]: r["cau_chi_A"] for r in W.values()}


def canvas(w, h, title):
    fig, ax = plt.subplots(figsize=(w, h)); ax.set_xlim(0, w*10); ax.set_ylim(0, h*10); ax.axis("off")
    ax.text(w*5, h*10-2, title, ha="center", va="top", fontsize=13, weight="bold")
    return fig, ax
def box(ax, x, y, w, h, t, fc=None, fs=8):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4", fc=fc or COL["box"], ec=COL["edge"], lw=1.2))
    ax.text(x+w/2, y+h/2, t, ha="center", va="center", fontsize=fs)
def arr(ax, p, q, c, ls="-", lw=2.2, t=None, off=(0, 1.2)):
    ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle="-|>", color=c, lw=lw, ls=ls))
    if t: ax.text((p[0]+q[0])/2+off[0], (p[1]+q[1])/2+off[1], t, fontsize=6.5, color=c, ha="center")
def legend(ax, x, y):
    for i, (k, t) in enumerate([("p12", "12 V pin/servo"), ("p5", "5 V Pi"), ("es", "mach cuon K1 + E-stop 2NC + Reset"),
                                ("lat", "chot truc nang (cuon tu SERVO+)"), ("sig", "GPIO / tin hieu")]):
        ax.plot([x, x+6], [y-i*3, y-i*3], color=COL[k], lw=2.5); ax.text(x+7, y-i*3, t, fontsize=7, va="center")
def save(fig, n):
    for e in ("svg", "png"): fig.savefig(f"{n}.{e}", dpi=160, bbox_inches="tight")
    plt.close(fig)
def a(m):   # dong lien tuc / dinh (P-L)
    return f"{W[m]['dong_lien_tuc_A']} A / {W[m]['dong_dinh_A']} A"
def L(m):   # dai duong di (m)
    return f"{int(W[m]['dai_mm'])/1000:.2f} m".replace(".", ",")

# ===== so do khoi =====
fig, ax = canvas(16, 10, "M1 v0.5 P-L + tram hai tay: so do khoi nguon chuoi L (chua duyet; can ky su dien xac nhan)")
box(ax, 3, 72, 22, 14, "Pack 3S4P 40T\n12 cell, 172,8 Wh\n+ BMS Daly 100 A", fs=7.5)
box(ax, 30, 72, 16, 14, "F1 MEGA 80 A\nsat cuc + pack\n(<= 10 cm)", fs=7.5)
box(ax, 51, 72, 18, 14, "J0 Anderson SB120\ncat ca hai cuc\n(thay D0)", fs=7.5)
box(ax, 74, 72, 18, 14, "BAT_F\ncoc M8", fs=7.5)
box(ax, 100, 72, 24, 14, "K1 contactor 100 A\ncuon 12 V (kieu SW80)\ndatasheet: DK-N1", fs=7.5)
box(ax, 130, 72, 22, 14, "SERVO+\ncoc M8", fs=7.5)
box(ax, 3, 52, 24, 10, f"Sac 3S 12,6 V, XT60\naptomat DC C{F['FCHG']} 6 kA tai BAT_F", fs=7)
box(ax, 34, 50, 40, 16, f"Mach dung (+12V_CTRL, aptomat DC C{F['FCTRL']} 6 kA)\nvong tram (21W4 chan 5-6): 2 khoa + E-stop tram\nnoi tiep E-stop dinh cot trong mach cuon K1\nReset, ro-le tu giu KH, K5 (Pi cho phep)", fs=6.8)
box(ax, 80, 50, 26, 12, f"Buck 5 V TPS54560 5 A\naptomat DC C{F['FPI']} 6 kA -> Pi 5 USB-C", fs=7)
box(ax, 112, 50, 40, 14, f"F3/F4 MEGA {F['F3']} A -> 4 AWG -> duoi 8 AWG\n-> giac 21W4 (4 x 40 A) -> FB-L/FB-R tren tram\nFW1-3 {F['FW1']} A, FLIFT {F['FLIFT']} A, FLAT {F['FLAT']} A inline\nSTS3095 {F['FL1']} A, STS3215 {F['FL4']} A (1 kA: DK-N29)", fs=6.6)
box(ax, 112, 26, 40, 16, f"Truc nang (xe nang): servo BUS_B cong S2\nFLIFT 16 AWG qua vong dich vu\nchot: FLAT ({F['FLAT']} A) -> ro-le K10 (tam dien) -> cuon\nK1 nha => cuon mat dien => chot cai", fc="#e8f8f5", fs=6.8)
box(ax, 56, 22, 44, 18, "Raspberry Pi 5 (SDK)\nK5_EN BCM 22, ESTOP_OK 17, ARM_SENSE 27, RESET_SENSE 23\nLIFT_BOT 5, LIFT_TOP 6, LATCH_EN 24, LATCH_RETRACTED 25\nDOCK_L 16, DOCK_R 26 (de xuat)\nTORQUE_LIMIT moi servo, tong <= 80 A", fs=6.6)
box(ax, 3, 26, 40, 14, f"Servo P-L (AM-ARM200 nguyen, 6+1):\n7 STS3095 + 11 STS3215, 3 bo bus\nchan moi tay {W['BW07']['dong_dinh_A']} A; dinh truc chinh {W['BW01']['dong_dinh_A']} A\nTram: hub 7 cong, 3 camera, 12V_AUX C{F['FAUX']}", fs=6.8)
arr(ax, (25, 79), (30, 79), COL["p12"], t="BW01", off=(0, 1.5))
arr(ax, (46, 79), (51, 79), COL["p12"], t="BW02", off=(0, 1.5))
arr(ax, (69, 79), (74, 79), COL["p12"], t="BW03", off=(0, 1.5))
arr(ax, (92, 79), (100, 79), COL["p12"], t="BW05", off=(0, 1.5))
arr(ax, (124, 79), (130, 79), COL["p12"], t="BW06", off=(0, 1.5))
arr(ax, (76, 72), (14, 62), COL["p12"], t="CH01", off=(0, 1.5))
arr(ax, (80, 72), (60, 66), COL["p12"], t="FCTRL", off=(3, 1))
arr(ax, (86, 72), (90, 62), COL["p12"], t="PW01", off=(5, 0))
arr(ax, (141, 72), (135, 64), COL["p12"], t="BW07/BW08", off=(7, 0))
arr(ax, (148, 50), (140, 42), COL["lat"], t="P.LIFT, L01-L04", off=(8, 0))
arr(ax, (60, 66), (106, 73), COL["es"], ls="--", t="C13 cuon K1", off=(0, 2))
arr(ax, (93, 50), (85, 40), COL["p5"], t="PW03 5,1 V", off=(6, 0))
arr(ax, (100, 33), (112, 33), COL["sig"], t="G13 / G15", off=(0, 1.5))
arr(ax, (56, 30), (43, 32), COL["sig"], t="USB qua 21W4 -> hub tram, ADuM3160", off=(0, 1.5))
arr(ax, (70, 50), (72, 40), COL["sig"], ls=":", t="opto", off=(-4, 0))
ax.text(80, 8, "Thu tu +pack -> F1 -> J0. Pi, 12V_AUX lay tu BAT_F (truoc K1): con dien khi E-stop. Lift, cuon chot sau K1: E-stop cat cung servo.\n"
        "Contactor va BMS chua co datasheet (DK-N1, DK-N2). Dong K1 khong nap truoc (DK-N30). So dong: tinh-toan.md phan L; tung soi: bang-day.csv.", fontsize=7, ha="center")
legend(ax, 3, 18)
save(fig, "so-do-khoi")

# ===== di day =====
fig, ax = canvas(18.4, 10.5, f"M1 v0.5 P-L: so do di day chuoi L ({len(W)} soi trong bang-day.csv; AWG, dai, dong lien tuc / dinh)")
box(ax, 2, 80, 16, 16, "Pack 3S4P\n40T + BMS 100 A\n(+ / P-)", fs=8)
box(ax, 22, 82, 16, 12, f"F1 MEGA 80 A\nBW01 2 AWG {L('BW01')}\n{a('BW01')}", fs=6.6)
box(ax, 42, 82, 16, 12, f"J0 SB120 (2 cuc)\nBW02 / BW04 2 AWG\n{L('BW02')}", fs=6.6)
box(ax, 62, 82, 16, 12, f"BAT_F\nBW03 2 AWG {L('BW03')}", fs=6.8)
box(ax, 84, 82, 22, 12, f"K1 100 A\nBW05/06 2 AWG {L('BW05')}\n{a('BW05')}", fs=6.6)
box(ax, 112, 82, 18, 12, "SERVO+\nBW07A/08A 2 AWG", fs=7)
box(ax, 136, 82, 36, 13, f"F3/F4 {F['F3']} A -> BW07/BW08 4 AWG {L('BW07')} (vong nang)\n-> duoi 8 AWG {L('BW07T')} -> 21W4 -> 8 AWG {L('BW07S')} -> FB\nmoi tay {a('BW07')}\nSTS3095 16 AWG {F['FL1']} A; STS3215 22 AWG {F['FL4']} A", fs=6.0)
box(ax, 136, 64, 36, 12, f"Banh: FW1-3 {F['FW1']} A inline tren SERVO+\nP.W1-3 22 AWG {L('P.W1+')}\nTruc nang: FLIFT {F['FLIFT']} A, 16 AWG {L('P.LIFT+')}", fs=6.2)
box(ax, 136, 44, 36, 14, f"Chot truc nang: FLAT ({F['FLAT']} A) tren SERVO+\nL01 -> ro-le K10 (tam dien) -> L03/L04 {L('L03')}\n1N5408 dap; cam bien KW12 G15 {L('G15')}", fc="#e8f8f5", fs=6.2)
box(ax, 62, 56, 30, 14, f"Mach dung (aptomat C{F['FCTRL']})\nE01/E09 vong tram {L('E01')}, E02-E06 tren tram\nC02/C04 E-stop {L('C02')}; C13/C14 cuon K1", fs=6.5)
box(ax, 40, 30, 30, 12, f"Buck TPS54560 (aptomat C{F['FPI']})\nPW03/PW04 USB-C {L('PW03')} -> Pi 5", fs=6.5)
box(ax, 96, 14, 40, 16, f"Raspberry Pi 5\nG13 BCM 24 -> ro-le K10 ({L('G13')})\nG15 BCM 25 <- cong tac chot (vong nang)\nG17/G20 DOCK_L/R BCM 16/26 <- opto (E10/E11)", fs=6.3)
box(ax, 2, 56, 22, 10, f"Sac 3S XT60\nCH01/CH02 14 AWG, aptomat C{F['FCHG']}", fs=6.8)
arr(ax, (18, 88), (22, 88), COL["p12"])
arr(ax, (38, 88), (42, 88), COL["p12"])
arr(ax, (58, 88), (62, 88), COL["p12"])
arr(ax, (78, 88), (84, 88), COL["p12"])
arr(ax, (106, 88), (112, 88), COL["p12"])
arr(ax, (130, 88), (136, 88), COL["p12"])
arr(ax, (126, 82), (136, 70), COL["p12"])
arr(ax, (173, 86), (173, 56), COL["lat"])
arr(ax, (70, 82), (75, 70), COL["p12"], t=f"C{F['FCTRL']}", off=(3, 0))
arr(ax, (64, 82), (58, 42), COL["p12"], t=f"C{F['FPI']}", off=(-3, 0))
arr(ax, (66, 82), (14, 66), COL["p12"], t=f"C{F['FCHG']}", off=(0, 2))
arr(ax, (92, 63), (92, 82), COL["es"], ls="--", t="cuon K1", off=(4, 0))
arr(ax, (70, 34), (96, 24), COL["p5"], t="5 V", off=(0, 2))
arr(ax, (136, 24), (150, 44), COL["sig"], t="G13/G15", off=(6, -2))
arr(ax, (10, 80), (60, 78), COL["p12"], t="BW00/BW04/BW04R duong ve (-) 2 AWG qua J0 -> coc GND", off=(-5, -2.5))
ax.text(2, 6, "Nguon servo hinh sao tu hop cau chi; cap 3 chan chi giu DATA di suot (V+/GND cat giua, noi vao nguon servo hai dau; DK-N33).\n"
        "Khong PCB nguon o v0.5. D0 bo (can ky su dien xac nhan); J0 SB120 khong rut khi co tai. Cau chi luoi 1 kA: DK-N29. Chua ky duyet.", fontsize=8)
legend(ax, 3, 46)
save(fig, "di-day")
