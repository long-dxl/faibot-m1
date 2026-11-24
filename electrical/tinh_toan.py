"""Tính dòng, dây, sụt áp, cầu chì, pack, contactor cho nguồn M1.
Phần L (v0.5, chốt 23/11/2026 tối: chuỗi L, cấu hình P-L; S2 24/11: trạm hai tay, giắc 21W4): khối GL ở cuối file, ghi bang-day.csv,
vat-tu-day.csv, bang-day-de-ban.csv, vat-tu-de-ban.csv và phần đầu tinh-toan.md; số servo
P-L và ngân sách dòng firmware đọc từ sdk/faibotic (spec.PL_SERVO_MODELS, limits.current_budget). Chuỗi B (ghép đôi/P-B) chỉ còn ghi chú.
Phần A (v0.3 cơ khí, tính lại 22/11/2026, D9; PCB nguồn, D0, EV200): khối GD, chỉ lưu đối chiếu: ghi vào archive/v0.4/electrical/
(bang-day-chuoi-a-v03.csv, tinh-toan-chuoi-A.md) khi thư mục đó có; tinh-toan.md chỉ còn phần L.
Chạy: python tinh_toan.py"""
import csv, math, os, re
os.chdir(os.path.dirname(os.path.abspath(__file__)))
ARCH = os.path.join("..", "archive", "v0.4", "electrical")   # phần A (bản cũ) chỉ ghi khi có thư mục lưu trữ
LUU_A = os.path.isdir(ARCH)

# ---------------- GIẢ ĐỊNH (đổi ở đây, chạy lại) ----------------
GD = dict(
  I3095=9.8, I3215=2.7, k=0.4,          # F016, trang bán 22/11/2026
  # Số servo theo nhánh (cơ khí v0.3: vai ghép đôi + khuỷu ghép đôi; sdk/faibotic/spec.py SERVO_IDS 20 ID, cad/bao-cao-cad.md v0.3)
  # mỗi tay: pan 1 STS3215 + vai 2 STS3095 + khuỷu 2 STS3095 + gập/xoay cổ tay/kẹp 3 STS3215 = 4 STS3095 + 4 STS3215
  arm3095=4, arm3215=4, base3215=3, lift3095=1, n_arm=2,
  CELL='40T',                 # '40T' (chính, D7) hoặc 'P45B' (dự phòng)
  AWG_T=2, F1=80,             # trục chính: dây 2 AWG, F1 MEGA 298 80 A (D9)
  AWG_ARM=4, F_ARM=60,        # nhánh tay: dây 4 AWG, F3/F4 MEGA 298 60 A (D9)
  BMS_A=200,                  # BMS 3S chọn (>= 160 A, D9)
  Ipi_max=2.6, Ipi_typ=1.2,   # A phía pin: 5,09 V x 5 A / 0,9 / 10,8 V; typ giả định
  Vnom=11.1, Vmin=9.0,
  rho=1.724e-8, strand=1.03, Thot=60, alpha=0.00393,
  k_cu_pvc=115,               # IEC 60364-4-43, đồng cách điện PVC 70->160 C [nhớ từ chuẩn, chưa kiểm URL]
  derate=0.82,                # hạ nhiệt dây trong vỏ kín 45 C [chưa kiểm NEC 310.15(B)], QC2 N1
  margin=1.25,                # biên yêu cầu cho BMS/pack
  n_par=5, n_ser=3, usable=0.85,   # dùng được 85% [giả định]; thông số cell ở CELLS
  R_fuse=2.0e-3, R_xt60=0.8e-3, R_contactor=3.0e-3, R_shunt=1.0e-3, R_pcb=0.7e-3,   # giả định, chưa datasheet
  R_bolt=0.3e-3,              # 1 mối cos M6 bắt bu-lông [giả định, chưa đo]; XT90 cũ 1,0 mOhm/điểm
  R_disc=0.5e-3,              # công tắc ngắt pin, tiếp điểm [giả định, chưa có số hãng]
  R_cell=20e-3, Rds_bms=3e-3,       # R_cell: giả định cận trên (DC), cận dưới theo CELLS[...]['ac']
  L_coil=1.0, I_coil=0.46, V_coil=12.0,        # cuộn contactor 5,5 W [giả định L = 1 H, QC2: 0,3-3 H]
  C_servo=4.4e-3, R_pc=10.0,                   # 20 servo x ~220 uF [QC2, chưa đo]; nạp trước 10 ohm 5 W
)
g = GD
# Cell (D7): AC = nội trở AC 1 kHz (cận dưới R); I_cont = dòng liên tục hãng; nguồn ở ghi chú cell nội bộ
CELLS = {'40T': dict(Ah=4.0, Vcell=3.6, m=70.0, ac=12e-3, Icont=35, name='Samsung INR21700-40T', note='35 A theo bản tentative (có/không cắt nhiệt chưa xác minh); AC <= 12 mOhm'),
         'P45B': dict(Ah=4.5, Vcell=3.6, m=70.0, ac=7e-3, Icont=45, name='Molicel INR-21700-P45B', note='45 A cắt 80 C; AC 7 mOhm')}
C = CELLS[g['CELL']]; g['Ah']=C['Ah']; g['Vcell']=C['Vcell']; g['m_cell']=C['m']
g['n3095']=g['n_arm']*g['arm3095']+g['lift3095']; g['n3215']=g['n_arm']*g['arm3215']+g['base3215']
def awg_d(n): return 0.127*92**((36-n)/39)            # mm, ASTM B258
def awg_a(n): return math.pi/4*awg_d(n)**2           # mm2
def awg_r(n):  # mOhm/m ở nhiệt độ làm việc
    return g['rho']/(awg_a(n)*1e-6)*1e3*g['strand']*(1+g['alpha']*(g['Thot']-20))
# ampacity tham chiếu NEC 310.16 cột 75 C, đồng (nhớ từ bảng, chưa kiểm URL)
AMP = {2:115, 3:100, 4:85, 6:65, 8:50, 10:35, 12:25, 14:20, 16:13, 18:10, 22:5, 24:3.5, 26:2.2}

# ---------------- dòng ----------------
arm = g['arm3095']*g['I3095']+g['arm3215']*g['I3215']; base = g['base3215']*g['I3215']+g['lift3095']*g['I3095']
tot = arm*g['n_arm']+base
Ik = g['k']*tot
I_trunk = Ik+g['Ipi_max']            # liên tục 41,9 A
Ipk_trunk = tot+g['Ipi_max']         # đỉnh 120,5 A (v0.3)
I_des = Ipk_trunk*g['margin']        # 126,1 A
n_cell = g['n_par']*g['n_ser']
Wh = n_cell*g['Ah']*g['Vcell']
Iavg2h = Wh*g['usable']/2/g['Vnom']
t_run = Wh*g['usable']/(g['Vnom']*I_trunk)*60   # phút ở dòng liên tục
Icell = {x:x/g['n_par'] for x in (I_trunk, Ipk_trunk, I_des)}
Iavg2h_by = {k_:15*v['Ah']*v['Vcell']*g['usable']/2/g['Vnom'] for k_,v in CELLS.items()}
Wh_by = {k_:15*v['Ah']*v['Vcell'] for k_,v in CELLS.items()}

# ---------------- dây ----------------
# id, tu, den, AWG, L(m, mot chieu; truc chinh: ca di va ve), I lien tuc, I dinh, dau noi, ghi chu, cau chi (A), ten cau chi
TRUNK = ('W01','W02','W03','W16','W19')
W = [
 ("W01","BMS P+","Cau chi F1",4,0.06,I_trunk,Ipk_trunk,"cos M6 + bu-long","phan truc chinh; F1 gan sat cuc P+ BMS (<=10 cm, boot cach dien); thu tu P+ -> F1 -> D0 (D5)",60,"F1 MEGA 60 A"),
 ("W02","Cong tac ngat pin D0","PCB J1 (+)",4,0.15,I_trunk,Ipk_trunk,"cos M6 bu-long (J1 cuc M6)","phan truc chinh; sau D0",60,"F1 MEGA 60 A"),
 ("W03","BMS P-","PCB J1 (-)",4,0.27,I_trunk,Ipk_trunk,"cos M6 bu-long","phan truc chinh, duong ve",60,"F1 MEGA 60 A"),
 ("W04","PCB J2","hub tay trai",8,0.60,arm*g['k'],arm,"XT60 [kiem 8 AWG]","8 servo: 4 STS3095 + 4 STS3215",40,"F3 MEGA 298 40 A"),
 ("W05","PCB J3","hub tay phai",8,0.60,arm*g['k'],arm,"XT60 [kiem 8 AWG]","8 servo",40,"F4 MEGA 298 40 A"),
 ("W06","PCB J4","hub de + truc nang",12,0.50,base*g['k'],base,"XT60","3 STS3215 banh + 1 STS3095 truc nang",20,"F5 MAXI 20 A"),
 ("W07","hub tay","moi STS3095 (x4/tay, 8)",16,0.30,g['I3095']*g['k'],g['I3095'],"theo servo thuc (chuoi A, da thay)","sao: khong noi chuoi nguon qua cap 3 chan",10,"ATOF 10 A o hub"),
 ("W08","hub tay","moi STS3215 (x4/tay, 8)",22,0.30,g['I3215']*g['k'],g['I3215'],"theo servo thuc (chuoi A, da thay)","",3,"ATOF 3 A o hub"),
 ("W09","hub de","moi STS3215 banh (x3)",22,0.40,g['I3215']*g['k'],g['I3215'],"theo servo thuc (chuoi A, da thay)","",3,"ATOF 3 A o hub"),
 ("W10","hub de","STS3095 truc nang (x1)",16,0.60,g['I3095']*g['k'],g['I3095'],"theo servo thuc (chuoi A, da thay)","",10,"ATOF 10 A o hub"),
 ("W11","PCB J5","Pi 5 (5 V)",16,0.30,5.0,5.0,"XT30","5,09 V; xem rui ro cap qua GPIO/USB-C; dong gioi han boi buck",0,"khong (buck gioi han dong)"),
 ("W12","Sac 12,6 V","BMS P+/P- (cong sac)",14,0.30,3.0,5.0,"XT60","bo sac 3S 12,6 V [chua chon]; dung truoc D0 va F1",5,"ATOF 5 A sat cong sac"),
 ("W13","pin 3S5P","BMS balance",24,0.10,0.05,0.2,"JST-XH 4P","day can bang",1,"1 A nhanh tren moi tap"),
 ("W14","PCB J8","nut E-stop 2NC",22,0.50,0.50,0.8,"cos tron / vit","vong cuon K1 0,46 A + K2 0,04 + LED/chia ap; PTC F2 1,1 A hold; Imax/Isc cua PTC F2: khong chot (chuoi A, da thay) (N9, can datasheet)",1.1,"PTC F2 1,1 A hold"),
 ("W15","PCB J6/J7","Pi 5 GPIO/I2C",26,0.20,0.01,0.1,"Dupont 2,54","3V3, SDA, SCL, GND, RELAY_EN, ESTOP/LATCH/ARM_SENSE, PRECHARGE_OK, ALERT",0,"khong (Pi gioi han)"),
 ("W16","PCB J9","contactor K1 200 A (vao + ra)",4,0.30,I_trunk,Ipk_trunk,"cos M6 bu-long","2 chang J9 <-> K1; tong 0,30 m",60,"F1 MEGA 60 A"),
 ("W17","PCB J11","nut Reset 1NO+1NC (4 day)",22,0.50,0.03,0.05,"vit","dong ESTOP_OK -> K3/K2, ~0,03 A",1.1,"PTC F2 1,1 A hold"),
 ("W18","PCB J10","cuon K1 contactor",22,0.30,0.46,0.5,"vit","cuon 5,5 W, ~0,46 A",1.1,"PTC F2 1,1 A hold"),
 ("W19","Cau chi F1","Cong tac ngat pin D0",4,0.06,I_trunk,Ipk_trunk,"cos M6 + bu-long","phan truc chinh; D0 tren mat vo (do dai thuc [gia dinh] giu 0,06 m de khong doi sut ap)",60,"F1 MEGA 60 A"),
 ("W20","PCB J1 (+) busbar ngoai PCB","PCB J9 (BAT_F, vao K1)",4,0.06,I_trunk,Ipk_trunk,"busbar/day 4 AWG cos M6 ngoai PCB","N5: doan J1 -> J9 khong di dong PCB; dong dinh 120,5 A keo toi khi F1 chay; dai [gia dinh]",60,"F1 MEGA 60 A"),
 ("W21","PCB J9 (SERVO_PRE)","RS1 -> F3/F4/F5",4,0.06,Ik,tot,"busbar/day 4 AWG cos M6 ngoai PCB; RS1 va F3/F4/F5 gan tren busbar","N5: SERVO_PRE -> RS1 -> SERVO_12V -> F3/F4/F5 (117,9 A chan) ngoai PCB; PCB chi nhan 2 day sense Kelvin; dai [gia dinh]",60,"F1 MEGA 60 A"),
]
AT, F1A, AA, FA = g['AWG_T'], g['F1'], g['AWG_ARM'], g['F_ARM']
_W=[]
for w in W:
    w=list(w)
    if w[3]==4 and w[0] not in ('W04','W05'): w[3]=AT; w[7]=w[7].replace('4 AWG',f'{AT} AWG'); w[8]=w[8].replace('4 AWG',f'{AT} AWG')
    if w[9]==60: w[9]=F1A; w[10]=f'F1 MEGA {F1A} A'
    if w[0] in ('W04','W05'):
        w[3]=AA; w[9]=FA; w[10]=f"F{3 if w[0]=='W04' else 4} MEGA 298 {FA} A"
        w[7]=f"cực M6 bu-long, cos {AA} AWG (XT60 không nhận {AA} AWG)"
        w[8]=f"{g['arm3095']} STS3095 + {g['arm3215']} STS3215 (vai, khuỷu ghép đôi)"
    if w[0]=='W07': w[2]=f"moi STS3095 (x{g['arm3095']}/tay, {g['arm3095']*g['n_arm']})"
    if w[0]=='W08': w[2]=f"moi STS3215 (x{g['arm3215']}/tay, {g['arm3215']*g['n_arm']})"
    if w[0]=='W21': w[8]=re.sub(r'98,3 A dinh',f'{tot:.1f} A chan'.replace('.',','),w[8])
    _W.append(tuple(w))
W=_W
Rbolt = g['R_bolt']
Rtr_w = awg_r(AT)*sum(w[4] for w in W if w[0] in TRUNK)*1e-3     # dây trục chính đi + về (W01,W02,W03,W16,W19)
Rtr_w_bms = awg_r(AT)*sum(w[4] for w in W if w[0] in ('W01','W02','W03','W19'))*1e-3
Rcell_eff = 3*g['R_cell']/g['n_par']
R_src_bms = Rcell_eff+g['Rds_bms']+Rtr_w_bms+g['R_fuse']
R_src_pcb = Rcell_eff+g['Rds_bms']+Rtr_w+g['R_fuse']+g['R_disc']+g['R_contactor']+g['R_shunt']+g['R_pcb']+6*Rbolt
def isc_branch(wid, awg, L):
    if wid in ('W11','W14','W15','W17','W18'): return None
    if wid in ('W12','W13'): return 12.6/(R_src_bms+2*awg_r(awg)*L*1e-3)
    if wid in TRUNK or wid in ('W20','W21'): return 12.6/R_src_bms
    return 12.6/(R_src_pcb+g['R_xt60']+2*awg_r(awg)*L*1e-3)
rows=[]
for w in W:
    i,a,b,awg,L,Ic,Ip,conn,note,fa,fname=w
    S=awg_a(awg); ad=AMP[awg]*g['derate']
    t_allow=(g['k_cu_pvc']*S/Ip)**2
    dv=Ic*awg_r(awg)*L*(1 if (i in TRUNK or i in ('W20','W21')) else 2)*1e-3
    isc=isc_branch(i,awg,L)
    rows.append(dict(ma=i,tu=a,den=b,AWG=awg,tiet_dien_mm2=round(S,2),dai_m_mot_chieu=L,dong_lien_tuc_A=round(Ic,2),dong_dinh_A=round(Ip,1),
        ampacity_tham_chieu_A=AMP[awg],ampacity_x0_82_A=round(ad,1),cau_chi_rieng=fname,
        cau_chi_nho_hon_ampacity_ha_nhiet=("co" if fa and fa<=ad else ("khong ap dung" if not fa else "KHONG")),
        Isc_cuoi_nhanh_A=round(isc) if isc else "",I2t_day_chiu_kA2s=round((g['k_cu_pvc']*S)**2/1e3,1),
        t_day_chiu_Isc_s=round((g['k_cu_pvc']*S/isc)**2,3) if isc else "",
        dat_dong_lien_tuc="co" if Ic<=ad else "KHONG",
        t_chiu_dong_dinh_s=round(t_allow,1),sut_ap_V=round(dv,3),dau_noi=conn,ghi_chu=note))
if LUU_A:   # phần A (v0.3, PCB) lưu đối chiếu trong archive; bang-day.csv là chuỗi L
  with open(os.path.join(ARCH,"bang-day-chuoi-a-v03.csv"),"w",newline="",encoding="utf-8") as f:
    wr=csv.DictWriter(f,fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)

# ---------------- sụt áp trục chính ----------------
Rtrunk = {'day truc chinh (W01,W02,W03,W16,W19; di + ve)':Rtr_w,'cau chi F1 MEGA (D9)':g['R_fuse'],
          'cong tac ngat pin D0':g['R_disc'],'contactor K1 200 A':g['R_contactor'],
          '6 moi cos M6 bu-long (J1 x2, D0 x2, J9 x2)':6*Rbolt,
          'shunt RS1':g['R_shunt'],'busbar/day ngoai PCB (W20, W21) + via + dong PCB nhanh':g['R_pcb'],'XT60 ra (1 cap)':g['R_xt60']}
Rall=sum(Rtrunk.values())
R_qc2 = awg_r(AT)*0.54*1e-3+g['R_fuse']+2*1.0e-3+g['R_contactor']+g['R_shunt']+g['R_pcb']+g['R_xt60']   # cách tính vòng QC2
V3=0.03*g['Vnom']
dv_all=Ik*Rall; dv_qc2=Ik*R_qc2
Rarm=awg_r(AA)*0.6*2*1e-3; dv_arm=arm*g['k']*Rarm
Rmax3 = V3/Ik
# ---------------- ngắn mạch ----------------
Isc_nom = 12.6/R_src_bms
Isc_12 = 12.6/(3*12e-3/g['n_par']+g['Rds_bms']+Rtr_w_bms+g['R_fuse'])
Isc_ub = 12.6/(3*10e-3/g['n_par']+1.5e-3+0.1e-3+1.0e-3)
R_need_1k = 12.6/1000
# ---------------- nhả K1 ----------------
Rc = g['V_coil']/g['I_coil']; Lc = g['L_coil']; I0 = g['I_coil']; I1 = 0.25*I0
def t_diode_only(L): return L/Rc*math.log(4)
def t_clamp(L,Vc): return L/Rc*math.log((Vc+I0*Rc)/(Vc+I1*Rc))
def t_clamp_noR(L,Vc): return L*(I0-I1)/Vc
Vd = 0.45
Vz = 15.0
# ---------------- nạp trước ----------------
tau_pc = g['R_pc']*g['C_servo']; E_pc = 0.5*g['C_servo']*12.6**2; I_pc = 12.6/g['R_pc']
R_loop_nopc = Rcell_eff+g['Rds_bms']+Rtr_w+g['R_fuse']+g['R_disc']+g['R_contactor']+g['R_shunt']
I_nopc = 12.6/R_loop_nopc
# RC trễ K1: R1 47k, R2 470k, C 10 uF, nguồn 3,3 V
R1_, R2_, C13 = 47e3, 470e3, 10e-6
Rpar = R1_*R2_/(R1_+R2_); Vf = 3.3*R2_/(R1_+R2_); tau_d = Rpar*C13
t_delay = lambda vth: -tau_d*math.log(1-vth/Vf)
# POR M2
R_por, C_por = 470e3, 1e-6
Ktol = 0.99*0.8      # R -1%, C -20% (X7R + lệch DC) [giả định dung sai]; cận trên R +1%, C +20%
t_por_min = lambda vt: Ktol*(-R_por*C_por*math.log(1-vt/3.3))
t_por_max = lambda vt: (1.01*1.2)*(-R_por*C_por*math.log(1-vt/3.3))
t_por = lambda vt: -R_por*C_por*math.log(1-vt/3.3)
# C_K3
tau_k3 = 360*220e-6
# ---------------- phối hợp pin, kích thước ----------------
cell_d, cell_l = 21.55, 70.15
cross_L, cross_H = 5*cell_d, 3*cell_d
# ---------------- PCB đồng ----------------
def ipc_width_mm(I,dT,oz):
    A=(I/(0.048*dT**0.44))**(1/0.725)
    return A/(oz*1.378)*0.0254
w60_2oz=ipc_width_mm(60,20,2); wb_2oz=ipc_width_mm(I_trunk,20,2); warm=ipc_width_mm(arm,20,2)
w_use=12.0; Lp=60e-3; Rpcb=g['rho']*Lp/(2*w_use*1e-3*70e-6)
# ---------------- buck / INA ----------------
Pout=5.09*5; Pin=Pout/0.9; Iin_min=Pin/7.5; Iin_max=Pin/12.6
fs=lambda rt:92471*rt**-0.991
vout=0.8*(1+53.6/10.0)
lsb1=0.0025; cal1=round(0.00512/(lsb1*1e-3)); lsb2=0.0002; cal2=round(0.00512/(lsb2*0.02))
Pshunt1=I_trunk**2*1e-3; Pshunt1b=60**2*1e-3

# ---------------- Isc NGAY SAU từng cầu chì nhánh (vòng 2, 22/11/2026) ----------------
# Hai bộ R nguồn: CẬN DƯỚI = cell Molicel P45B (R pack + BMS 5,7 mOhm = 3 x 7 mOhm / 5 + 1,5), các phần tử khác hạ,
# dây lạnh (20 C); CẬN TRÊN = giả định hiện hành (R_cell 20, BMS 3 mOhm, dây nóng). Mọi R phần tử khác là [giả định], chưa đo.
LO = dict(R_cell=C['ac'], Rds_bms=1.5e-3, R_fuse=1.0e-3, R_contactor=1.0e-3, R_disc=0.3e-3, R_shunt=1.0e-3, R_pcb=0.35e-3, R_bolt=0.2e-3, R_xt60=0.5e-3, wire=1/(1+g['alpha']*(g['Thot']-20)))
HI = dict(R_cell=g['R_cell'], Rds_bms=g['Rds_bms'], R_fuse=g['R_fuse'], R_contactor=g['R_contactor'], R_disc=g['R_disc'], R_shunt=g['R_shunt'], R_pcb=g['R_pcb'], R_bolt=g['R_bolt'], R_xt60=g['R_xt60'], wire=1.0)
def r_src(P, where, wid=None, awg=None, L=0.0):
    w=P['wire']; rc=3*P['R_cell']/g['n_par']+P['Rds_bms']
    rt=awg_r(AT)*sum(x[4] for x in W if x[0] in ('W01','W02','W03','W19'))*1e-3*w
    base=rc+rt+P['R_fuse']
    if where=='bms': R=base                                             # sát cực BMS / cổng sạc
    elif where=='batf': R=base+P['R_disc']+4*P['R_bolt']                # BAT_F trước K1 (F6, F10, F11)
    else:                                                              # sau K1 + shunt, ra PCB (F3-F5)
        R=base+P['R_disc']+P['R_contactor']+P['R_shunt']+P['R_pcb']+6*P['R_bolt']
        if where=='hub': R+=P['R_xt60']+2*awg_r(AA)*0.6*1e-3*w          # ATOF ở hub tay (W04 8 AWG 0,6 m)
    return R
# (ma, cầu chì, dây bảo vệ, vị trí, khả năng cắt A @ 32 VDC, nguồn khả năng cắt)
IR_MAXI=(1000,"Littelfuse MAXI 299: 1000 A @ 32 VDC, https://www.littelfuse.com/assetdocs/littelfuse-datasheet-299-maxi32v?assetguid=68b8bbe8-3e51-4c7b-bbda-3c71667f75f3 [snippet, 22/11/2026]")
IR_MEGA=(2000,"Littelfuse MEGA 298 (40-250 A): 2000 A @ 32 VDC, https://www.littelfuse.com/assetdocs/littelfuse-datasheet-mega-32v?assetguid=9fe0cd60-17bf-4fd7-b18d-977d32179af9 [snippet, 22/11/2026]; mã 0298040.ZXEH nhớ từ catalogue, chưa đối chiếu datasheet")
IR_ATOF=(1000,"Littelfuse ATOF 287: 1000 A @ 32 VDC, https://www.littelfuse.com/assetdocs/littelfuse-datasheet-287-atof?assetguid=43dcdce8-8ca2-426f-8998-7e566f048d40 [snippet, 22/11/2026]")
BR=[("F3",f"MEGA 298 {g['F_ARM']} A (D9; D7: 40 A)","W04 (tay trái)","pcb",IR_MEGA),("F4",f"MEGA 298 {g['F_ARM']} A (D9; D7: 40 A)","W05 (tay phải)","pcb",IR_MEGA),
    ("F5","MAXI 299 20 A","W06 (đế + trục nâng)","pcb",IR_MAXI),("F6","ATOF 7,5 A","buck TPS54560 (BAT_F)","batf",IR_ATOF),
    ("F10","ATOF 2 A","K4/R_pc (BAT_F)","batf",IR_ATOF),("F11","ATOF 5 A","TVS1 (BAT_F)","batf",IR_ATOF),
    ("EXT-F7","ATOF 10 A / 3 A ở hub","W07/W08 (từng servo)","hub",IR_ATOF),("EXT-F8","ATOF 5 A","W12 (cổng sạc)","bms",IR_ATOF)]
BRrows=[]
for ma,fz,day,where,(ir,src) in BR:
    lo=12.6/r_src(LO,where); hi=12.6/r_src(HI,where)
    lo45=12.6/r_src(dict(LO,R_cell=CELLS['P45B']['ac']),where); lo40=12.6/r_src(dict(LO,R_cell=CELLS['40T']['ac']),where)
    BRrows.append(dict(ma=ma,fz=fz,day=day,lo=lo,hi=hi,lo40=lo40,lo45=lo45,ir=ir,src=src,ok=max(lo40,lo45)<=ir))
Rneed_ir = {ir: 12.6/ir for ir in (1000,2000)}

def r(x,n=2): return f"{x:.{n}f}".replace(".",",")

md=f"""# Tính toán điện M1 (v0.3 cơ khí, chưa duyệt)

Ngày tính: 22/11/2026 (tính lại theo cơ khí v0.3, D9; bản đầu 21/11/2026). Sinh bằng `tinh_toan.py` (sửa giả định ở khối `GD`, chạy lại). Áp quyết định D3/D4 (`quyet-dinh-bms-estop.md` Phần 3, QC2 vòng 2). Cờ: [trang bán] = số hãng tự báo, [snippet] = chỉ thấy qua tìm kiếm/bên bán, [giả định] = tự đặt, chưa đo, [chưa kiểm] = nhớ từ chuẩn/datasheet, chưa mở nguồn, [datasheet] = đã đọc nguồn gốc, [QC2] = số do kiểm QC2, [tự tính]. Chưa phải thiết kế đã duyệt, chưa phải chứng nhận ISO 13849-1 hay IEC 60204-1.

## 1. Dòng

| Nhánh | Thành phần | Dòng chặn (A) | Dòng liên tục giả định, hệ số {g['k']} (A) |
|---|---|---|---|
| Tay trái | {g['arm3095']} STS3095 + {g['arm3215']} STS3215 | {r(arm,1)} | {r(arm*g['k'],1)} |
| Tay phải | {g['arm3095']} STS3095 + {g['arm3215']} STS3215 | {r(arm,1)} | {r(arm*g['k'],1)} |
| Đế + trục nâng | {g['base3215']} STS3215 (bánh) + {g['lift3095']} STS3095 | {r(base,1)} | {r(base*g['k'],1)} |
| Tổng servo | {g['n3095']} STS3095 + {g['n3215']} STS3215 | {r(tot,1)} | {r(Ik,1)} |
| Nhánh Pi (phía pin) | buck 5,09 V x 5 A, hiệu suất 90% [giả định] | {r(g['Ipi_max'],1)} | tối đa {r(g['Ipi_max'],1)} (ở 10,8 V) |
| Trục chính pin -> PCB | servo + Pi | **{r(Ipk_trunk,1)}** (đỉnh) | **{r(I_trunk,1)}** |
| Dòng thiết kế BMS/pack (đỉnh x {g['margin']}) | | **{r(I_des,1)}** | |

Nguồn: dòng chặn 9,8 A/STS3095 (https://openelab.io/products/feetech-st3095-c002-serial-servo) và 2,7 A/STS3215 C018 (https://evelta.com/st3215-c018-12v-30kg-cm-1-345-dual-shaft-ttl-serial-servo-metal-gears-magnetic-encoder/), tra 22/11/2026 qua báo cáo F016, [trang bán], chưa có datasheet Feetech. Hệ số {g['k']} là giả định F016, chưa đo.

## 2. Pack 3S5P và thời gian chạy

Cell (D7, 22/11/2026): chính {CELLS['40T']['name']} ({CELLS['40T']['note']}), dự phòng {CELLS['P45B']['name']} ({CELLS['P45B']['note']}); P42A hết hàng nên không còn dùng. Bảng dưới theo cell **{C['name']}** (tham số `CELL`), 15 cell 3S5P; cột phụ cho cell kia. BMS 3S chọn >= {g['BMS_A']} A [snippet, thông số thật Thiếu dữ liệu, D9].

| Mục | Giá trị | Ghi chú |
|---|---|---|
| Năng lượng | {r(Wh,1)} Wh = {n_cell} x {g['Ah']} Ah x {g['Vcell']} V | [tự tính] danh định; 40T {r(Wh_by['40T'],1)} Wh, P45B {r(Wh_by['P45B'],1)} Wh; 40T ở 3,9 Ah: {r(15*3.9*3.6,1)} Wh |
| Dòng cell ở {r(I_trunk,1)} / {r(Ipk_trunk,1)} / {r(I_des,1)} A | {r(Icell[I_trunk],1)} / {r(Icell[Ipk_trunk],1)} / **{r(Icell[I_des],1)}** A | {r(Icell[I_des]/C['Icont']*100,0)}% của {C['Icont']} A liên tục hãng ({C['name']}); 40T: {r(Icell[I_des]/35*100,0)}% của 35 A; P45B: {r(Icell[I_des]/45*100,0)}% của 45 A. Dòng cell thực ở đỉnh servo {r(Icell[Ipk_trunk],1)} A ({r(Icell[Ipk_trunk]/C['Icont']*100,0)}%); {r(Icell[I_des],1)} A chỉ là mức BMS phải chịu |
| Chạy 2 giờ | dòng TB <= **{r(Iavg2h,2)} A** | Wh x {g['usable']} [giả định] / (2 h x {g['Vnom']} V); 40T {r(Iavg2h_by['40T'],2)} A, P45B {r(Iavg2h_by['P45B'],2)} A. Không phụ thuộc số servo; số servo đổi thì dòng TB thực tế (đo EVT) mới quyết định đạt hay không |
| Chạy ở dòng liên tục {r(I_trunk,1)} A | ~{r(t_run,0)} phút | |
| Khối lượng cell | tối đa {r(n_cell*g['m_cell'],0)} g | + nickel + BMS |
| BMS {g['BMS_A']} A / đỉnh | {r(g['BMS_A']/Ipk_trunk,2)} lần; / {r(I_des,1)} A = {r(g['BMS_A']/I_des,2)} lần | bên bán khuyên liên tục <= 50% [snippet]: {g['BMS_A']//2} A >= {r(I_trunk,1)} A |

Thiếu dữ liệu: datasheet BMS (ngưỡng quá dòng/ngắn mạch, trễ, kích thước, nhiệt), busbar 5P cho {r(I_des,0)} A, datasheet bản chính thức cell, UN38.3 và IEC 62133-2 cho pack tự đóng [chưa kiểm].

## 3. Cỡ dây (đã hạ nhiệt)

Công thức: đường kính AWG theo ASTM B258, đồng 1,724e-8 ohm.m ở 20 C, hệ số bện {g['strand']}, nhiệt độ làm việc {g['Thot']} C (x{r(1+g['alpha']*(g['Thot']-20),3)}) [tự tính]. Dòng chịu tham chiếu: NEC 310.16 cột 75 C [chưa kiểm URL], hạ x{g['derate']} cho vỏ kín 45 C [chưa kiểm NEC 310.15(B)]. Thời gian chịu dòng đỉnh: t = (k.S/I)^2 với k = {g['k_cu_pvc']} (IEC 60364-4-43, PVC) [chưa kiểm URL]. Toàn bộ 21 dây ở `bang-day.csv`.

| Dây | AWG | S (mm2) | mOhm/m nóng | Liên tục (A) | Chịu x0,82 (A) | Đỉnh (A) | Chịu đỉnh (s) | Cầu chì riêng |
|---|---|---|---|---|---|---|---|---|
"""
for x in rows:
    if x['ma'] in ('W01','W04','W06','W07','W08','W12','W13','W14','W16','W19','W20','W21'):
        md+=f"| {x['ma']} {x['tu']} -> {x['den']} | {x['AWG']} | {x['tiet_dien_mm2']} | {r(awg_r(x['AWG']),2)} | {x['dong_lien_tuc_A']} | {x['ampacity_x0_82_A']} | {x['dong_dinh_A']} | {x['t_chiu_dong_dinh_s']} | {x['cau_chi_rieng']} |\n"
t4=(g['k_cu_pvc']*awg_a(AT)/Ipk_trunk)**2
md+=f"""
Dây trục chính **{AT} AWG** (D9; v0.1.1 dùng 4 AWG cho F1 60 A): {r(AMP[AT]*g['derate'],1)} A >= F1 {F1A} A; dòng liên tục {r(I_trunk,1)} / {F1A} = {r(I_trunk/F1A*100,0)}% của F1 (ngưỡng liên tục <= 80% [chưa kiểm hướng dẫn Littelfuse]; F1 60 A cho {r(I_trunk/60*100,0)}%: vượt). Ở dòng đỉnh {r(Ipk_trunk,1)} A dây chịu **{r(t4,0)} s**; F1 ở {r(Ipk_trunk/F1A*100,0)}% phải cắt < {r(t4/2,0)} s (hệ số 2). **Thiếu dữ liệu:** đường đặc tính thời gian-dòng MEGA {F1A} A. Dây nhánh tay {AA} AWG ({r(AMP[AA]*g['derate'],1)} A >= F3/F4 {FA} A >= dòng chặn nhánh {r(arm,1)} A).

Dòng nối tiếp qua cáp 3 chân của chuỗi servo không đủ cho STS3095: nguồn đi hình sao từ hub, chỉ dữ liệu theo cáp 3 chân. Đầu nối nguồn thật của STS3095/STS3215 C018: **Thiếu dữ liệu**.

## 4. Sụt áp

Ngưỡng SOR-016 (Mong muốn, không phải an toàn): <= 3% ở dòng liên tục {r(Ik,1)} A. 3% của {g['Vnom']} V = {r(V3*1000,0)} mV, tức R tổng <= **{r(Rmax3*1000,1)} mOhm**.

| Đoạn | R (mOhm) | Nguồn |
|---|---|---|
"""
src={'day truc chinh (W01,W02,W03,W16,W19; di + ve)':f'tính, {r(sum(w[4] for w in W if w[0] in TRUNK),2)} m cả đi lại, {r(awg_r(AT),2)} mOhm/m nóng','cau chi F1 MEGA (D9)':'[giả định] chưa có số hãng','cong tac ngat pin D0':'[giả định] chưa có số hãng','contactor K1 200 A':'[giả định] chưa chọn mã; QC2: contactor 200 A thật thường thấp hơn','6 moi cos M6 bu-long (J1 x2, D0 x2, J9 x2)':'[giả định] 0,3 mOhm/mối, chưa đo (XT90 cũ tính 1,0 mOhm/điểm)','shunt RS1':'1 mOhm','busbar/day ngoai PCB (W20, W21) + via + dong PCB nhanh':'[giả định] giữ 0,70 mOhm: busbar/dây trục W20+W21 ngoài PCB (<= 0,12 mOhm tính) + via + đồng PCB nhánh; thay đoạn đồng PCB cũ (0,62 mOhm), chưa đo','XT60 ra (1 cap)':'[giả định] 0,8 mOhm'}
for k_,v in Rtrunk.items(): md+=f"| {k_} | {r(v*1000,2)} | {src[k_]} |\n"
md+=f"""| **Tổng trục chính tới đầu ra PCB** | **{r(Rall*1000,2)}** | |

- Tổng: {r(dv_all*1000,0)} mV = **{r(dv_all/g['Vnom']*100,2)}%** của {g['Vnom']} V ở {r(Ik,1)} A ({r(Rall*1000,2)} mOhm x {r(Ik,1)} A). {'Trong 3%.' if dv_all<=V3 else 'VƯỢT 3%: SOR-016 chưa đạt theo giả định hiện tại.'} Con số {r(dv_all/g['Vnom']*100,2)}% là số duy nhất dùng trong tài liệu; {r(Rall*1000,2)} mOhm ĐÃ gồm D0 (0,50), 6 mối cos M6 (D0 x2, J1 x2, J9 x2, theo công thức 6 x 0,3) và dây W01/W19; không cộng thêm D0 lần nữa.
- Cách tính vòng QC2 (v0.1.1) đã bỏ: số duy nhất là bảng trên. Để đạt 3% ở {r(Ik,1)} A cần R tổng <= {r(Rmax3*1000,1)} mOhm, hiện {r(Rall*1000,2)} mOhm; phần dây ({r(Rtr_w*1000,2)} mOhm) nhỏ so với contactor, F1, mối cos, nên đổi dây trục không cứu SOR-016; chỉ số thật (contactor, F1, D0) mới quyết định.

- Nhánh tay ({AA} AWG, 0,6 m, {r(arm*g['k'],1)} A): thêm {r(dv_arm*1000,0)} mV ({r(dv_arm/g['Vnom']*100,2)}%).
- Sụt áp trong cell + BMS (R_cell {g['R_cell']*1000:.0f} mOhm [giả định]) nằm ngoài ngưỡng này, cần đo khi có pack. Đo ở Q-FQC-C10.

## 5. Cầu chì F1, ngắn mạch và phối hợp cầu chì nhánh

- F1 = Littelfuse MEGA 298 bolt-down {F1A} A (D9; mã Thiếu dữ liệu, 0298060.ZXB là bản 60 A của D3), 32 V, cắt 2 kA @ 32 VDC [snippet: https://www.littelfuse.com/products/fuses/automotive-passenger-car/high-current-fuses/298, 22/11/2026: dải 40-500 A, 40 và 60 A đầu đồng thau, 80 A trở lên đầu đồng, 40-250 A cho dòng liên tục]; 50 A và 70 A có tồn tại hay không: Thiếu dữ liệu; xác nhận riêng {F1A} A và giá: Thiếu dữ liệu. MIDI 498 chỉ 1 kA @ 32 VDC [snippet] nên loại.
- Vị trí (D5): thứ tự P+ BMS -> F1 -> D0. F1 gắn bolt-down sát cực P+ BMS, đoạn P+ -> F1 <= 10 cm (W01 = 6 cm, giới hạn cứng 15 cm), cos + thân F1 bọc boot cách điện: cực P+ và thân F1 vẫn mang điện khi D0 tắt. Sụt áp, ampacity, Isc không đổi so với thứ tự cũ [tự tính].
- Isc = 12,6 V / (3 x R_cell / {g['n_par']} + R_FET BMS + dây + R cầu chì): **{r(Isc_nom,0)} A** (R_cell {g['R_cell']*1000:.0f} mOhm, BMS {g['Rds_bms']*1000:.0f} mOhm, [giả định]); R_cell 12 mOhm: **{r(Isc_12,0)} A**; cận trên (R_cell 10, BMS 1,5, dây 0,1, cầu chì 1 mOhm): **{r(Isc_ub,0)} A** [tự tính]. Cận trên {r(Isc_ub,0)} A < MEGA 2 kA. Cầu chì nhánh 1 kA chỉ đủ nếu R pack + BMS thật >= {r(R_need_1k*1000,1)} mOhm; với 3S5P R ở nominal là {r(R_src_bms*1000,1)} mOhm, nhưng R_cell 12 mOhm cho {r(Isc_12,0)} A sát 1 kA: **bắt buộc đo R pack + BMS** hoặc dùng cầu chì nhánh >= 2 kA.
- Mọi I²t của cầu chì là [giả định] (3-10 A: vài chục A²s; 20-40 A: 0,5-10 kA²s). Đường cong, I²t, khả năng cắt từng mã: Thiếu dữ liệu (QC2 điều kiện b).

| Dây | Cầu chì | Ampacity x0,82 (A) | Cầu chì <= ampacity hạ? | Liên tục / đỉnh (A) | Isc cuối nhánh (A) | I²t dây chịu (kA²s) | Dây chịu ở Isc (s) |
|---|---|---|---|---|---|---|---|
"""
for x in rows:
    if x['ma'] in ('W04','W06','W07','W08','W12','W13','W14','W16','W20','W21'):
        md+=f"| {x['ma']} | {x['cau_chi_rieng']} | {x['ampacity_x0_82_A']} | {x['cau_chi_nho_hon_ampacity_ha_nhiet']} | {x['dong_lien_tuc_A']} / {x['dong_dinh_A']} | {x['Isc_cuoi_nhanh_A']} | {x['I2t_day_chiu_kA2s']} | {x['t_day_chiu_Isc_s']} |\n"
md+="""
### Isc ngay sau từng cầu chì nhánh, so với khả năng cắt (vòng 2, 22/11/2026)

Ba cột Isc: (1) cell chính 40T (AC <= 12 mOhm, R pack + BMS = 3 x 12 / 5 + 1,5 = 8,7 mOhm), (2) cell P45B (AC 7 mOhm, 5,7 mOhm; molicel.com ghi DCIR tối đa 13,8 mOhm [theo kiểm chứng, chưa tự mở]), cả hai với các phần tử khác ở giá trị thấp (F1 1,0; K1 1,0; D0 0,3; PCB 0,35; mối cos 0,2; XT60 0,5 mOhm; dây lạnh) nên là **Isc lớn nhất có thể**; (3) giả định hiện hành (R_cell 20, BMS 3 mOhm, dây nóng) là Isc nhỏ hơn. Tất cả [giả định], chưa đo. Isc = 12,6 V / R, ngay sau cầu chì (dây sau cầu chì không tính). Số servo không đổi Isc ở đầu nhánh, chỉ đổi dòng định mức cần chịu.

| Cầu chì | Bảo vệ | Isc max, 40T (A) | Isc max, P45B (A) | Isc, giả định hiện hành (A) | Khả năng cắt (A @ 32 VDC) | Đạt ở Isc max? |
|---|---|---|---|---|---|---|
"""
for x in BRrows:
    md+=f"| {x['ma']} {x['fz']} | {x['day']} | {r(x['lo40'],0)} | {r(x['lo45'],0)} | {r(x['hi'],0)} | {x['ir']} | {'đạt' if x['ok'] else 'KHÔNG ĐẠT'} |\n"
md+=f"""
Nguồn khả năng cắt: MAXI 299 và ATOF 287 đều 1000 A @ 32 VDC, MEGA 298 2000 A @ 32 VDC [snippet, Littelfuse, 22/11/2026; link ở khối `IR_*` của tinh_toan.py]. **F3/F4 (D9) là MEGA 298 {FA} A (2 kA)**: dòng chặn nhánh {r(arm,1)} A, ampacity W04/W05 ({AA} AWG) {r(AMP[AA]*g['derate'],1)} A; F5 (20 A, chặn {r(base,1)} A) không đổi. F5, F6, F10, F11 và F8 (cổng sạc) giữ ATOF/MAXI nhưng xem cột "Đạt?": điều kiện bắt buộc là đo R pack + BMS + đường ở EVT (cần R tới điểm đặt >= 12,6 mOhm), chưa đạt thì không phát hành; cell nickel và mối hàn chưa tính vào R nên cận dưới thiên về an toàn. Cầu chì cần R nguồn >= {r(Rneed_ir[1000]*1000,1)} mOhm (tới điểm đặt) để Isc <= 1 kA. Cầu chì 1 A x3 trên dây cân bằng (EXT-F9) và PTC F2: khả năng cắt Thiếu dữ liệu (mã chưa chọn). **Điều kiện bắt buộc cho mọi cầu chì nhánh 1 kA:** đo R pack + BMS + dây ở EVT trước khi chốt; nếu Isc cận dưới > 1 kA thì đổi sang cầu chì có khả năng cắt >= 2 kA.
"""
md+=f"""
Phối hợp: cầu chì nhánh nằm sau F1 và phải cắt trước F1 ở dòng sự cố cuối nhánh (hàng trăm A); cầu chì nhỏ cắt nhanh hơn MEGA {F1A} A nên chọn lọc theo thiết kế, **chưa kiểm bằng đường cong** (Thiếu dữ liệu). Điều kiện còn mở (QC2): ngưỡng chảy quy ước ~1,35x vượt ampacity x0,82 ở 8/12/16 AWG nên quá tải 100-130% có thể kéo dài (đo nhiệt dây ở EVT); F1 {F1A} A so đỉnh {r(Ipk_trunk,1)} A ({r(Ipk_trunk/F1A*100,0)}%): sẽ chảy nếu đỉnh kéo dài, cần số liệu thời lượng đỉnh. F6 7,5 A (đầu vào buck, 3,8 A tối đa) bảo vệ vệt đồng mỏng vì buck lấy BAT_F trước K1. **N3 (v0.1.1):** F10 ATOF 2 A 32 V nối tiếp K4/R_pc (dòng nạp trước đỉnh 1,26 A xung ~40 ms; 63% định mức [giả định], đường cong Thiếu dữ liệu) và F11 ATOF 5 A 32 V nối tiếp TVS1 (TVS chập thì F11 chảy, F1 không phải chảy; xung SMCJ14A có thể chảy F11, I²t Thiếu dữ liệu); quy tắc layout: vệt BAT_F tới F10/F11 ngắn, đặt cầu chì ngay chỗ rẽ. F10 không bảo vệ trường hợp bus servo chập khi K4 đóng (1,26 A < 2 A, R_pc nóng 15,9 W/5 W): N2, chưa đóng.

## 6. Công tắc ngắt pin (điểm cách ly chính, tay)

Yêu cầu từ QC lắp ráp: cực M6 không cách ly được pack; nhánh Pi lấy trước contactor nên E-stop không cách ly toàn robot. Cần một điểm ngắt tay sau F1 (thứ tự P+ -> F1 -> D0, D5), DC liên tục >= {F1A} A, chịu đỉnh {r(I_des,0)} A, khoá được ở vị trí tắt khi bảo trì. Công tắc đặt ở cực dương, cắt cả Pi.

| | Phương án 1: công tắc ngắt pin xoay, khoá được | Phương án 2: đầu nối Anderson SB120/SB175 |
|---|---|---|
| Ví dụ | họ Blue Sea Systems 6006 / loại tương đương (DC, 300 A liên tục [chưa kiểm]) | Anderson SB175: ~175 A, SB120: ~120 A [chưa kiểm] |
| Khoá khi bảo trì | Có (tháo núm hoặc khoá), thao tác một tay, thấy trạng thái | Rút đầu nối: không khoá được ở vị trí "tắt"; dễ cắm lại nhầm |
| Cắt có tải | Thiết kế để ngắt tay có tải [chưa kiểm]; phải tắt qua phần mềm trước (Pi) | Hồ quang khi rút có tải: không nên, phải tắt trước |
| R tiếp điểm | [giả định] {g['R_disc']*1000:.1f} mOhm + 2 mối cos | [giả định] ~0,3-0,5 mOhm + 2 mối cos |
| Giá | Thiếu dữ liệu | Thiếu dữ liệu |
| Chiều hợp IEC 60204-1 mục ngắt nguồn [chưa kiểm điều khoản] | Phù hợp (thiết bị ngắt khoá được) | Không (không khoá được) |

**Chọn phương án 1** (công tắc ngắt pin xoay, DC, khoá được; mã Thiếu dữ liệu, cần datasheet: định mức DC 14 V liên tục >= {F1A} A, chịu {r(I_des,0)} A, điện trở tiếp điểm, khả năng ngắt). Lý do: bảo trì cần khoá (LOTO), người dùng thấy ngay trạng thái, không phụ thuộc đầu nối. Ghi: Anderson chỉ chọn nếu cần tháo nhanh pack và kèm khoá cơ khí riêng. Hệ quả cần ghi nhận: (a) tắt D0 cắt luôn Pi: Pi mất nguồn đột ngột, hướng dẫn tắt phần mềm trước (N8); (b) cổng sạc (W12) nối BMS P+/P- trước D0 nên vẫn có điện khi D0 tắt: cách ly đầy đủ cần rút bộ sạc; (c) đoạn P+ -> F1 là đoạn duy nhất không có cầu chì phía trước: F1 sát cực P+, <= 10 cm, boot cách điện; khi D0 tắt F1 và cực P+ vẫn mang điện, quy trình thay cầu chì: rút bộ sạc, tắt D0, nhớ F1 phía pin vẫn có điện, dùng dụng cụ cách điện; (d) công tắc một cực (chỉ cực dương), cực âm vẫn nối BMS P-.

## 7. Contactor K1, nạp trước, nhả nhanh

- Contactor 200 A, cuộn 12 V, ngoài PCB (J9 cực M6 cho đường lực, J10 cho cuộn); mã, ngắt DC 14 V, R tiếp điểm, điện áp hút ở 9-10 V, thời gian nhả: Thiếu dữ liệu (ứng viên: Altran AEVT200-B, Gigavac GV200 BA, TE EV200 [snippet, xem `quyet-dinh-bms-estop.md` 1.7]). Không nhận khi chưa có datasheet.
- **Nạp trước (bắt buộc):** C servo {g['C_servo']*1e3:.0f} mF [QC2, chưa đo]. Không nạp trước: R vòng {r(R_loop_nopc*1000,1)} mOhm -> xung ~{r(I_nopc,0)} A [tự tính, mô hình này; QC2 ghi ~485 A với R vòng khác]. Có nạp trước: R_pc {g['R_pc']:.0f} ohm 5 W, dòng đỉnh {r(I_pc,2)} A, tau = {r(tau_pc*1000,0)} ms (90% ở {r(2.3*tau_pc*1000,0)} ms, 99% ở {r(4.6*tau_pc*1000,0)} ms), năng lượng {r(E_pc*1000,0)} mJ trong R_pc (5 W đủ, xung).
- **Trình tự (mạch):** RELAY_EN (cạnh lên) -> U4.Q = RUN -> Q4 đóng K4 (nạp trước) ngay; K1 đóng trễ bởi RC (R1 47 k // R2 470 k, C13 10 uF, nguồn 3,3 V -> Vf = {r(Vf,2)} V, tau = {r(tau_d*1000,0)} ms); Q5 (cùng nút G_Q1) tắt K4 khi K1 bắt đầu hút. Pi xác nhận U1 >= 90% trước khi cho chạy (PRECHARGE_OK chỉ cho biết hết trễ). Đo ở EVT.
- **Bảng trễ đóng K1 theo Vth của Q1 (N4, thay bảng cũ theo AO3400A):** t = -tau x ln(1 - Vth/Vf), Vf = {r(Vf,2)} V, tau = {r(tau_d*1000,0)} ms [tự tính].

| Q1 | Vth (V) | Trễ đóng K1 (ms) | Ghi chú |
|---|---|---|---|
| AO3400A (bản cũ, 30 V: không còn dùng cho Q1) | 0,65 / 1,05 / 1,45 | {r(t_delay(0.65)*1000,0)} / {r(t_delay(1.05)*1000,0)} / {r(t_delay(1.45)*1000,0)} | [giả định theo họ AO3400A, chưa kiểm]; vẫn dùng cho Q4 |
| Ứng viên DMN6075S (Diodes, 60 V, SOT-23, 2 A) | 1,3 min / 2,7 max (Vds = Vgs, 250 uA) | {r(t_delay(1.3)*1000,0)} / {r(t_delay(2.7)*1000,0)} | [snippet] https://www.diodes.com/part/view/DMN6075S (tra 21/11/2026); bên bán Element14/Farnell ghi Vth max 3 V; Rds(on) chỉ ghi ở Vgs 4,5 V (120 mOhm) và 10 V, không ghi ở 3 V; datasheet gốc chưa đọc |
| Bất kỳ, Vth max 2,0 V | 2,0 | {r(t_delay(2.0)*1000,0)} | điều kiện đề xuất cho mã chốt |
| Vth max >= 3,0 V (= Vf) | >= 3,0 | không bao giờ đóng | loại |

  Hệ quả: (1) Vgs thực chỉ {r(Vf,2)} V, nên Q1 chỉ chọn nếu **Vth max <= 2,0 V** và Rds(on) có ghi ở Vgs <= 3 V cho cuộn 0,46 A; DMN6075S ở Vth max 2,7 V chỉ còn 0,3 V quá áp: không đạt theo thông số ghi trên trang, **mã chốt Thiếu dữ liệu**. (2) K4 nhả theo Vth(Q5), K1 hút theo Vth(Q1): nếu Vth(Q1) > Vth(Q5) bus servo hở trong khoảng chênh. Để chênh chỉ còn do sai số mã, Q5 đổi sang **cùng mã với Q1** (bản v0.1.1); với DMN6075S (1,3-2,7 V) khoảng hở trong cùng lô tối đa {r((t_delay(2.7)-t_delay(1.3))*1000,0)} ms nếu hai con lệch hết dải (cận xấu, trong thực tế cùng lô nhỏ hơn nhiều, chưa đo). Cách chặt hơn nếu EVT đo thấy bus tụt: cho K4 nhả theo tiếp điểm phụ của K1 (chưa thiết kế).

- **Nhả K1 (SOR-011 < 100 ms):** cuộn {g['V_coil']:.0f} V, {g['I_coil']} A (R = {r(Rc,1)} ohm), L = {g['L_coil']:.0f} H [giả định, 0,3-3 H].

| Mạch dập | L = 1 H: về 25% dòng | L = 3 H | Ghi chú |
|---|---|---|---|
| D1 đơn | {r(t_diode_only(1)*1000,0)} ms | {r(t_diode_only(3)*1000,0)} ms | trượt 100 ms nếu L = 3 H |
| D1 + Zener 15 V (không tính R cuộn) | {r(t_clamp_noR(1,Vz+Vd)*1000,0)} ms | {r(t_clamp_noR(3,Vz+Vd)*1000,0)} ms | cận thận trọng (số QC2: 22 ms) |
| D1 + Zener 15 V (có R cuộn) | {r(t_clamp(1,Vz+Vd)*1000,0)} ms | {r(t_clamp(3,Vz+Vd)*1000,0)} ms | |

Cộng cơ khí và hồ quang: chưa đo. **Phát hiện mới:** khi dừng bằng phần mềm (Q1 tắt, K1_FEED còn 12,6-14 V), drain Q1 lên tới 12,6 + {Vz+Vd:.2f} = **{r(14+Vz+Vd,1)} V** (TVS 14 V: {r(14+Vz+Vd,1)} V) so với 30 V của AO3400A: không có biên. Bản v0.1 đổi Q1 sang N-MOSFET >= 60 V (bảng trễ ở trên, mã Thiếu dữ liệu); Q4 (drain <= {r(14+0.5,1)} V) giữ AO3400A, Q5 cùng mã Q1. Zener 15 V hấp thụ ~{r(0.5*Lc*I0**2*1000,0)} mJ, ~{r(t_clamp_noR(1,Vz+Vd)*1e3,0)} ms: cần loại Zener >= 1 W/xung ~5 W hoặc TVS, mã Thiếu dữ liệu.
- **Xả âm K1_FEED (Minor QC2):** khi E-stop mở, dòng cuộn K1 tiếp tục qua D1 + Zener, kéo K1_FEED (và ESTOP_OK qua K2 pole 2 còn đóng) xuống ~-{Vz+Vd:.1f} V cho tới khi K2 nhả (vài chục ms). Xử lý trong bản v0.1: (1) **D8 SS14 chặn** nhánh K4 (nếu không, D6 + diode thân Q4 kẹp K1_FEED ở ~-1 V và làm mất tác dụng Zener); (2) LED3 qua chặn ngược (LED chịu ngược ~5 V); (3) ba nhánh cảm biến: chia áp 100k/33k + Zener 3,3 V (kẹp -0,7 V) + 1k nối tiếp, dòng kẹp ~{r(15.7/100e3*1000,2)} mA; (4) tiếp điểm K2 pole 2 mở chịu {Vz+Vd:.1f} V (cách điện rơ-le >> 16 V [chưa kiểm]); (5) D4 (SS14, 40 V ngược) và nút Reset chịu được. Chứng minh bằng tính; kiểm SPICE/oscilloscope ở EVT.

## 8. Chốt reset P1, M2 và thời gian

- Reset kích theo nhả nút: K3 hút khi nhấn (NO đóng, ESTOP_OK có điện), giữ nhờ C_K3 220 uF qua cuộn 360 ohm [giả định]: tau = {r(tau_k3*1000,0)} ms, còn >= 20% điện áp sau {r(tau_k3*math.log(5)*1000,0)} ms; K2 hút trong thời gian đó khi nhả nút (NC đóng lại), rồi tự giữ qua pole 1. Nút kẹt nhấn: NC mở, K2 không bao giờ được cấp. K2/K3 cuộn thật: Thiếu dữ liệu (đo).
- **K3 dính (lỗi đơn, Category B; sửa M1 QC2):** nút nghỉ có NC đóng nên ESTOP_OK -> NC -> K3a (dính) -> K2 hút ngay khi nhả E-stop, không cần nhấn NO. Cùng loại K2 pole 1 dính. Chốt phần cứng mất; chỉ còn U4 + Pi (không an toàn) chặn chạy; Pi chẩn đoán LATCH_SENSE cao khi ARM_SENSE thấp. Chấp nhận ở Category B, ghi công khai. **Không** gọi là lỗi kép.
- **U4 74LVC1G74 buộc cạnh RELAY_EN:** D = 3V3, PRE = 3V3, /CLR = LATCH_SENSE AND POR_OK (U6 74LVC1G08). Reset không là lệnh chạy; Pi treo kẹt cao không chạy lại sau E-stop.
- **M2 (POR):** 74LVC1G74 không tự xoá khi cấp nguồn. R20 470 k (đổi từ 330 k, N7) + C15 1 uF + Schmitt 74LVC1G17 (U5) giữ POR_OK thấp -> /CLR thấp -> Q = 0. Danh định {r(t_por(1.5)*1000,0)}-{r(t_por(2.0)*1000,0)} ms sau khi 3V3 lên (VT+ 1,5-2,0 V [giả định theo họ LVC, chưa kiểm datasheet]); **với dung sai** R +/-1%, C X7R -20%/+20% (kể cả lệch điện áp DC): {r(t_por_min(1.5)*1000,0)}-{r(t_por_max(2.0)*1000,0)} ms, cận dưới >= 200 ms (bản 330 k cho 158 ms ở cận dưới, QC2 N7). Nếu VT+ min của U5 thấp hơn 1,5 V (datasheet Thiếu dữ liệu): VT+ 1,0 V cho cận dưới {r(t_por_min(1.0)*1000,0)} ms, cần tăng C hoặc R. Phép đo FQC 'POR >= 200 ms' dùng cận này. D9 BAT54 xả C15 khi 3V3 sụt. R21 100 k kéo CLK xuống. **Lý do chọn RC + Schmitt** thay IC giám sát: ít linh kiện cùng họ 74LVC1G, trễ do R, C chứ không do ngưỡng IC, rẻ; nhược: không bắt sụt áp chậm. Nếu EVT đo thấy brown-out lúc Pi khởi động, đổi sang IC giám sát (ví dụ họ TPS3808 có chân trễ [chưa kiểm]). Lớp thứ hai: glitch Q lúc 3V3 lên ngắn hơn trễ RC (tối thiểu {r(t_delay(1.3)*1000,0)} ms theo ứng viên Q1) nên không thể đóng K1. **N1 (an toàn):** nhánh LATCH_CLR (mức K1_FEED sau chia áp 100k/33k + Zener 3,3 V, qua R35 1k) chỉ nối U6.A; Pi đọc qua đệm một chiều U7 (74LVC1G17, cấp 3V3_PI) + R39 1k tới LATCH_SENSE (J6.P7). Pi cấu hình output hay pull-up chỉ chạm đầu ra U7, không có đường tới /CLR của U4 [tự suy ra từ netlist; U7 chưa có datasheet đối chiếu]. Khi Pi tắt (3V3 = 0), đầu vào U7 vẫn nhận <= 3,3 V: họ LVC chịu đầu vào 5 V khi VCC = 0 [chưa kiểm datasheet]. Ghi thêm: chọn chân Pi cho LATCH_SENSE là GPIO >= 9 (pull-down mặc định) làm lớp phòng thủ thứ hai. Mức logic LATCH_CLR: 12,6 V -> 3,13 V; VIH ~2,0 V [giả định] tương ứng K1_FEED >= ~8,1 V (dưới đó /CLR xoá, K1 nhả: coi như cắt thấp áp). Thiếu dữ liệu: datasheet U4/U5/U6, đo ở EVT.

"""
md+=f"""## 10. Nguồn 5 V cho Pi 5

TPS54560B (4,5-60 V vào, 5 A, Vref 0,8 V +/- 1%, chân theo SLVSF00: 1 BOOT, 2 VIN, 3 EN, 4 RT/CLK, 5 FB, 6 COMP, 7 GND, 8 SW, pad GND) [datasheet] https://www.ti.com/lit/ds/symlink/tps54560b.pdf (tra 22/11/2026). Linh kiện theo mạch ví dụ 5 V/5 A/400 kHz của TI mục 8.2 (RT 243 kOhm cho 400 kHz theo công thức fsw = 92471 x RT^-0,991 = {r(fs(243),0)} kHz, R_comp 16,9 kOhm, C_comp 4700 pF, C 47 pF, L 7,2 uH, diode B560C, Cout 3 x 47 uF). Đổi R_FB dưới 10,2 -> 10,0 kOhm để ra Vout = 0,8 x (1 + 53,6/10,0) = **{r(vout,2)} V** (Pi 5 cần sụt dây nhỏ, chấp nhận >= 4,75 V ở Pi). L thay bằng 6,8 uH bậc chuẩn, Isat >= 8 A (ripple tăng nhẹ so với 7,2 uH, cần mô phỏng). Phân áp EN 442 k/90,9 k bật ở ~6,5 V theo ví dụ TI.

Dòng vào: Pout = 5,09 V x 5 A = {r(Pout,1)} W, Pin ~ {r(Pin,1)} W (90% [giả định]) -> {r(Iin_max,1)} A ở 12,6 V, {r(Iin_min,1)} A ở 7,5 V.

**Rủi ro:** cấp 5 V qua chân GPIO/J5 bỏ qua đàm phán PD của Pi 5, USB ngoại vi bị giới hạn dòng mặc định; xử lý bằng cấu hình và dây USB-C PD, **Thiếu dữ liệu** (chưa kiểm tài liệu Raspberry Pi).

## 11. Đo dòng/áp (INA226)

INA226: bus 0-36 V, shunt +/-81,92 mV, LSB 2,5 uV (shunt) và 1,25 mV (bus), VS 2,7-5,5 V, VSSOP-10, địa chỉ chọn bằng A0/A1 (SBOS547C) [datasheet] https://www.ti.com/lit/ds/symlink/ina226.pdf (tra 22/11/2026). Chân: A1=1, A0=2, ALERT=3, SDA=4, SCL=5, VS=6, GND=7, VBUS=8, IN-=9, IN+=10.

| Chip | Vị trí | Shunt | Địa chỉ | Dải dòng đo | Current_LSB | CAL |
|---|---|---|---|---|---|---|
| U1 | bus servo sau relay | 1 mOhm (5 W, 4 chân) | 0x40 | +/-81,9 A (ở dòng đỉnh {r(tot,0)} A sẽ bão hoà, không hỏng chip) | 2,5 mA | {cal1} (0x{cal1:04X}) |
| U2 | đầu vào buck | 20 mOhm (1 W) | 0x41 | +/-4,1 A (đủ {r(Iin_min,1)} A) | 0,2 mA | {cal2} (0x{cal2:04X}) |

CAL = 0,00512 / (Current_LSB x R_shunt) [datasheet]. Công suất shunt 1 mOhm: {r(Pshunt1,1)} W ở {r(I_trunk,1)} A, {r(Pshunt1b,1)} W ở 60 A (cũ), **{r(tot**2*1e-3,1)} W ở {r(tot,1)} A (dòng đỉnh servo) và {r(Ipk_trunk**2*1e-3,1)} W ở {r(Ipk_trunk,1)} A**: RS1 5 W (bản trước) sai, đổi RS1 **>= 20 W** (D9; 15 W chỉ hơn dòng chặn {r(15/(tot**2*1e-3),2)} lần; hệ số {r(20/(tot**2*1e-3),2)} so với {r(tot,1)} A), 4 chân Kelvin, gắn ngoài PCB, mã Thiếu dữ liệu. Phương án khác: shunt 0,5 mOhm (5,0 W ở {r(tot,1)} A, dải đo +/-163,8 A, CAL x2) nếu không tìm được 1 mOhm >= 15 W [tự tính, chưa chọn]. Pi cấp 3V3 cho INA226 (INA226 tắt khi Pi tắt). U2 đọc điện áp pin ngay cả khi E-stop mở relay, để Pi cảnh báo pin yếu.

## 12. Đồng PCB (IPC-2221, ngoài, ΔT = 20 C) [chưa kiểm URL; công thức quen thuộc]

| Dòng | Rộng cần trên 2 oz (70 um) |
|---|---|
| 60 A (F3/F4 cũ, F1 cũ) | {r(w60_2oz,1)} mm |
| {r(I_trunk,1)} A (liên tục) | {r(wb_2oz,1)} mm |
| {r(arm,1)} A (nhánh tay, đỉnh) | {r(warm,1)} mm |

Đề xuất: 2 lớp 2 oz, đường dòng chính rộng 12 mm trên cả hai lớp (tổng 24 mm: đủ cho dòng liên tục {r(wb_2oz,1)} mm một lớp, đủ cho 60 A khi chia đều hai lớp (10,7 mm/lớp; nhánh tay {r(arm,1)} A chặn cần {r(warm,1)} mm một lớp, tức {r(warm/2,1)} mm/lớp so với 12 mm/lớp đang đề xuất) nhưng KHÔNG đủ cho đỉnh {r(Ipk_trunk,1)} A kéo dài tới khi F1 chảy (~22 mm/lớp, ΔT ~54 C ở 12 mm/lớp) [QC2 N5, IPC-2221 ngoại suy trên 35 A, chưa kiểm]. Vì vậy ĐỊNH TUYẾN NGOÀI PCB: (a) J1 -> J9 (BAT_F, W20) bằng busbar/dây {AT} AWG; (b) SERVO_PRE -> RS1 -> SERVO_12V -> F3/F4/F5 (W21, {r(tot,1)} A chặn) bằng busbar/dây {AT} AWG, RS1 và F3/F4/F5 gắn trên busbar; (c) GND trục chính là dây {AT} AWG W03 tới J1(-), đổ GND PCB chỉ cho nhánh. PCB chỉ còn dòng nhánh, tín hiệu và sense Kelvin của RS1, khâu via dày, mở mặt nạ hàn và phủ thiếc. R đồng chính 60 mm với 2 x 12 mm: {r(Rpcb*1000,2)} mOhm. Cần kiểm nhiệt thật. Kích thước đề xuất: 100 x 75 mm (<= 100 x 80 mm của `cad/ASSUMPTIONS.md`), chưa layout nên kích thước là ước lượng.

"""
md+=f"""## 13. Kích thước đề xuất (khớp `cad/ASSUMPTIONS.md`)

| Hạng mục | Giả định CAD | Đề xuất / số tìm được | Cờ |
|---|---|---|---|
| PCB nguồn | <= 100 x 80 mm | 100 x 80 mm, 2 lớp 2 oz, cao <= 28 mm; layout nháp có 94 khối, chưa đi dây | [tự suy ra], `pcb-nguon/pcb_nguon.kicad_pcb` |
| Pack 3S5P {C['name']} (15 cell xếp 5 x 3) | khay 160 x 80 x 70 mm | cắt ngang {r(cross_L,1)} x {r(cross_H,1)} mm, dài ~76 mm gồm nickel [QC2]: xem `cad/ASSUMPTIONS.md` | [snippet] Ø21,55 x 70,15 mm (như P42A) |
| BMS 3S {g['BMS_A']} A | | kích thước Thiếu dữ liệu; bản 100 A 184 x 65 x 33 mm vượt khay 160 mm | [snippet] |
| Contactor 200 A, F1 MEGA bolt-down, công tắc ngắt pin D0, khối cầu chì nhánh | | khối và chỗ gắn Thiếu dữ liệu | |
| Nút E-stop 2NC + nút Reset 1NO+1NC | Ø22 mm | lỗ bảng 22 mm; thân sau ~35-50 mm mỗi nút [giả định] | [chưa kiểm] |
"""
md+=f"""
## 14. Chọn lại theo cơ khí v0.3 (D9, 22/11/2026)

Quy tắc: BMS >= đỉnh x {r(g['margin'],2)}; cầu chì >= dòng chặn nhánh và <= ampacity dây x {g['derate']}; khả năng cắt >= Isc; dòng liên tục <= 80% định mức cầu chì [chưa kiểm hướng dẫn Littelfuse]. Ampacity: NEC 310.16 cột 75 C [chưa kiểm URL]. Khả năng cắt MEGA 298 2 kA @ 32 VDC, dải 40-500 A [snippet Littelfuse 22/11/2026]; **50 A và 70 A có hay không: Thiếu dữ liệu**.

### Trục chính (F1 + dây)

| Phương án | Dây | Chịu x{g['derate']} (A) | Liên tục / F1 | Đỉnh {r(Ipk_trunk,1)} A / F1 | Dây >= F1? | Dây chịu đỉnh (s) | Ghi chú |
|---|---|---|---|---|---|---|---|
"""
for fz,aw,note in [(60,4,"D3 cũ: liên tục vượt 80%"),(70,3,"nấc 70 A chưa xác nhận tồn tại; 3 AWG ít gặp, khó tìm cos"),(80,2,"**chọn** (D9): đầu đồng, dây chuẩn, cos M6 có")]:
    amp=AMP[aw]*g['derate']
    md+=f"| F1 {fz} A | {aw} AWG | {r(amp,1)} | {r(I_trunk/fz*100,0)}% {'(vượt 80%)' if I_trunk/fz>0.8 else '(đạt)'} | {r(Ipk_trunk/fz*100,0)}% | {'có' if amp>=fz else 'KHÔNG'} | {r((g['k_cu_pvc']*awg_a(aw)/Ipk_trunk)**2,0)} | {note} |\n"
md+=f"""
### Nhánh tay (F3/F4 + dây + đầu nối)

| Phương án | Dây | Chịu x{g['derate']} (A) | Cầu chì >= dòng chặn {r(arm,1)} A? | Dây >= cầu chì? | Ghi chú |
|---|---|---|---|---|---|
"""
for fz,aw,note in [(40,8,"D7 cũ: nhỏ hơn dòng chặn; chỉ giữ nếu chia nhánh 3095/3215 (3215 không có cầu chì 2 kA < 40 A)"),(50,6,"nấc 50 A chưa xác nhận tồn tại; 6 AWG nhẹ hơn; dự phòng"),(60,4,"**chọn** (D9): nấc 60 A có trên trang Littelfuse, dây trùng cỡ cũ của trục, cos M6; XT60 không nhận 4 AWG nên J2/J3 đổi sang cực M6")]:
    amp=AMP[aw]*g['derate']
    md+=f"| F3/F4 MEGA {fz} A | {aw} AWG | {r(amp,1)} | {'có' if fz>=arm else 'KHÔNG'} | {'có' if amp>=fz else 'KHÔNG'} | {note} |\n"
md+=f"""
Cầu chì tại hub (ATOF 10 A mỗi STS3095 chặn {r(g['I3095'],1)} A; 3 A mỗi STS3215 chặn {r(g['I3215'],1)} A) và F5 20 A (chặn {r(base,1)} A, 12 AWG {r(AMP[12]*g['derate'],1)} A) không đổi. Rủi ro cơ khí: 4 AWG silicone dày và cứng, 0,6 m x 2 dây mỗi tay đi qua khớp pan: cần CAD kiểm bán kính uốn; đảo lại: 50 A + 6 AWG nếu nấc 50 A có thật.

### BMS 3S, contactor, D0, shunt

| Hạng mục | Yêu cầu mới | Phương án | Đánh giá |
|---|---|---|---|
| BMS 3S | >= {r(I_des,1)} A (đỉnh {r(Ipk_trunk,1)} x {r(g['margin'],2)}) | 150 A: {r(150/I_des,3)} lần | **loại** (thiếu 0,4%) |
| | | 160 A: {r(160/I_des,2)} lần; liên tục theo khuyến cáo bên bán 50% = 80 A >= {r(I_trunk,1)} A | dùng được nếu có mã |
| | | **200 A**: {r(200/I_des,2)} lần; 50% = 100 A | **chọn** (D9); listing 3S 100/150/200 A có UART/BT [snippet, Walmart.ca marketplace, 22/11/2026]; datasheet, kích thước, ngưỡng cắt: Thiếu dữ liệu |
| Contactor K1 200 A | ngắt >= {r(I_des,0)} A @ 14 V | đỉnh {r(Ipk_trunk/200*100,0)}% / liên tục {r(I_trunk/200*100,0)}% của 200 A | **còn đủ** (về dòng); mã, R tiếp điểm, make/break vẫn Thiếu dữ liệu |
| F1 MEGA | {F1A} A | liên tục {r(I_trunk/F1A*100,0)}%, đỉnh {r(Ipk_trunk/F1A*100,0)}% | đường cong Thiếu dữ liệu |
| D0 | >= {F1A} A liên tục, chịu {r(I_des,0)} A | | đổi yêu cầu từ 60 A lên {F1A} A |
| RS1 | >= 20 W | chặn {r(tot,1)} A: {r(tot**2*1e-3,1)} W; đỉnh {r(Ipk_trunk**2*1e-3,1)} W | đổi từ 15 W |
| Cell | | 40T chính: {r(Icell[Ipk_trunk],1)} A/cell ở đỉnh servo ({r(Icell[Ipk_trunk]/35*100,0)}% của 35 A); {r(Icell[I_des],1)} A ở ngưỡng BMS ({r(Icell[I_des]/35*100,0)}%). P45B: {r(Icell[Ipk_trunk]/45*100,0)}% / {r(Icell[I_des]/45*100,0)}% của 45 A | giữ D7 (40T chính, P45B dự phòng); biên mỏng hơn 25,2 A (72%) cũ |
"""

# =====================================================================================================================
# PHẦN L (v0.5, chốt 23/11/2026 tối): chuỗi nguồn L cho M1 bản gọn cấu hình P-L (2 tay AM-ARM200 nguyên, vai/khuỷu đơn, 6+1).
# Số servo P-L và ngân sách dòng firmware đọc từ sdk/faibotic (spec.PL_SERVO_MODELS, limits.current_budget). Số ghép đôi / P-B
# của chuỗi B (trước khi chốt P-L) chỉ còn là ghi chú lịch sử. Phần A ở dưới (v0.3, PCB, D0, EV200) giữ để đối chiếu.
# =====================================================================================================================
import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sdk'))
from faibotic import spec as _spec, limits as _lim   # noqa: E402

GL = dict(
  n_par=4, n_ser=3,                 # 3S4P, 12 cell
  BMS_A=100,                        # Daly Smart 3S 100 A [snippet srikobatteries.com 23/11/2026]; ngưỡng cắt chờ datasheet (DK-N2)
  F1=80, AWG_T=2,                   # MEGA 298 80 A + dây 2 AWG
  K1_A=100,                         # contactor 100 A cuộn 12 V kiểu Albright SW80 [snippet]; ngắt DC 12-14 V chờ datasheet (DK-N1)
  R_contactor=3.0e-3, R_plug=0.5e-3, R_fuse=2.0e-3, R_bolt=0.3e-3, R_cell=20e-3, Rds_bms=3e-3,   # [giả định] như phần A
  k=0.4,                            # hệ số liên tục [giả định F016]
  C_servo_per=220e-6,               # tụ đầu vào mỗi servo [giả định như phần A, chưa đo]
  blade_IR=1000,                    # cầu chì lưỡi (ATO/ATC) cắt 1 kA @ 32 VDC [snippet Littelfuse 287, phần A mục 5]
  latch_A=_spec.LIFT_LATCH_COIL_A, ctrl_A=_spec.CTRL_CURRENT_A, pi_A=_spec.PI_CURRENT_A,
)
_PL = _spec.PL_SERVO_MODELS
_bud = _lim.current_budget(_PL, models=_PL)
n95 = sum(1 for v in _PL.values() if v is _spec.STS3095); n15 = len(_PL) - n95
stallL = n95 * _spec.STS3095.stall_a + n15 * _spec.STS3215.stall_a
import day_L as DL   # noqa: E402  bảng dây từng sợi (nguồn duy nhất cho bang-day.csv, vat-tu-day.csv)
auxL = GL['pi_A'] + GL['ctrl_A'] + GL['latch_A'] + DL.AUX12_A
pkL_pi = stallL + GL['pi_A']                 # 95,5 A: cách tính của điều phối (servo + Pi)
pkL_sdk = stallL + GL['pi_A'] + GL['ctrl_A'] + GL['latch_A']   # 97,7 A: dòng chặn theo limits.current_budget (không có 12V_AUX)
pkL = stallL + auxL
desL = pkL * g['margin']
contL = GL['k'] * stallL + auxL
arm95 = sum(1 for n, v in _PL.items() if n.startswith('left') and v is _spec.STS3095)
arm15 = sum(1 for n, v in _PL.items() if n.startswith('left') and v is _spec.STS3215)
armL = arm95 * _spec.STS3095.stall_a + arm15 * _spec.STS3215.stall_a
baseL = 3 * _spec.STS3215.stall_a + _spec.STS3095.stall_a + GL['latch_A']
# lịch sử chuỗi B (trước P-L): ghép đôi 5+1 và P-B 5+1 theo SERVO_IDS hiện tại
_hist = {}
for tag, names in (("ghép đôi 5+1", list(_spec.SERVO_IDS)), ("P-B 5+1", [n for n in _spec.SERVO_IDS if not n.endswith(_spec.PAIR_SUFFIX)])):
    b_ = _lim.current_budget(names)
    _hist[tag] = (b_['stall_a'], b_['stall_a'] * g['margin'], b_['allowed_a'])
ncL = GL['n_par'] * GL['n_ser']
WhL = ncL * C['Ah'] * C['Vcell']
Iavg2hL = WhL * g['usable'] / 2 / g['Vnom']
IscL_ideal = 12.6 / (3 * C['ac'] / GL['n_par'])
_G = DL.G
L_POS = _G['pack_f1'] + _G['f1_j0'] + _G['j0_bus']          # BW01 + BW02 + BW03
L_NEG = _G['pack_bms'] + _G['bms_j0'] + _G['j0_bus']        # BW00 + BW04 + BW04R
L_K1 = _G['bus_k1'] + _G['k1_bus']                          # BW05 + BW06
def _rL(P_, where):
    w = P_['wire']; rc = 3 * P_['R_cell'] / GL['n_par'] + P_['Rds_bms']
    if where == 'bms': return rc + awg_r(GL['AWG_T']) * _G['pack_f1'] * 1e-3 * w + P_['R_fuse']
    rt = awg_r(GL['AWG_T']) * (L_POS + L_NEG) * 1e-3 * w
    sc = P_['R_fuse'] / g['R_fuse']
    R = rc + rt + P_['R_fuse'] + 2 * GL['R_plug'] * sc + 4 * P_['R_bolt']        # J0 cắt hai cực; cos F1 ra, J0 ×2, BAT_F, GND
    if where == 'servo': R += P_['R_contactor'] + 4 * P_['R_bolt'] + awg_r(GL['AWG_T']) * L_K1 * 1e-3 * w
    return R
IscL = {wh: dict(lo40=12.6 / _rL(dict(LO, R_cell=CELLS['40T']['ac']), wh), lo45=12.6 / _rL(dict(LO, R_cell=CELLS['P45B']['ac']), wh),
                 hi=12.6 / _rL(HI, wh)) for wh in ('bms', 'batf', 'servo')}
# đóng K1 không nạp trước (chuỗi L bỏ KT/K4): xung dòng nạp tụ servo
C_busL = len(_PL) * GL['C_servo_per']
R_inrushL = _rL(HI, 'servo')
I_inrushL = 12.6 / R_inrushL
tau_inrushL = R_inrushL * C_busL
E_inrushL = 0.5 * C_busL * 12.6 ** 2
_Ltr = L_POS + L_NEG + L_K1
RtrL = {f'dây trục chính 2 AWG (BW00–BW06, đi + về, {r(_Ltr,2)} m)': awg_r(GL['AWG_T']) * _Ltr * 1e-3,
        'cầu chì F1 MEGA 80 A': GL['R_fuse'], 'giắc J0 SB120, hai cực (cách ly bảo trì)': 2 * GL['R_plug'],
        'contactor K1 100 A kiểu SW80': GL['R_contactor'], '8 mối cos M8 (F1 ra, J0 ×2, BAT_F ×2, K1 ×2, GND)': 8 * GL['R_bolt']}
RallL = sum(RtrL.values())
assert abs(DL.PK_TRUNK - pkL) < 1e-9 and abs(DL.CONT_TRUNK - contL) < 1e-9, "day_L và tinh_toan lệch dòng trục chính"
rowsL = DL.rows()
vtL = DL.vat_tu(rowsL)
with open("bang-day.csv", "w", newline="", encoding="utf-8") as f:
    wr = csv.DictWriter(f, fieldnames=list(rowsL[0])); wr.writeheader(); wr.writerows(rowsL)
with open("vat-tu-day.csv", "w", newline="", encoding="utf-8") as f:
    wr = csv.DictWriter(f, fieldnames=list(vtL[0])); wr.writeheader(); wr.writerows(vtL)
rowsB = DL.rows(DL.WIRES_BAN)
vtB = DL.vat_tu(rowsB, DL.MISC_BAN)
with open("bang-day-de-ban.csv", "w", newline="", encoding="utf-8") as f:
    wr = csv.DictWriter(f, fieldnames=list(rowsB[0])); wr.writeheader(); wr.writerows(rowsB)
with open("vat-tu-de-ban.csv", "w", newline="", encoding="utf-8") as f:
    wr = csv.DictWriter(f, fieldnames=list(vtB[0])); wr.writeheader(); wr.writerows(vtB)
_RL = {x['ma']: x for x in rowsL}
def _dv(*ids): return sum(_RL[i]['sut_ap_lien_tuc_pct'] for i in ids)
ARM_TO_FB = ('BW07', 'BW07N', 'BW07T', 'BW07TN', 'BW07S', 'BW07SN')        # SERVO+ → F3 → … → 21W4 → FB-L, đi và về
dv_feed = _dv(*ARM_TO_FB)
dv_dsub = 2 * DL.R_DSUB * DL.CONT_FBL / g['Vnom'] * 100                    # 2 tiếp điểm (A1 + A2) ở dòng liên tục một tay
dv_arm_far = dv_feed + dv_dsub + _dv('P.LEFT_GRIPPER+', 'P.LEFT_GRIPPER-')
dv_arm_95 = dv_feed + dv_dsub + _dv('P.LEFT_ELBOW+', 'P.LEFT_ELBOW-')
dv_wheel = _dv('P.W1+', 'P.W1-')
dv_lift = _dv('P.LIFT+', 'P.LIFT-')
L8 = (_RL['BW07T']['dai_mm'] + _RL['BW07S']['dai_mm']) / 1000                # 8 AWG một chiều
def _r_fb(P_):   # nguồn tới thanh cái hộp FB-L trên trạm: sau F3, 4 AWG, khối cọc, đuôi 8 AWG, 2 tiếp điểm 21W4, 8 AWG trạm; đi và về
    w = P_['wire']
    return (_rL(P_, 'servo') + P_['R_fuse'] + 8 * P_['R_bolt'] + 2 * DL.R_DSUB + awg_r(4) * 2 * _RL['BW07']['dai_mm'] * 1e-6 * w
            + awg_r(2) * 2 * _RL['BW07A']['dai_mm'] * 1e-6 * w + awg_r(8) * 2 * L8 * 1e-3 * w)
# sụt áp vòng E-stop (mạch cuộn K1 + KH ~ CTRL_A) sau S2: C01, C02, C04, C13, C14, C15 + vòng trạm E01–E06, E09; 2 chân 21W4 tín hiệu [giả định 5 mΩ/chân]
LOOP_IDS = ('C01', 'C02', 'C03', 'C04', 'C05', 'C06', 'C08', 'C13', 'C14', 'C15', 'E01', 'E02', 'E03', 'E04', 'E05', 'E06', 'E09')
dV_loop = sum(_RL[i]['sut_ap_lien_tuc_pct'] for i in LOOP_IDS) / 100 * g['Vnom'] + 2 * 5e-3 * GL['ctrl_A']
L_tram_loop = sum(_RL[i]['dai_mm'] for i in ('E01', 'E02', 'E03', 'E04', 'E05', 'E06', 'E09')) / 1000
# dòng servo trục nâng (dòng tỉ lệ mô-men [giả định DK-N19])
lift_T = _spec.LIFT_HOLD_FORCE_N * _spec.LIFT_PINION_R
lift_frac = lift_T / _spec.STS3095.stall_nm
lift_I = lift_frac * _spec.STS3095.stall_a
lift_I09 = lift_I / 0.9
lift_lim_I = _spec.LIFT_TORQUE_LIMIT_PERMILLE / 1000 * _spec.STS3095.stall_a
rated_frac = lift_T / (_spec.STS3095.rated_kgcm * _spec.KGCM)
# chế độ bàn (đế bàn tuỳ chọn): adapter 12 V 15 A
ban_budget = _spec.ROBOT_CONFIGS['tram_ban']['current_budget_a']
arm_allowed = sum(DL.allowed(f"left_{j}") for j in DL.ARM_J)
IscFB = dict(lo40=12.6 / _r_fb(dict(LO, R_cell=CELLS['40T']['ac'])), lo45=12.6 / _r_fb(dict(LO, R_cell=CELLS['P45B']['ac'])), hi=12.6 / _r_fb(HI))
GRP = [("pan", "Pan"), ("shoulder", "Vai"), ("elbow", "Khuỷu"), ("wrist_flex", "Gập cổ tay"), ("wrist_yaw", "Yaw cổ tay"), ("wrist_roll", "Cuộn cổ tay"),
       ("gripper", "Kẹp"), ("wheel", "Bánh"), ("lift", "Trục nâng")]
blade_ok40 = IscL['servo']['lo40'] <= GL['blade_IR']; blade_ok45 = IscL['servo']['lo45'] <= GL['blade_IR']
mdB = f"""# Tính toán điện M1 bản gọn v0.5: chuỗi nguồn L, cấu hình P-L, trạm hai tay (chưa duyệt)

Ngày tính: 23/11/2026 (chốt P-L tối 23/11); tính lại 24/11/2026 cho S2 trạm hai tay. Sinh bằng `tinh_toan.py` (khối `GL`; số servo P-L và ngân sách dòng firmware
đọc từ `sdk/faibotic/spec.py` `PL_SERVO_MODELS` và `limits.current_budget`). P-L: 2 tay AM-ARM200 nguyên, vai và khuỷu đơn, 6+1 (24/11/2026: mở khớp yaw như upstream, thay 5+1):
**{n95} STS3095** (2 pan, 2 vai, 2 khuỷu, 1 trục nâng) + **{n15} STS3215** (2 gập, 2 yaw, 2 cuộn, 2 kẹp, 3 bánh).

Chuỗi L: pack 3S4P → BMS Daly 100 A → F1 MEGA 80 A → giắc cách ly bảo trì J0 → BAT_F → contactor K1 100 A cuộn 12 V → SERVO+.
Mạch dừng: nút E-stop 2NC nối thẳng trong mạch cuộn K1, nút Reset và **một** rơ-le tự giữ KH để nhả E-stop không tự chạy lại;
bỏ K2/K3/KT (và vì thế bỏ nạp trước K4). Buck 5 V TPS54560 5 A (BOM V04) cho Pi, hai hộp cầu chì lưỡi trên trạm (FB-L, FB-R),
đế cầu chì inline cho bánh, Pi và mạch dừng, bộ sạc 3S. J0 = giắc Anderson SB120 cắt cả hai cực, thay công tắc ngắt pin
riêng: **cần kỹ sư điện xác nhận.** Datasheet contactor kiểu SW80 và BMS Daly 100 A **chưa có**: DK-N1, DK-N2 còn mở, không bảng nào dưới đây nói chúng đạt.

S2 (trạm hai tay): hai tay, hộp FB-L/FB-R, 2 bo bus tay, cách ly, hub và camera đầu nằm trên trạm, nối với xe nâng qua **một giắc D-sub combo
21W4** (bảng chân ở L6b). F3/F4 hạ **60 → 40 A** cho tiếp điểm 40 A và đuôi 8 AWG. Servo trục nâng sang **BUS_B** (bus đế), nguồn
và cuộn chốt lấy ở tấm điện qua đế cầu chì inline, không còn qua FB-L. E-stop trạm và 2 công tắc khoá nối tiếp E-stop đỉnh cột trong
mạch cuộn K1. Đế bàn là bộ tuỳ chọn, mạch riêng ở L9 và `bang-day-de-ban.csv`.

## L1. Dòng (P-L)

| Mục | Giá trị |
|---|---:|
| Dòng chặn servo: {n95} × 9,8 + {n15} × 2,7 A [trang bán] | {r(stallL,1)} A |
| + Pi {r(GL['pi_A'],1)} A | **{r(pkL_pi,1)} A** |
| + mạch điều khiển {r(GL['ctrl_A'],1)} A + cuộn chốt lift {r(GL['latch_A'],1)} A [giả định] (= dòng chặn theo SDK) | {r(pkL_sdk,1)} A |
| + 12V_AUX lên trạm {r(DL.AUX12_A,1)} A (hub, 3 camera, bo bus; [giả định R3b 5.4]) = đỉnh trục chính | **{r(pkL,1)} A** |
| Dòng chặn mỗi tay ({arm95} STS3095 + {arm15} STS3215) qua F3/F4 và 21W4 | **{r(DL.PK_FBL,1)} A** {'< 40 A' if DL.PK_FBL < 40 else '> 40 A (VƯỢT nhẹ khi mọi servo một tay cùng chặn, thêm servo yaw; xem L6b)'} |
| Dòng thiết kế BMS/contactor (đỉnh × {r(g['margin'],2)}) | {r(desL,1)} A |
| Liên tục (servo × {r(GL['k'],1)} [giả định] + phụ tải) | {r(contL,1)} A |
| **Ngân sách firmware** (L2, SDK) | **{r(_bud['allowed_a'],1)} A** (+ 12V_AUX {r(DL.AUX12_A,1)} A ngoài SDK = {r(_bud['allowed_a'] + DL.AUX12_A,1)} A) |

Số của các cấu hình trước P-L: `archive/v0.4/electrical/tinh-toan-chuoi-A.md`.

## L2. Ngân sách dòng firmware (R2 7.4, P-L)

Dòng cho phép mỗi servo = dòng chặn × TORQUE_LIMIT/1000 [giả định dòng tỉ lệ mô-men, chưa đo, DK-N19]. TORQUE_LIMIT theo nhóm ở
`spec.TORQUE_LIMIT_PCT`; vai đơn: min(60%, 60 kg·cm / 95) = 600‰; lift = min(70% định mức, 0,5 × F_đo × r) =
{_spec.LIFT_TORQUE_LIMIT_PERMILLE}‰ với F_đo {r(_spec.LIFT_TOOTH_F_MEASURED_N,0)} N **[giả định, chờ đo DK-N18]**. SDK ghi TORQUE_LIMIT cho mọi servo và từ chối
`enable()` khi tổng vượt {r(_spec.CURRENT_BUDGET_A,0)} A. Bảng SDK `SERVO_IDS` là {len(_spec.SERVO_IDS)} servo P-L 6+1 (ID 1–16, yaw 21/22);
ngân sách P-L tính bằng `current_budget(PL_SERVO_MODELS, models=PL_SERVO_MODELS)`.

| Nhóm | SL | Loại | Chặn (A) | ‰ | Cho phép (A) |
|---|---:|---|---:|---:|---:|
"""
for gk, gl in GRP:
    rr = _bud['rows'][gk]
    typ = 'STS3095' if gk in ('pan', 'shoulder', 'elbow', 'lift') else 'STS3215'
    mdB += f"| {gl} | {rr['n']} | {typ} | {r(rr['stall_a'],1)} | {'/'.join(str(v) for v in sorted(rr['permille']))} | {r(rr['allowed_a'],2)} |\n"
mdB += f"""| Pi + điều khiển + cuộn chốt | | | {r(_bud['pi_a'],1)} | | {r(_bud['pi_a'],1)} |
| **Tổng** | | | **{r(_bud['stall_a'],1)}** | | **{r(_bud['allowed_a'],1)}** |

{r(_bud['allowed_a'],1)} A ≤ 80 A (F1) ≤ 100 A (BMS). Pan là STS3095 ở AM-ARM200 nên nhóm pan nặng như vai, khuỷu. Tổng chỉ đúng khi
dòng tỉ lệ mô-men và servo tôn trọng TORQUE_LIMIT khi kẹt: chưa đo (DK-N19). Mỗi tay ở giới hạn mô-men: {r(arm_allowed,2)} A.

**Trục nâng sau** (bánh 20 răng, r {r(_spec.LIFT_PINION_R*1000,0)} mm; dòng tỉ lệ mô-men [giả định]):

| Mục | Giá trị |
|---|---:|
| Lực giữ tải thiết kế {r(_spec.LIFT_DESIGN_LOAD_KG,2)} kg × (g + 0,5 m/s²) | {r(_spec.LIFT_HOLD_FORCE_N,1)} N |
| Mô-men trên bánh | {r(lift_T,2)} N·m = {r(rated_frac*100,0)}% định mức, {r(lift_frac*100,1)}% chặn |
| Dòng servo ở tải thiết kế, η 1 / η 0,9 [giả định] | {r(lift_I,2)} / {r(lift_I09,2)} A |
| TORQUE_LIMIT lift (răng PETG F_đo {r(_spec.LIFT_TOOTH_F_MEASURED_N,0)} N giữ chỗ) | {_spec.LIFT_TORQUE_LIMIT_PERMILLE}‰ → {r(lift_lim_I,2)} A, lực răng {r(_spec.LIFT_FORCE_LIMIT_N,1)} N |
| Dòng chặn (cầu chì phải chịu) | {r(_spec.STS3095.stall_a,1)} A → FLIFT 7,5 A, 16 AWG ({r(DL.amp_d(16),1)} A) |

Ở mức răng giữ chỗ, lift **không** nâng đủ tải thiết kế ({r(_spec.LIFT_FORCE_LIMIT_N,1)} < {r(_spec.LIFT_HOLD_FORCE_N,1)} N, `LIFT_LIMIT_COVERS_DESIGN_LOAD` = {_spec.LIFT_LIMIT_COVERS_DESIGN_LOAD}): chờ DK-N18 đo răng
≥ {r(_spec.LIFT_HOLD_FORCE_N/_spec.LIFT_TOOTH_SAFETY,0)} N. Phần điện không đổi theo kết quả đó: dòng lift ≤ {r(lift_I09,1)} A khi nâng, cầu chì và dây chọn theo dòng chặn.

## L3. Pack 3S4P, cell, ngắn mạch, đóng K1

| Mục | Giá trị | Ghi chú |
|---|---|---|
| Năng lượng | {r(WhL,1)} Wh = {ncL} × {r(C['Ah'],1)} Ah × {r(C['Vcell'],1)} V | {C['name']}; P45B {r(ncL*4.5*3.6,1)} Wh |
| Chạy 2 giờ | dòng TB ≤ **{r(Iavg2hL,2)} A** | dùng được {r(g['usable'],2)} [giả định]; chưa có số đo teleop (SOR-014) |
| Dòng cell: đỉnh {r(pkL,1)} / thiết kế {r(desL,1)} / ngân sách {r(_bud['allowed_a'],1)} A | {r(pkL/4,1)} / {r(desL/4,1)} / {r(_bud['allowed_a']/4,1)} A | {r(pkL/4/35*100,0)}% / {r(desL/4/35*100,0)}% / {r(_bud['allowed_a']/4/35*100,0)}% của 35 A (40T bản tentative) |
| BMS 100 A / đỉnh | {r(100/pkL,2)} lần; / thiết kế {r(100/desL,2)} lần | dưới mức × 1,25: dựa vào ngân sách firmware + F1 80 A (DK-N2) |
| Contactor 100 A phải ngắt | ≥ {r(desL,0)} A ở 12–14 V DC | hạ về mức ngân sách chỉ khi kỹ sư điện chấp nhận và đã đo (DK-N1) |
| Isc cận trên lý tưởng (chỉ cell AC 12 mΩ) | **{r(IscL_ideal,0)} A** | MEGA 2 kA: biên {r(2000/IscL_ideal,2)} lần |
| Isc sau K1 (SERVO+), 40T / P45B / giả định hiện hành | {r(IscL['servo']['lo40'],0)} / {r(IscL['servo']['lo45'],0)} / {r(IscL['servo']['hi'],0)} A | **cầu chì lưỡi chỉ cắt 1 kA @ 32 VDC** [snippet]: {'đạt sát với 40T' if blade_ok40 else 'KHÔNG ĐẠT với 40T'}, {'đạt' if blade_ok45 else '**KHÔNG ĐẠT** với P45B'} ở Isc lớn nhất. Bắt buộc đo R pack + đường; không dùng P45B với cầu chì lưỡi |
| Isc tại BAT_F (aptomat Pi, mạch dừng, cổng sạc) | {r(IscL['batf']['lo40'],0)} / {r(IscL['batf']['lo45'],0)} / {r(IscL['batf']['hi'],0)} A | cầu chì lưỡi 1 kA sẽ {'vượt' if IscL['batf']['lo40'] > GL['blade_IR'] else 'sát dưới'} với 40T ({r(IscL['batf']['lo40']/GL['blade_IR']*100,0)}%) và {'vượt' if IscL['batf']['lo45'] > GL['blade_IR'] else 'dưới'} với P45B, nên ba nhánh BAT_F dùng **aptomat DC CHINT NB1-63DC 1P, 6 kA** (C6 Pi, C4 mạch dừng, C6 cổng sạc; [trang hãng]): biên {r(6000/max(IscL['batf']['lo40'], IscL['batf']['lo45']),1)} lần. Aptomat có phân cực: đấu theo chiều dòng sự cố (BAT_F → tải) |
| Đóng K1 không nạp trước | tụ bus ~{r(C_busL*1e3,1)} mF ({len(_PL)} × 220 µF [giả định]) | xung ~{r(I_inrushL,0)} A, tau ~{r(tau_inrushL*1e6,0)} µs, {r(E_inrushL,2)} J (R vòng {r(R_inrushL*1e3,1)} mΩ, giả định hiện hành): định mức đóng của contactor và tụ servo: DK-N30 (datasheet + đo xung ở EVT) |

## L4. Sụt áp trục chính

| Đoạn | R (mΩ) | Nguồn |
|---|---:|---|
"""
for kk, v in RtrL.items():
    mdB += f"| {kk} | {r(v*1000,2)} | {'tính, ' + r(awg_r(GL['AWG_T']),2) + ' mΩ/m nóng' if 'dây' in kk else '[giả định]'} |\n"
mdB += f"""| **Tổng** | **{r(RallL*1000,2)}** | |

Ở dòng liên tục {r(contL,1)} A: {r(contL*RallL*1000,0)} mV = **{r(contL*RallL/g['Vnom']*100,2)}%** ({'trong' if contL*RallL/g['Vnom'] <= 0.03 else 'VƯỢT'} 3% của SOR-016, mong muốn).
{'Không hạ được dưới 3% bằng cỡ dây: phần lớn R nằm ở contactor, F1, J0 và mối cos [giả định]; dây 2 AWG chỉ chiếm ' + r(awg_r(GL['AWG_T'])*_Ltr*1e-3/RallL*100,0) + '%. Giữ DK-N24, đo ở EVT.' if contL*RallL/g['Vnom'] > 0.03 else ''}
Nhánh (từ SERVO+, mỗi sợi đi và về cộng lại, dòng liên tục, `bang-day.csv` cột `sut_ap_lien_tuc_pct`):
- SERVO+ → hộp FB-L trên trạm (4 AWG {r(_RL['BW07']['dai_mm']/1000,2)} m, đuôi 8 AWG, 8 AWG trạm, đi + về): {r(dv_feed,2)}%; hai tiếp điểm 21W4
  (A1 + A2, {r(DL.R_DSUB*1000,1)} mΩ mỗi tiếp điểm [giả định, DK-N55]) ở {r(DL.CONT_FBL,1)} A: **{r(dv_dsub,2)}%** ({r(2*DL.R_DSUB*DL.CONT_FBL*1000,0)} mV;
  ở dòng chặn {r(DL.PK_FBL,1)} A: {r(2*DL.R_DSUB*DL.PK_FBL*1000,0)} mV, {r(DL.R_DSUB*DL.PK_FBL**2,2)} W mỗi tiếp điểm).
- Tới kẹp trái (xa nhất, 22 AWG): {r(dv_arm_far,2)}%; tới khuỷu trái (STS3095): {r(dv_arm_95,2)}%; tới bánh {r(dv_wheel,2)}%; tới trục nâng
  (16 AWG qua vòng dịch vụ, {r(_RL['P.LIFT+']['dai_mm']/1000,2)} m) {r(dv_lift,2)}% ở dòng {r(_RL['P.LIFT+']['dong_lien_tuc_A'],2)} A (0,4 × chặn; khi nâng thật ~{r(lift_I09,1)} A).
- Trục chính + nhánh xa nhất: {r(contL*RallL/g['Vnom']*100 + dv_arm_far,2)}%.

## L5. Mạch dừng chuỗi L

```
+12V_CTRL (aptomat C4) ─ 21W4.1 ─[trạm: khoá trái ─┬─ khoá phải ─┬─ EST-NC1 ─ EST-NC2]─ 21W4.3 ─ TB3 ─ ES-NC1 ─ ES-NC2 ─ K5 ─┐
                                                   └─ 21W4.5     └─ 21W4.7                                                    │
                                                      DOCK_L        DOCK_R                                                    │
  ┌───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
  └─┬─ RST-NO1 ─┬─ HOLD ─┬─ cuộn KH ─ GND
    └─ KH.1 NO ─┘        └─ cuộn K1 (diode + Zener) ─ GND
HOLD ─ opto ─ ESTOP_OK ;  SERVO+ ─ opto ─ ARM_SENSE ;  +12V_CTRL ─ RST-NO2 ─ opto ─ RESET_SENSE ;  DOCK_L/R ─ opto ─ BCM 16/26 [đề xuất]
```

Nhấn E-stop (đỉnh cột hoặc trạm) hoặc mở một khoá: vòng hở, KH và K1 mất cuộn cùng lúc; đóng lại: KH không tự hút lại, chờ Reset.
Khoá đặt trước E-stop trạm để DOCK_L/DOCK_R chỉ phụ thuộc +12V_CTRL và hai khoá: DOCK_L = khoá trái, DOCK_R = cả hai khoá (khoá trái
mở thì chưa biết khoá phải; Studio báo khoá trái trước). Mở lẫy là hở vòng, K1 nhả (~20 ms) trước khi giắc tách (≥ 0,5 s thao tác tay).
Không có trạm: vòng hở, K1 không đóng; bảo trì đế dùng nắp giắc có cầu nối 1–3, nắp phải che kín A1–A4 (khi tháo trạm, chân 1
(+12V_CTRL) và 9 (12V_AUX) ở nửa cái trên xe luôn có điện; sau Reset A1/A3 có SERVO+).

**Chống chập vòng (QC2 M1).** E-stop trạm là một kênh (NC1 nối tiếp NC2 chống dính tiếp điểm, không chống chập dây). Chập hai nút
bất kỳ của vòng (vào 1, về 3, DOCK_L 5, DOCK_R 7) hoặc với 12V_AUX (9) là bỏ qua khoá hoặc E-stop trạm mà C4 không cắt. Chặn bằng:
(1) bố trí chân: 1, 3, 5, 7, 9 cách nhau một chân chắn nối GND (2, 4, 6, 8, cầu về chân 16 GND_AUX ngay ở nửa cái), chập sang chân
chắn thì C4/C2 cắt hoặc K1 nhả; chân kề 3/5/7 ở hàng dưới chỉ là để trống, D+/D−, GND; VBUS (10) chỉ kề chân 1 (nguồn) và chắn 2;
(2) E01, E09, E10, E11, AX01 đi ống bọc riêng từng sợi qua vòng dịch vụ (cách điện kép); (3) phép thử nghiệm thu và định kỳ trong
DK-N56: nhấn E-stop trạm, tháo từng khoá, nhấn E-stop cột: K1 phải nhả. Bố trí hàng chân là giả định theo D-sub chuẩn (DK-N55).

Vòng trạm thêm {r(L_tram_loop,2)} m dây 22 AWG và 2 chân tín hiệu 21W4 vào mạch cuộn: sụt toàn mạch cuộn K1 + KH ({r(GL['ctrl_A'],1)} A) ~{r(dV_loop,2)} V
[tự tính, chân 5 mΩ giả định], cuộn K1 còn ~{r(g['Vmin']-dV_loop,1)} V khi pin {r(g['Vmin'],1)} V: điện áp hút của contactor chưa có datasheet (DK-N1, DK-N56).
Bảng trạng thái và bảng nối dây: `bang-di-day-evt.md` mục L.

## L6. Bảng dây từng sợi (`bang-day.csv`, sinh từ `day_L.py`)

{len(rowsL)} sợi, mỗi dòng một sợi: mã, điểm đầu → điểm cuối (đầu nối.chân), tín hiệu, AWG/mm², chiều dài đường đi và chiều dài cắt
(mm), nguồn chiều dài, vòng dịch vụ, màu, đầu cos/giắc hai đầu, cầu chì, dòng liên tục/đỉnh, chịu dòng ×0,82, sụt áp. Hình học ở
khối `GEO` của `day_L.py` (mỗi số ghi nguồn; [ước từ CAD] là kích thước lấy từ CAD/BOM/R1b, chưa đo trên robot). Vòng dịch vụ trục
nâng: chữ U neo giữa cột, dài hành trình/2 + π·r + dư = {r(DL.LOOP*1000,0)} mm (hành trình {r(DL.G['travel'],2)} m, r {r(DL.G['bend_r']*1000,0)} mm).
Vật tư cắt/bấm: `vat-tu-day.csv`.

| Nhóm | Số sợi | AWG | Tổng cắt (m) |
|---|---:|---|---:|
"""
_grp = {}
for x in rowsL:
    gg = _grp.setdefault(x['nhom'], [0, set(), 0])
    gg[0] += 1; gg[1].add(str(x['AWG'])); gg[2] += x['cat_mm']
for k_, (n_, a_, c_) in _grp.items():
    mdB += f"| {k_} | {n_} | {', '.join(sorted(a_, key=lambda t: (len(t), t)))} | {r(c_/1000,2)} |\n"
mdB += f"""
Cầu chì (mỗi servo một cầu chì; quy tắc: ≥ 1,25 × dòng cho phép firmware và ≤ chịu dòng ×0,82 của dây nhỏ nhất phía sau):

| Cầu chì | A | Loại | Cắt (A @ 32 VDC) | Vị trí | Bảo vệ |
|---|---:|---|---:|---|---|
"""
for fid, fz in DL.FUSE.items():
    mdB += f"| {fid} | {r(fz['A'],1).replace(',0','')} | {fz['loai']} | {fz['Icu_A']} | {fz['vi_tri']} | {fz['ghi_chu']} |\n"
mdB += f"""
Isc tại thanh cái hộp FB-L/FB-R trên trạm (L6b), 40T / P45B / giả định hiện hành: {r(IscFB['lo40'],0)} /
{r(IscFB['lo45'],0)} / {r(IscFB['hi'],0)} A: cầu chì lưỡi trong hộp {'dưới' if max(IscFB['lo40'], IscFB['lo45']) <= GL['blade_IR'] else 'VƯỢT'} 1 kA cả với P45B. Cầu chì lưỡi ở
cọc SERVO+ (FW1–3, FLIFT, FLAT) vẫn theo L3 (đạt với 40T, KHÔNG ĐẠT với P45B): giữ DK-N29. Nhánh BAT_F (Pi, mạch dừng, sạc, 12V_AUX) dùng aptomat 6 kA.

STS3095 dùng 7,5 A thay 10 A của bản trước: dòng cho phép lớn nhất {r(max(DL.allowed(n) for n in DL.PL if DL.is95(n)),2)} A × 1,25 =
{r(1.25*max(DL.allowed(n) for n in DL.PL if DL.is95(n)),2)} A ≤ 7,5 A, và hai đuôi 22 AWG song song (2 × {r(DL.amp_d(22),1)} A) chịu được 7,5 A.
STS3215 giữ 3 A ({r(1.25*max(DL.allowed(n) for n in DL.PL if not DL.is95(n)),2)} A ≤ 3 A ≤ {r(DL.amp_d(22),1)} A). Dòng chặn > cầu chì: kẹt lâu thì cầu chì chảy, đúng ý.

Dây chính:

| Dây | Từ → đến | AWG | Dài / cắt (mm) | Đầu cos | Cầu chì | Liên tục / đỉnh (A) | Chịu ×0,82 (A) |
|---|---|---:|---|---|---|---|---:|
"""
for x in rowsL:
    if x['nhom'] in ('trục chính', 'nhánh tay', 'Pi', 'sạc', 'chốt trục nâng') or x['ma'] in ('P.LEFT_PAN+', 'P.LEFT_GRIPPER+', 'P.LIFT+', 'FH.LIFT.i', 'P.W1+', 'C02', 'C13', 'G13', 'AX01', 'AX03', 'E01', 'E09', 'D.LIFT', 'U03'):
        mdB += f"| {x['ma']} | {x['tu']} → {x['den']} | {x['AWG']} | {x['dai_mm']} / {x['cat_mm']} | {x['dau_tu']} / {x['dau_den']} | {x['cau_chi']} {x['cau_chi_A'].replace('.',',')} | {r(x['dong_lien_tuc_A'],2)} / {r(x['dong_dinh_A'],1)} | {r(x['ampacity_x0_82_A'],1) if x['ampacity_x0_82_A'] != '' else ''} |\n"
mdB += f"""
Đi dây servo: nguồn hình sao, mỗi servo một cặp V+/GND từ hộp cầu chì; DATA nối chuỗi theo bus (bo bus → pan → vai → khuỷu →
gập → cuộn → kẹp; bánh 0 → 1 → 2; **trục nâng ở cổng S2 của bo bus đế BUS_B**, D.LIFT 24 AWG qua vòng dịch vụ). Cáp 3 chân giữa hai servo chỉ giữ DATA đi suốt, V+ và
GND cắt giữa, mỗi nửa nối vào cặp nguồn của servo ở đầu đó: không có đường nguồn nối tiếp qua cáp, và STS3095 nhận điện ở cả
hai cổng. USB: Pi → chân 10, 13–15 giắc 21W4 (qua vòng trục nâng) → hub 7 cổng trong hộp điện trạm (tự cấp nguồn 5 V từ 12V_AUX qua buck trạm);
hub nuôi camera đầu, 2 camera cổ tay và, qua 2 module cách ly USB ADuM3160, 2 bo bus tay; bo bus đế cắm thẳng Pi. Hộp điện trạm ở ngoài
tai phải nên dây tay trái dài hơn tay phải {r(DL.G['hop_pan_L']-DL.G['hop_pan_R'],2)} m.

Đi qua vòng dịch vụ trục nâng (chữ U {r(DL.LOOP*1000,0)} mm): BW07/BW07N/BW08/BW08N 4 AWG; P.LIFT± 16 AWG; L03/L04 cuộn chốt, AX01/AX02 12V_AUX,
E01/E09 vòng trạm, E10/E11 DOCK (22 AWG; cùng AX01 mỗi sợi một ống bọc riêng); D.LIFT, G15/G16 (24 AWG); U03 USB. Không cáp nào nối trạm với xe: hai nửa giắc gắn cứng.
Bán kính uốn 4 AWG chỗ vào đế ngàm ≥ 50 mm, kẹp giảm lực ≤ 50 mm trước giắc (R3c mục 4).

**Mất dây về GND nhánh tay (QC2 M1).** Nếu BW07N/BW08N (hoặc tiếp điểm A2/A4) hở mà còn đường GND khác giữa servo tay và Pi (GND USB,
GND_AUX, DATA), tới {r(DL.PK_FBL,1)} A sẽ về qua cáp USB hoặc chân 11 22 AWG: cháy, không phát hiện. Chọn **cắt mọi đường GND thứ hai**
thay vì thêm dây về thứ hai: (1) bo bus tay nằm sau module ADuM3160 (cách ly 1,5 kV, 12 Mbps đủ cho bus 1 Mbps), GND bus nối
GND servo của hộp mình (GB.L, GB.R); hub, buck trạm, camera thuộc miền GND_AUX = GND Pi, không nối GND servo trên trạm;
(2) cuộn chốt đóng bằng rơ-le K10 (nay trên tấm điện), dây cuộn về GNDS; (3) servo trục nâng sang BUS_B, nguồn và GND về thẳng tấm điện.
Dây về tay hở làm servo tay mất nguồn (an toàn khi hỏng) và SDK thấy mất bus. **BUS_B không cách ly** và nay mang thêm trục nâng: đứt
P.LIFT− thì dòng lift về qua DATA/GND USB của BUS_B (giới hạn bởi FLIFT 7,5 A). Còn đo ở EVT hoặc thêm ADuM3160 cho BUS_B: DK-N36.

## L6b. Giắc trạm D-sub combo 21W4

| Chân | Tín hiệu | Bảo vệ | Dây phía xe / phía trạm |
|---|---|---|---|
"""
for k_, (sig_, fz_, w_) in DL.PIN_21W4.items():
    mdB += f"| {k_} | {sig_} | {fz_} | {w_} |\n"
mdB += f"""
- Tiếp điểm nguồn {r(DL.DSUB_A,0)} A [R3b 5.4, chưa kiểm datasheet]: dòng chặn mỗi tay {r(DL.PK_FBL,1)} A ({r(DL.PK_FBL/DL.DSUB_A*100,0)}%), liên tục ~{r(DL.CONT_FBL,1)} A,
  ở giới hạn mô-men {r(arm_allowed,1)} A. F3/F4 MEGA 40 A ≥ 1,25 × {r(arm_allowed,1)} = {r(1.25*arm_allowed,1)} A và {'≥ dòng chặn' if DL.PK_FBL <= 40 else f'THẤP HƠN dòng chặn đồng thời {r(DL.PK_FBL,1)} A (+1 STS3215 mỗi tay; chỉ xảy ra khi cả 7 servo cùng kẹt và bỏ qua TORQUE_LIMIT; cầu chì 40 A khi đó chảy chậm, bảo vệ dây 8 AWG vẫn đạt; tiếp điểm 21W4 ở {r(DL.PK_FBL/DL.DSUB_A*100,0)}% định mức: chờ kỹ sư điện xác nhận hoặc nâng F3/F4 lên nấc 50 A nếu tiếp điểm cho phép, DK-N55)'}, ≤ tiếp điểm 40 A và ≤ 8 AWG ×0,82
  ({r(DL.amp_d(8),1)} A). Tiếp điểm chỉ nhận dây ≤ 8 AWG: 4 AWG dừng ở khối cọc CDK trên đế ngàm, đuôi 8 AWG {r(DL.G['pigtail'],2)} m tới tiếp điểm.
- Chân tín hiệu 5 A: vòng trạm và DOCK dưới FCTRL 4 A, 12V_AUX dưới FAUX 2 A; USB 2.0 qua chân 10, 13–15 là rủi ro (DK-N45).
- Bố trí hàng [giả định]: trên 1–9, dưới 10–17 so le. Không cặp chân kề nào nằm trong nhóm vòng + 12V_AUX (1, 3, 5, 7, 9); test
  `test_giac_21W4_khong_ke_chan_vong` kiểm theo `day_L.dsub_adjacent()`.
- Không cắt dòng tải bằng giắc: mở lẫy khoá là hở vòng E-stop, K1 nhả trước khi tiếp điểm tách.
- Isc tại thanh cái hộp FB trên trạm (sau F3/F4, 4 AWG {r(_RL['BW07']['dai_mm']/1000,2)} m, 8 AWG {r(L8,2)} m, 2 tiếp điểm, đi + về), 40T / P45B / giả định hiện hành:
  {r(IscFB['lo40'],0)} / {r(IscFB['lo45'],0)} / {r(IscFB['hi'],0)} A.

## L7. Vỏ pin 12 cell (ghi chú cho CAD, chưa sửa `cad/`)

- 3S4P xếp 4 × 3: mặt cắt {r(4*cell_d,1)} × {r(3*cell_d,1)} mm, dài ~76 mm gồm nickel [tự tính]; bản 15 cell cũ {r(cross_L,1)} × {r(cross_H,1)} mm.
- BMS Daly 100 A ~184 × 65 × 33 mm [snippet, bản FAN]; bản Smart Bluetooth đo khi có hàng (DK-N2).
- Bỏ khoang công tắc ngắt pin, LOTO M1-PW-018, cọc M8 xuyên nắp; thêm chỗ giắc J0 Anderson SB120 (vỏ ~ 68 × 51 mm [nhớ từ catalogue, kiểm khi có hàng]) và
  cổng sạc XT60E-F trên thành vỏ. Khối cell tối đa {r(ncL*70,0)} g.

## L9. Đế bàn (bộ tuỳ chọn; `bang-day-de-ban.csv`, `vat-tu-de-ban.csv`)

Không thuộc robot chuẩn. Adapter kín 12 V 15 A → thanh +12 V → rơ-le chính KB 40 A → 2 cầu chì ATO 15 A (mỗi tay một) → 21W4 A1/A3;
GND về A2/A4. Vòng trạm (khoá + E-stop trạm, chân 1–3) nối tiếp cuộn rơ-le tự giữ KHB (JQ1P) và cuộn KB; nút Reset song song tiếp điểm
tự giữ, nên nhả E-stop xong phải bấm Reset. 12V_AUX (chân 10/11) lấy trước KB qua cầu chì 2 A: laptop vẫn thấy camera khi nhấn E-stop.
USB chân 10, 13–15 ra laptop. Nửa cái trên đế bàn cũng có chân chắn GND 2, 4, 6, 8 (BGD1–BGD4). Thêm KHB so với R3b (một rơ-le 40 A không tự giữ được qua vòng mà không cho dòng tay đi qua chân tín hiệu).
Hai tụ 4.700 µF 25 V đặt sau KB trên đế bàn, không trên trạm, để không cộng {r(2*4700e-6*1e3,1)} mF vào xung đóng K1 ở chế độ di động (DK-N30).

| Mục | Giá trị |
|---|---:|
| Adapter | 12 V × 15 A = 180 W |
| Mỗi tay ở giới hạn mô-men `di_dong` | {r(arm_allowed,2)} A; hai tay {r(2*arm_allowed,1)} A |
| Ngân sách servo `tram_ban` hiện ở `spec.ROBOT_CONFIGS` | {r(ban_budget,1)} A + 12V_AUX {r(DL.AUX12_A,1)} A = {r((ban_budget+DL.AUX12_A)/15*100,0)}% adapter |
| **Đề xuất** (sửa `spec` ở lượt SDK) | **{r(DL.BAN_I,1)} A** + {r(DL.AUX12_A,1)} A = {r(DL.BAN_I+DL.AUX12_A,1)} A = {r((DL.BAN_I+DL.AUX12_A)/15*100,0)}%; đo bằng thanh ghi dòng STS, quá 0,5 s thì hạ mô-men (R3b mục 6) |
| Cầu chì / dây | BF1, BF2 {r(DL.FUSE_BAN['BF1']['A'],0)} A ATO, 12 AWG ({r(DL.amp_d(12),1)} A); vòng 1 A và 12V_AUX 2 A, 22 AWG; adapter phải có quá dòng ≤ {r(DL.amp_d(14),1)} A (dây ra 14 AWG) |
| Sụt áp adapter → tay (1 m 14 AWG + 0,4 m 12 AWG, đi + về, 12 A) | {r(12*2*(awg_r(14)*1.0+awg_r(12)*0.4)*1e-3,2)} V [tự tính] |
| Rơ-le KB đóng vào tụ | ~12 mF (2 × 4.700 µF + tụ servo hai tay [giả định]): xung ~400 A vài chục µs, nguy cơ dính tiếp điểm, bàn không có ARM_SENSE (DK-N57) |

Ngân sách 12 A là {r(ban_budget/(2*arm_allowed)*100,0)}% mức `di_dong`: chế độ bàn chỉ dùng cho tải nhẹ, SDK phải giới hạn động (DK-N50). Quá dòng adapter,
rơ-le 40 A cắt DC: DK-N57.

## L8. Điều kiện còn mở

Từ trước: DK-N1, DK-N2, DK-N3, DK-N7, DK-N17, DK-N19, DK-S2 (không K1_AUX), DK-N24 (sụt áp), DK-N28,
DK-N30 (đóng K1 không nạp trước). Bảng dây sửa hoặc thêm:

"""
for k_, v_ in DL.DK.items():
    mdB += f"- **{k_}**: {v_}.\n"
mdB += "\n"

md = md.replace("# Tính toán điện M1 (v0.3 cơ khí, chưa duyệt)", "# Phần A (lưu đối chiếu): tính toán điện M1 v0.3, chuỗi A (PCB nguồn, D0, EV200, 3S5P, BMS 200 A)\n\n"
                "> Không dùng cho v0.5. Bảng dây của phần này ở `bang-day-chuoi-a-v03.csv`. Phần L trong `electrical/tinh-toan.md` thay thế.", 1)
md = md.replace("Toàn bộ 21 dây ở `bang-day.csv`", "Toàn bộ 21 dây ở `bang-day-chuoi-a-v03.csv`")
md = md.replace("**THIẾU DỮ LIỆU**", "không chốt (chuỗi A, đã thay)").replace("**Thiếu dữ liệu:**", "Không chốt (chuỗi A, đã thay):").replace("**Thiếu dữ liệu**", "không chốt (chuỗi A, đã thay)")
md = md.replace("Thiếu dữ liệu", "không chốt (chuỗi A, đã thay)")
open("tinh-toan.md","w",encoding="utf-8").write(mdB + "---\n\nPhần A cũ (chuỗi A, v0.3, lưu đối chiếu): `archive/v0.4/electrical/tinh-toan-chuoi-A.md`.\n")
if LUU_A:
    open(os.path.join(ARCH,"tinh-toan-chuoi-A.md"),"w",encoding="utf-8").write(md)
print("arm",arm,"base",base,"tot",tot,"Ik",Ik)
print("I_trunk",I_trunk,"Ipk",Ipk_trunk,"Ides",I_des,"Wh",Wh,"Iavg2h",Iavg2h,"t_run",t_run)
print("Rall mOhm",Rall*1e3,"dv%",dv_all/g['Vnom']*100,"qc2 style %",dv_qc2/g['Vnom']*100)
print("BR",[(x["ma"],round(x["lo"]),round(x["hi"])) for x in BRrows])
print("Isc",Isc_nom,Isc_12,Isc_ub,"I_nopc",I_nopc)
print("t_release diode",t_diode_only(1),"zener noR",t_clamp_noR(1,15.45),"zener R",t_clamp(1,15.45))
print("delay",t_delay(0.65),t_delay(1.05),t_delay(1.45),"por",t_por(1.5),t_por(2.0),"t4",t4)
print("all cont ok", all(x['dat_dong_lien_tuc']=='co' for x in rows), [x['ma'] for x in rows if x['cau_chi_nho_hon_ampacity_ha_nhiet']=='KHONG'])
