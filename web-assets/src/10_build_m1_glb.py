"""Dựng web-assets/m1.glb + joints.json + cum.json từ CHÍNH sim/assets/*.stl (R7) và cây body/khớp của sim/m1.xml. Không đọc STEP, không import build123d.
Dùng: python 10_build_m1_glb.py REPO LABEL_NPZ OUT_DIR [RATIO] [ROLLER_TRI] [SERVO_TRI] [POWER_RATIO] [MIN_TRI] [CREASE_DEG]
  REPO       = bản sao repo (sim/, sdk/, product/)
  LABEL_NPZ  = parts_zero.npz (tên chi tiết CAD + hộp bao, chỉ dùng để GÁN NHÃN cụm/mã M1 cho từng thành phần liên thông của STL; hình học luôn lấy từ STL)
               xuất cùng lượt với sim/assets: python cad/faibotic/export_sim_meshes.py --labels parts_zero.npz (cùng CAD, cùng tư thế 0)
Mesh đặt vào body theo đúng quy tắc của sim (sdk/faibotic/model.mesh_place): giá + chốt J5 quay cùng khớp gập (node wrist_flex) như sim/m1.xml.
Node GLB = body MJCF có khớp (tên = tên khớp SDK), gốc node = pos body trong m1.xml. Node lá <cum>.<node>.<vật liệu> mang mesh. Đơn vị m; node m1 quay CAD (z lên) -> glTF Y-up."""
import sys, os, re, json, math, csv, hashlib, subprocess
import numpy as np, trimesh, fast_simplification, pygltflib as G
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as Rot
repo, labnpz, out = sys.argv[1:4]; RATIO = float(sys.argv[4]) if len(sys.argv) > 4 else 0.2; ROLLER = int(sys.argv[5]) if len(sys.argv) > 5 else 80
SERVO_TRI = int(sys.argv[6]) if len(sys.argv) > 6 else 260     # v0.4.6: tam giác tối đa mỗi thành phần thân servo (vật liệu servo_body) trong GLB
POWER_RATIO = float(sys.argv[7]) if len(sys.argv) > 7 else 0.3
MIN_TRI = int(sys.argv[8]) if len(sys.argv) > 8 else 100
CREASE_DEG = float(sys.argv[9]) if len(sys.argv) > 9 else 30.0
CAD_MESH = sys.argv[10] if len(sys.argv) > 10 else None     # v0.5d: thư mục mesh CAD từng chi tiết (assembly_pl.py --lift -160 --mesh DIR) để thay trụ cột
URDF_SERVO_TRI = int(sys.argv[12]) if len(sys.argv) > 12 else 0   # tam giác tối đa mỗi thân servo lấy từ mesh URDF (0 = giữ nguyên); 6+1 thêm 2 servo yaw, rút để vừa brotli 1,2 MB
COL_TRI = int(sys.argv[11]) if len(sys.argv) > 11 else 3000  # tam giác mỗi trụ (M1-LF-051..055) lấy từ mesh CAD   # v0.5b: góc gấp pháp tuyến (độ)      # S1G: tam giác tối thiểu mỗi thành phần > 160 tam giác  # v0.4.6: cụm power nằm kín trong thân đế (chỉ thấy khi tách lớp): mọi thành phần rút còn 30% (tối thiểu 24)
sys.path.insert(0, os.path.join(repo, "sdk")); from faibotic import spec, model as SM
ASSETS = os.path.join(repo, "sim", "assets"); MJ = json.load(open(os.path.join(ASSETS, "meshes.json")))
FR = {k: np.array(v) for k, v in MJ["source"]["frames"].items()}
root = ET.parse(os.path.join(repo, "sim", "m1.xml")).getroot().find("worldbody")
# ---- cây node từ m1.xml: body có khớp được dùng làm node (bỏ base_x/y/yaw, bánh) ----
BODY2NODE = {"base": "base", "carriage": "lift"}
for s in ("left", "right"):
    for b, j in ((f"{s}_shoulder", f"{s}_pan"), (f"{s}_upper", f"{s}_shoulder"), (f"{s}_fore", f"{s}_elbow"), (f"{s}_wrist", f"{s}_wrist_flex"), (f"{s}_yaw", f"{s}_wrist_yaw"), (f"{s}_hand", f"{s}_wrist_roll"), (f"{s}_jaw", f"{s}_gripper")): BODY2NODE[b] = j
NODE2BODY = {v: k for k, v in BODY2NODE.items()}
T = {"m1": (None, np.zeros(3)), "base": ("m1", np.zeros(3))}; XJ = {}
def walk(b, parent_body):
    for c in b.findall("body"):
        n = c.get("name")
        if n in BODY2NODE:
            pos = np.array([float(x) for x in (c.get("pos") or "0 0 0").split()]); 
            T[BODY2NODE[n]] = (BODY2NODE[parent_body], pos); j = [x for x in c.findall("joint") if x.get("name") == (n if n.endswith("finger_a") else None) or x.get("name") == (BODY2NODE[n] if BODY2NODE[n] != n else n) or x.get("name") in (BODY2NODE[n], n)]
            XJ[BODY2NODE[n]] = [(x.get("name"), x.get("type") or ("slide" if x.get("class") == "finger" else "hinge"), [float(v) for v in x.get("axis").split()], x.get("range")) for x in c.findall("joint")]
            walk(c, n)
        elif n.startswith("wheel"): continue
bb = root.find("body"); walk(bb, "base"); XJ["base"] = []
# gốc node ở tư thế 0 (m, world) = cộng dồn pos
W0 = {"m1": np.zeros(3)}
def world(n):
    if n not in W0: p, o = T[n]; W0[n] = world(p) + (o if p != "m1" else 0)
    return W0[n]
for n in T: world(n)
assert all(np.allclose(W0[BODY2NODE[b]], FR[b], atol=1e-9) for b in BODY2NODE), "gốc body m1.xml khác meshes.json frames"
# ---- nhãn: hộp bao chi tiết CAD (world, mm) ----
d = np.load(labnpz); lnames = list(d["names"]); lmats = list(d["mats"]); lbbs = d["bbs"] / 1000.0
LBODY = dict(zip(lnames, list(d["bodies"])))      # S1G (P-L): body ghi cùng lượt xuất (export_sim_meshes.py --pl --labels)
SPECIAL = {"M1-LF-lift_servo": "M1-SV-002"}
def code_of(n):
    """P-L: tên chi tiết bắt đầu bằng mã M1 (assembly_pl.py, export_sim_meshes.main_pl); servo trục nâng STS3095 = M1-SV-002."""
    if n in SPECIAL: return SPECIAL[n]
    m = re.match(r"^(M1-[A-Z]{2}-\d{3})", n)
    if not m: raise SystemExit("chua co ma: " + n)
    return m.group(1)
def body_of(n): return LBODY.get(n)
def side_of(n): return "L" if n.endswith("_L") else "R" if n.endswith("_R") else None
def cum_of(body, code, name):
    if code.startswith("M1-CM"): return "camera"                     # camera ngực (khung T) + camera cổ tay (kẹp)
    if code == "M1-BS-038": return "power"                           # khoang pin 12 V, Pi 5 + bus adapter + E-stop (khối giữ chỗ)
    if body == "base": return "lift" if code.startswith("M1-LF-05") else "base"      # trụ AlohaMini2 (thanh răng) thuộc cụm nâng
    if body == "carriage": return ("shoulder_" + side_of(name)) if code == "M1-SH-005" else "lift"     # đế J1 hai tay trên khung T
    s = "L" if body.startswith("left") else "R"; j = body.split("_", 1)[1]
    return ("shoulder_" if j in ("shoulder", "upper") else "arm_" if j in ("fore", "wrist", "yaw") else "gripper_") + s
# ---- đọc STL, tách thành phần liên thông, gán nhãn bằng hộp bao, decimate ----
def clean(v, f):
    m = trimesh.Trimesh(v, f, process=False); m.update_faces(m.area_faces > 1e-12); m.remove_unreferenced_vertices(); return np.asarray(m.vertices), np.asarray(m.faces)
def smooth(v, f, crease=None):
    """v0.5b: pháp tuyến mượt theo góc gấp (mặc định CREASE_DEG): ở mỗi đỉnh, góc tam giác lấy trung bình (theo diện tích) pháp tuyến các mặt kề
    lệch <= góc gấp so với mặt của nó; đỉnh chỉ tách khi pháp tuyến khác nhau (cạnh sắc giữ sắc, mặt cong decimate không còn gãy khúc).
    Ít đỉnh tách hơn trimesh.smooth_shaded (tách theo facet), nên GLB nhẹ hơn ở cùng số tam giác."""
    v, f = clean(v, f); m = trimesh.Trimesh(v, f, process=True); m.update_faces(m.area_faces > 1e-12); m.remove_unreferenced_vertices()
    V, F = np.asarray(m.vertices), np.asarray(m.faces); fn = np.asarray(m.face_normals, float); fa = np.asarray(m.area_faces, float)
    cth = math.cos(math.radians(CREASE_DEG if crease is None else crease))
    cv = F.reshape(-1); cf = np.repeat(np.arange(len(F)), 3); order = np.argsort(cv, kind="stable"); cn = np.zeros((len(cv), 3))
    starts = np.flatnonzero(np.r_[True, cv[order][1:] != cv[order][:-1]]); ends = np.r_[starts[1:], len(order)]
    for s, e in zip(starts, ends):
        idx = order[s:e]; fs = cf[idx]; N = fn[fs]; W = N * fa[fs, None]
        cn[idx] = ((N @ N.T) >= cth) @ W      # mỗi góc: tổng có trọng số các mặt kề trong góc gấp (gồm chính nó)
    cn /= np.maximum(np.linalg.norm(cn, axis=1, keepdims=True), 1e-12)
    key = np.column_stack([cv, np.round(cn * 2e3).astype(np.int64)])      # góc cùng đỉnh, cùng pháp tuyến (lượng tử ~0,03 độ) thì gộp
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return V[cv[first]], inv.reshape(-1, 3), cn[first]
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
def components(V, F):
    """Tách thành phần liên thông theo đỉnh dùng chung (không cần networkx)."""
    n = len(V); r = np.repeat(F[:, 0], 2); c = F[:, [1, 2]].reshape(-1)
    k, lab = connected_components(coo_matrix((np.ones(len(r)), (r, c)), shape=(n, n)), directed=False); fl = lab[F[:, 0]]
    for i in range(k):
        fi = F[fl == i]
        if len(fi) == 0: continue
        u, inv = np.unique(fi, return_inverse=True); yield V[u], inv.reshape(-1, 3)
groups = {}; stat = dict(tri_in=0, tri_out=0); TRI_CUM = {}; NCOMP = {}
COL_CODES = ("M1-LF-051", "M1-LF-052", "M1-LF-053", "M1-LF-054", "M1-LF-055")
def URDF_SKIP(body, code):
    """v0.5c: thành phần sim thay bằng mesh link URDF: J1 (M1-SH-005, trên xe nâng) và mọi thứ ở body vai, cánh tay, cẳng tay (J2, J3, J4, servo gập)."""
    if CAD_MESH and body == "base" and code.startswith(COL_CODES): return True      # v0.5d: trụ cột lấy từ mesh CAD (sim đã gộp lưới 1.200 tam giác, cạnh răng cưa)
    if body.startswith(("left_", "right_")) and body.split("_", 1)[-1] in ("wrist", "yaw", "hand", "jaw"): return True     # v0.5e: cổ tay/kẹp từ URDF; (6+1): servo yaw cũng từ URDF, không còn khoá yaw
    return (body == "carriage" and code == "M1-SH-005") or body.split("_", 1)[-1] in ("shoulder", "upper", "fore") and body.startswith(("left_", "right_"))
def SERVO_OF(n):
    """Tên chi tiết servo trong STL sim -> khoá SDK (spec.SERVO_IDS)."""
    if n == "M1-LF-lift_servo": return "lift"
    m = re.match(r"^M1-SV-00\d_(wrist_flex|wrist_yaw|wrist_roll|gripper|wheel)(\d)?(?:_([LR]))?$", n)
    if not m: raise SystemExit("servo khong ro khop: " + n)
    return f"wheel_{m.group(2)}" if m.group(1) == "wheel" else ("left_" if m.group(3) == "L" else "right_") + m.group(1)
SERVO_MODEL = {k: ("STS3095" if k == "lift" or k.split("_", 1)[-1] in ("pan", "shoulder", "elbow") else "STS3215") for k in spec.SERVO_IDS}
_all = np.concatenate([np.asarray(trimesh.load(os.path.join(ASSETS, e["file"]), process=False).vertices) + FR[e["body"]] for e in MJ["meshes"]]); GB = np.concatenate([_all.min(0), _all.max(0)]); unl = []; parts = {}
for e in MJ["meshes"]:
    body, mat = e["body"], e["material"]; sha = hashlib.sha256(open(os.path.join(ASSETS, e["file"]), "rb").read()).hexdigest(); assert sha == e["sha256"], e["file"]
    pb, pp = SM.mesh_place(e, MJ["source"]["frames"]); pp = np.array(pp)       # body gắn + vị trí gốc mesh trong body, như sim/m1.xml
    m = trimesh.load(os.path.join(ASSETS, e["file"]), process=True); node = BODY2NODE[pb]; o = FR[body]
    cand = [i for i, n in enumerate(lnames) if lmats[i] == mat and body_of(n) == body]
    for v, f in components(np.asarray(m.vertices), np.asarray(m.faces)):
        stat["tri_in"] += len(f)
        lo, hi = v.min(0) + o, v.max(0) + o; sc_ = []
        for i in cand:
            b = lbbs[i]; ov = np.maximum(0, np.minimum(hi + 5e-4, b[3:] + 5e-4) - np.maximum(lo - 5e-4, b[:3] - 5e-4)).prod(); vol = max(np.prod(hi - lo + 1e-3), 1e-12)
            sc_.append((ov / vol, np.prod(b[3:] - b[:3] + 1e-3), i))
        # phần thể tích hộp bao thành phần nằm trong hộp nhãn; các nhãn cách nhau <= 0,05 thì lấy hộp nhãn nhỏ nhất
        # (v0.4.4: vỏ pin + nắp dính nhau, nhô 3 mm khỏi hộp vỏ, không để thân đế bao ngoài giành nhãn)
        top = max(x[0] for x in sc_); _, _, i = min((x for x in sc_ if x[0] >= top - 0.05), key=lambda x: x[1]); best = (top, i)
        code = code_of(lnames[i]); cm = cum_of(body, code, lnames[i])
        if URDF_SKIP(body, code): parts.setdefault((cm, code), set()).add(lnames[i]); continue      # v0.5c: J1-J4 lấy từ mesh link URDF (dưới)
        if best[0] < 0.5:      # thành phần liên thông gộp nhiều chi tiết chạm nhau: gán thêm mã của mọi chi tiết phủ >= 50% thể tích hộp bao của chính nó
            unl.append((e["file"], round(best[0], 2), lnames[i]))
            for k in cand:
                b = lbbs[k]; ov = np.maximum(0, np.minimum(hi + 5e-4, b[3:] + 5e-4) - np.maximum(lo - 5e-4, b[:3] - 5e-4)).prod()
                if ov / max(np.prod(b[3:] - b[:3] + 1e-3), 1e-12) >= 0.5: parts.setdefault((cum_of(body, code_of(lnames[k]), lnames[k]), code_of(lnames[k])), set()).add(lnames[k])
        parts.setdefault((cm, code), set()).add(lnames[i])
        nf = len(f); tgt = min(nf, SERVO_TRI) if mat == "servo_body" else max(24, min(nf, int(nf * POWER_RATIO))) if cm == "power" else ROLLER if "roller" in lnames[i] else (nf if nf <= 160 else max(MIN_TRI, int(nf * RATIO)))   # P-L: mọi chi tiết > 160 tam giác rút theo RATIO
        edge = bool((np.abs(lo - GB[:3]) < 1e-3).any() or (np.abs(hi - GB[3:]) < 1e-3).any())     # thành phần chạm hộp bao toàn robot
        while tgt < nf:     # S1G: thành phần ở biên robot giữ hộp bao (lệch <= 0,8 mm), không thì tăng số tam giác đích 25%
            v2, f2 = fast_simplification.simplify(v, f, target_reduction=1 - tgt / nf, agg=6)
            if len(f2) >= 4 and (not edge or np.abs(np.concatenate([v2.min(0) - v.min(0), v2.max(0) - v.max(0)])).max() <= 8e-4): v, f = v2, f2; break
            if len(f2) < 4: break
            tgt = int(tgt * 1.25) + 1
        vv, ff, nn = smooth(v, f); stat["tri_out"] += len(ff); vv = vv + pp; TRI_CUM[cm] = TRI_CUM.get(cm, 0) + len(ff); NCOMP[cm] = NCOMP.get(cm, 0) + 1
        sv = SERVO_OF(lnames[i]) if mat == "servo_body" else ""       # v0.5c: mỗi servo một node lá riêng (đếm được, đối chiếu BOM)
        g = groups.setdefault((node, cm, mat, sv), [[], [], [], 0]); g[0].append(vv); g[1].append(ff + g[3]); g[2].append(nn); g[3] += len(vv)
for i, n in enumerate(lnames):      # mọi chi tiết CAD có trong STL (cùng lượt xuất) đều có mã trong cụm của nó, kể cả chi tiết dính vào thành phần liên thông khác (nắp pin M1-PW-014)
    b_ = body_of(n)
    if b_ is not None and any(e["body"] == b_ and e["material"] == lmats[i] for e in MJ["meshes"]): parts.setdefault((cum_of(b_, code_of(n), n), code_of(n)), set()).add(n)
# ---- v0.5c: J1-J4 tay lấy mesh link URDF upstream AM-ARM200 follower (có sẵn thân servo, tai bắt, ổ bi, bu lông; Apache-2.0, liyiteng/AM-ARM 5b03af1) ----
# cad/upstream/AM-ARM/am-arm200/urdf/derived/ (tách + rút gọn bằng cad/faibotic/extract_urdf_servos.py), đặt bằng FK URDF ở tư thế 0 SDK, đúng phép đặt
# export_sim_meshes.pl_urdf_to_fore dùng cho STEP: cùng động học với SDK/MJCF (URDF follower = nguồn hằng số spec.py). CHỈ GLB: sim/MJCF vẫn dùng STEP follower,
# chưa có thân servo pan/vai/khuỷu (DK-N62). (6+1): cổ tay, kẹp, servo cuộn/yaw/kẹp từ mesh URDF (wrist_yaw là node riêng).
URDF_ARM = {"left_Base": ("lift", "shoulder_L", "ivory"), "left_Rotation_Pitch": ("left_pan", "shoulder_L", "navy"),
            "left_Upper_Arm": ("left_shoulder", "shoulder_L", "ivory"), "left_Lower_Arm": ("left_elbow", "arm_L", "navy"),
            # v0.5e: cổ tay J5/J56, kẹp J7 thân + hàm, camera cổ tay. (6+1): link wrist_yaw (vỏ J56 + servo cuộn + servo yaw) -> node wrist_yaw
            "left_Wrist_Pitch_Roll": ("left_wrist_flex", "arm_L", "ivory"), "left_wrist_yaw": ("left_wrist_yaw", "arm_L", "navy"),
            "left_Fixed_Jaw": ("left_wrist_roll", "gripper_L", "ivory"), "left_Moving_Jaw": ("left_gripper", "gripper_L", "navy"), "left_camera": ("left_wrist_roll", "camera", "servo")}
SERVO_NODE = {"left_gripper": ("left_wrist_roll", "gripper_L")}     # servo kẹp nằm trong mesh wrist_yaw nhưng quay theo khớp cuộn; servo cuộn + yaw ở node wrist_yaw
DER = os.path.join(repo, "cad", "upstream", "AM-ARM", "am-arm200", "urdf", "derived"); DERJ = json.load(open(os.path.join(DER, "derived.json")))
sys.path.insert(0, os.path.join(repo, "sim", "tools")); import am_arm200_pl as UA
_a = math.radians(spec.UPPER_URDF_DEG); UT1 = UA.fk({"left_shoulder_lift": -_a, "left_elbow_flex": _a, "left_gripper": spec.GRIPPER_CLOSE_URDF}); _el1 = UA.R_SDK @ UT1["left_Lower_Arm"][1]     # hàm ở góc khép = kẹp 0 của SDK
def urdf_to_world(link, v):
    R1, p1 = UT1[link]; return (v @ R1.T + p1) @ UA.R_SDK.T - _el1 + W0["left_elbow"]     # khung link URDF (m) -> thế giới M1, tay trái, tư thế 0
for link, (node0, cm0, mat0) in URDF_ARM.items():
    files = [(f"{link}__body.stl", mat0, ""), (f"{link}__metal.stl", "steel", "")] + [(f"{link}__servo__{k}.stl", "servo_body", k) for k in DERJ["link"][link]["servo"]]
    for fn, mat, svk in files:
        fp = os.path.join(DER, fn)
        if not os.path.exists(fp): continue
        m = trimesh.load(fp, process=True); vw = urdf_to_world(link, np.asarray(m.vertices)); F0 = np.asarray(m.faces)
        if svk and URDF_SERVO_TRI and len(F0) > URDF_SERVO_TRI:     # rút quadric thân servo URDF (không gộp lưới)
            vw, F0 = fast_simplification.simplify(vw, F0, target_reduction=1 - URDF_SERVO_TRI / len(F0), agg=7)
        n0, c0 = SERVO_NODE.get(svk, (node0, cm0))
        for s, k in (("left", 1), ("right", -1)):
            node = n0 if n0 == "lift" else n0.replace("left", s); cm = c0 if c0 == "camera" else c0[:-1] + ("L" if k == 1 else "R")
            v = vw * [1, k, 1]; f = F0 if k == 1 else F0[:, ::-1]          # tay phải = gương y (như MJCF), lật thứ tự đỉnh
            vv, ff, nn = smooth(v - W0[node], f); sv = svk.replace("left", s) if svk else ""
            g = groups.setdefault((node, cm, mat, sv), [[], [], [], 0]); g[0].append(vv); g[1].append(ff + g[3]); g[2].append(nn); g[3] += len(vv)
            TRI_CUM[cm] = TRI_CUM.get(cm, 0) + len(ff); stat["tri_out"] += len(ff); NCOMP[cm] = NCOMP.get(cm, 0) + 1
            if svk: parts.setdefault((cm, "M1-SV-002" if SERVO_MODEL[sv] == "STS3095" else "M1-SV-001"), set()).add(f"{SERVO_MODEL[sv]}_{sv}")
if CAD_MESH:      # v0.5d: trụ AlohaMini2 (node base, cụm lift) từ mesh CAD ở tư thế 0 (mm, thế giới), rút quadric còn COL_TRI, không gộp lưới
    for n, mat in json.load(open(os.path.join(CAD_MESH, "manifest.json"))):
        if not n.startswith(COL_CODES): continue
        m = trimesh.load(os.path.join(CAD_MESH, n + ".stl"), process=True); v, f = np.asarray(m.vertices) / 1000.0, np.asarray(m.faces)
        if len(f) > COL_TRI:
            v, f = fast_simplification.simplify(v, f, target_reduction=1 - COL_TRI / len(f), agg=7)
        vv, ff, nn = smooth(v - W0["base"], f); cm = "lift"
        g = groups.setdefault(("base", cm, mat, ""), [[], [], [], 0]); g[0].append(vv); g[1].append(ff + g[3]); g[2].append(nn); g[3] += len(vv)
        TRI_CUM[cm] = TRI_CUM.get(cm, 0) + len(ff); stat["tri_out"] += len(ff)
print("thanh phan nhan do tin cay thap:", len(unl), unl[:5]); print("tam giac theo cum", TRI_CUM, "so thanh phan", NCOMP)
# ---- GLB ----
MAT = {"ivory": ("#f1efe8", .5, 0), "ivory_hi": ("#f6f5f1", .45, 0), "navy": ("#1e3a5f", .42, 0), "alu": ("#b3b7bd", .40, .85), "steel": ("#8d939b", .35, 1.0), "servo": ("#1b1d21", .62, 0), "servo_body": ("#2b2d31", .38, .35), "pad": ("#33363d", .75, 0), "rubber": ("#26282d", .8, 0), "hub": ("#6d727a", .42, .9)}
def lin(h): return [round(((int(h[i:i + 2], 16) / 255) ** 2.2), 4) for i in (1, 3, 5)] + [1.0]
gl = G.GLTF2(asset=G.Asset(version="2.0", generator="faibotic web-assets/src/10_build_m1_glb.py", copyright="Copyright (c) 2026 Leo Dao (Faibotic). Faibotic-drawn parts: CC BY-NC-SA 4.0. Contains parts derived from AM-ARM200 and AlohaMini2 by liyiteng, and the STS3215 mesh of MuJoCo Menagerie robotstudio_so101 (Apache-2.0, modified: tessellated, decimated, regrouped by joint). See NOTICE."), scene=0, scenes=[G.Scene(nodes=[0])])
matidx = {}
for k, (h, r, me) in MAT.items(): matidx[k] = len(gl.materials); gl.materials.append(G.Material(name=k, pbrMetallicRoughness=G.PbrMetallicRoughness(baseColorFactor=lin(h), metallicFactor=me, roughnessFactor=r)))
blob = bytearray()
def add(arr, target, comp, typ):
    while len(blob) % 4: blob.append(0)
    bv = len(gl.bufferViews); gl.bufferViews.append(G.BufferView(buffer=0, byteOffset=len(blob), byteLength=arr.nbytes, target=target)); blob.extend(arr.tobytes())
    mn = mx = None
    if typ == "VEC3" and comp == G.FLOAT: mn, mx = arr.min(0).tolist(), arr.max(0).tolist()
    gl.accessors.append(G.Accessor(bufferView=bv, componentType=comp, count=len(arr) if typ != "SCALAR" else arr.size, type=typ, min=mn, max=mx)); return len(gl.accessors) - 1
cad2gl = Rot.from_matrix([[0, -1, 0], [0, 0, 1], [-1, 0, 0]]).as_quat().tolist()
idx = {}
for n in list(T):
    par, o = T[n]
    node = G.Node(name=n, translation=(o.tolist() if par else None) if par != "m1" or n in ("base",) else None, rotation=cad2gl if n == "m1" else None, children=[])
    if par == "m1" and n != "base": node.translation = o.tolist()
    leaves = []
    for (gn, cm, m, sv), (vs, fs, ns, _) in sorted(groups.items()):
        if gn != n: continue
        v = np.concatenate(vs).astype(np.float32); f = np.concatenate(fs); nn = np.concatenate(ns).astype(np.float32)
        f = f.astype(np.uint16) if len(v) < 65535 else f.astype(np.uint32)
        pr = G.Primitive(attributes=G.Attributes(POSITION=add(v, G.ARRAY_BUFFER, G.FLOAT, "VEC3"), NORMAL=add(nn, G.ARRAY_BUFFER, G.FLOAT, "VEC3")), indices=add(f.reshape(-1), G.ELEMENT_ARRAY_BUFFER, G.UNSIGNED_SHORT if f.dtype == np.uint16 else G.UNSIGNED_INT, "SCALAR"), material=matidx[m])
        nm = f"{cm}.{n}.{m}" + (f".{sv}" if sv else ""); ex = dict(cum=cm, joint_node=n, material=m)
        if sv: ex.update(servo=sv, servo_id=spec.SERVO_IDS[sv], model=SERVO_MODEL[sv])
        gl.meshes.append(G.Mesh(name=nm, primitives=[pr])); leaves.append(G.Node(name=nm, mesh=len(gl.meshes) - 1, extras=ex))
    idx[n] = len(gl.nodes); gl.nodes.append(node)
    for lf in leaves: node.children.append(len(gl.nodes)); gl.nodes.append(lf)
for n in T:
    if T[n][0]: gl.nodes[idx[T[n][0]]].children.append(idx[n])
for nd in gl.nodes:
    if not nd.children: nd.children = None
gl.buffers = [G.Buffer(byteLength=len(blob))]; gl.set_binary_blob(bytes(blob)); os.makedirs(out, exist_ok=True); gl.save_binary(os.path.join(out, "m1.glb"))
size = os.path.getsize(os.path.join(out, "m1.glb")); print("m1.glb", size, "byte; tri", stat, "ratio", round(stat["tri_out"] / stat["tri_in"], 3), "nodes", len(gl.nodes))
# ---- joints.json: trục/loại từ m1.xml, giới hạn từ spec.py ----
deg = math.degrees; J = {}; nodes = {}
for n, (par, o) in T.items():
    nodes[n] = dict(parent=par, sim_body=NODE2BODY.get(n), origin_rel_m=[round(float(x), 6) for x in (o if par and par != "m1" or n == "base" else (o if par else np.zeros(3)))] if n not in ("m1",) else [0, 0, 0], origin_world_home_m=[round(float(x), 6) for x in W0[n]])
nodes["m1"]["note"] = "Gốc cảnh: quay CAD (x trước, y trái, z lên) -> glTF Y-up (X_gl = -y, Y_gl = z, Z_gl = -x). Mọi node con giữ hệ CAD/MJCF."
def jd(name, node, jt, ax, rng, unit, extra):
    d = dict(node=node, sim_joint=name, type="prismatic" if jt == "slide" else "revolute", axis=ax, sdk_unit=unit, **extra); return d
for jn in ["lift"] + [f"{s}_{j}" for s in ("left", "right") for j in spec.ARM_SERVO_JOINTS]:
    node = "lift" if jn == "lift" else jn; xj = {x[0]: x for x in XJ[node]}; nm, jt, ax, rng = xj[jn]
    if jn == "lift": J[jn] = jd(nm, node, jt, ax, rng, "m", dict(limit_soft=list(spec.LIFT_RANGE), note="Trượt z của node lift (xe nâng + mọi tay). Cột, ray, vitme, hộp số ở node base."))
    elif jn.endswith("gripper"):
        J[jn] = jd(nm, node, jt, ax, rng, "pct 0..100 (0 khép, 100 mở)", dict(limit_soft=[0.0, 100.0], open_rad=spec.GRIPPER_OPEN_RAD, mapping="góc_rad = pct/100 * open_rad quanh axis",
                   max_tip_gap_m=spec.GRIPPER_MAX_GAP,
                   note="Kẹp gốc AM-ARM200: hàm động (node <bên>_gripper, gốc tại trục kẹp) quay quanh axis; thân kẹp và ngón cố định nằm ở node <bên>_wrist_roll. 0 = hai ngón chạm, 100 = mở 64 độ (đầu ngón cách ~97 mm)."))
    else:
        j = jn.split("_", 1)[1]; lo, hi = spec.ARM_LIMITS[j]
        sg = {"pan": "dương = ngược chiều kim đồng hồ nhìn từ trên", "shoulder": "dương = gập xuống; 0 = duỗi ngang ra trước", "elbow": "dương = gập xuống; giới hạn trên theo vai (couplings.elbow_max)", "wrist_flex": "dương = gập xuống; giới hạn trên theo vai và khuỷu (couplings.wrist_max)", "wrist_yaw": "quanh +z khung cổ tay (vuông góc trục gập và trục cuộn), cùng dấu hai tay (như MJCF); 6+1, góc URDF upstream = -góc SDK; tầm theo gập (couplings.wrist_yaw)", "wrist_roll": "quanh +x, cùng dấu hai tay (như MJCF); trục đặt trên trục cổ tay theo SDK"}[j]
        J[jn] = jd(nm, node, jt, ax, rng, "rad", dict(limit_soft=[lo, hi], limit_soft_deg=[round(deg(lo), 3), round(deg(hi), 3)], sign=sg))
S, E = spec.WRIST_COUPLE_SHOULDER_DEG, spec.WRIST_COUPLE_ELBOW_DEG
C = dict(note="Bảng giới hạn ghép (độ), nội suy như spec.elbow_max / spec.wrist_max (xem limits.mjs).", pan_soft_max_deg=spec.PAN_SOFT_MAX_DEG,
         elbow_max=dict(shoulder_deg=list(spec.ELBOW_COUPLE_SHOULDER_DEG), max_deg=list(spec.ELBOW_COUPLE_MAX_DEG)), wrist_max=dict(shoulder_deg=list(S), elbow_deg=list(E), max_deg=[list(r) for r in spec.WRIST_COUPLE_MAX_DEG]),
         WRIST_COUPLE_MAX_DEG=[list(r) for r in spec.WRIST_COUPLE_MAX_DEG], WRIST_COUPLE_SHOULDER_DEG=list(S), WRIST_COUPLE_ELBOW_DEG=list(E), ELBOW_COUPLE_SHOULDER_DEG=list(spec.ELBOW_COUPLE_SHOULDER_DEG), ELBOW_COUPLE_MAX_DEG=list(spec.ELBOW_COUPLE_MAX_DEG),
         wrist_yaw=dict(flex_deg=list(spec.WRIST_YAW_COUPLE_FLEX_DEG), min_deg=list(spec.WRIST_YAW_COUPLE_MIN_DEG), max_deg=list(spec.WRIST_YAW_COUPLE_MAX_DEG),
                        note="tầm yaw (độ SDK) theo gập cổ tay, hẹp nhất hai nút kề như spec.wrist_yaw_range; trong limit_soft của wrist_yaw"),
         unscanned="Cuộn cổ tay khác 0, kẹp, cáp chưa quét va chạm (Chưa có số liệu, theo spec.py).")
def pose(qs, qe, qw, lift=0.0, grip=100.0):
    dd = {}
    for s in ("left", "right"): dd.update({f"{s}_pan": 0.0, f"{s}_shoulder": math.radians(qs), f"{s}_elbow": math.radians(qe), f"{s}_wrist_flex": math.radians(qw), f"{s}_wrist_yaw": 0.0, f"{s}_wrist_roll": 0.0, f"{s}_gripper": grip})
    dd["lift"] = lift; return {k: round(v, 6) for k, v in dd.items()}
h = spec.HOME_ARM
P = dict(home=dict(desc="Tư thế gốc SDK (spec.HOME_ARM), lift 0", q=pose(deg(h["shoulder"]), deg(h["elbow"]), deg(h["wrist_flex"]))), ready=dict(desc="Sẵn sàng (vai -30, khuỷu 70, cổ tay 10), lift 0,20 m, như bộ render", q=pose(-30, 70, 10, 0.2)),
         straight=dict(desc="Tay duỗi thẳng, lift 0,25 m", q=pose(0, 0, 0, 0.25)), reach=dict(desc="Với tới (-5, 30, 10), lift 0", q=pose(-5, 30, 10)), rest=dict(desc="Nghỉ (30, 100, 20), lift 0,45 m (đỉnh P-L)", q=pose(30, 100, 20, 0.45)))
doc = dict(schema="faibot-m1-joints/2", units=dict(length="m", angle="rad (limit_soft_deg: độ)"), source=dict(mesh="sim/assets (R7, meshes.json)", tree="sim/m1.xml", limits="sdk/faibotic/spec.py"), nodes=nodes, joints=J, couplings=C, poses=P,
           tcp=dict(note="TCP SDK (spec.py): tâm kẹp cách gốc khớp cuộn LT - LW dọc trục cuộn, lệch (TCP_DY, TCP_DZ) khỏi trục cuộn khi cuộn 0; tay phải lấy gương y. local_in_wrist_roll_m là tay trái, by_side cho từng tay.",
                    local_in_wrist_roll_m=[round(spec.LT - spec.LW, 6), round(spec.TCP_DY, 6), round(spec.TCP_DZ, 6)],
                    by_side={s: [round(spec.LT - spec.LW, 6), round(k * spec.TCP_DY, 6), round(spec.TCP_DZ, 6)] for s, k in (("left", 1), ("right", -1))}),
           ghi_chu=["P-L (S1G): gốc tay (đáy J1, trục pan) ở (0,0599; ±0,18726; 0,4137 + lift) m theo khớp URDF AlohaMini2; trục vai cao hơn 0,14823 và ra trước 0,05799 (URDF AM-ARM200). Lift SDK 0..0,45 = lift CAD -160..+290 mm.",
                    "Tay AM-ARM200 6+1 (như bản gốc): khớp yaw cổ tay (node <bên>_wrist_yaw, servo STS3215 ID 21/22) giữa giá J5 và vỏ J56, gốc lệch (28,1; ∓3,8; -1,3) mm so với trục gập. Cổ tay theo URDF upstream: trục gập thấp hơn đường trục khuỷu 3,2 mm, gốc khớp cuộn lệch (89,1; ∓3,8; -23,3) mm so với trục gập. Tay phải = gương y tay trái (như MJCF).",
                    "Gán chi tiết vào khớp theo sim/m1.xml (cùng nguồn với sim). Đế di chuyển (base_x, base_y, base_yaw) và bánh quay không có node: web dời node m1. Camera ngực đi theo node lift."])
json.dump(doc, open(os.path.join(out, "joints.json"), "w"), ensure_ascii=False, indent=1)
# ---- cum.json ----
reg = {r["ma"]: r for r in csv.DictReader(open(os.path.join(repo, "product", "parts-register.csv"), encoding="utf-8"))}
NHAP_08 = {"M1-BS-020", "M1-BS-021", "M1-AR-021", "M1-LF-024"}
DESC = dict(base="Đế kiểu LeKiwi: tấm thép đối trọng 2 lớp (~8 kg), nan nhôm 20x40, giá servo bánh, sàn in, 3 bánh omni 4 in + 3 servo STS3215, khay trước",
            power="Khoang điện tử trên sàn đế (khối giữ chỗ): pin 12 V; Pi 5, bus adapter, E-stop, contactor",
            lift="Trục nâng AlohaMini2: trụ Post1-4 có thanh răng, khung T giữa/trái/phải, thanh ngang 2040, bánh răng + servo STS3095 (giữ chỗ), chốt hãm, công tắc hành trình",
            camera="Camera đầu H65V1 trên cột trạm và camera cổ tay trên giá OB_F_Camera của kẹp",
            shoulder_L="Vai trái AM-ARM200: đế J1, khung xoay J2, cánh tay J3", shoulder_R="Vai phải", arm_L="Tay trái: khuỷu J4, giá J5, vỏ cổ tay J56, servo gập, yaw và cuộn STS3215", arm_R="Tay phải", gripper_L="Kẹp gốc AM-ARM200 trái: thân J7, hàm J7, giá camera, servo kẹp STS3215", gripper_R="Kẹp phải")
NHAP_08 = set()
cb = {}
cums = []
for c in ("base", "power", "lift", "shoulder_L", "shoulder_R", "arm_L", "arm_R", "gripper_L", "gripper_R", "camera"):
    items = {}
    for (cm, code), names in sorted(parts.items()):
        if cm != c: continue
        r = reg[code]; items[code] = dict(ma=code, ten=r["ten"], trang_thai_so=r["trang_thai"], license=r["license"], nguon_goc=r["nguon_goc"], chi_tiet_cad=sorted(names), nhap=("nháp" in r["trang_thai"]) or code in NHAP_08)
    dd = dict(ten=c, mo_ta=DESC[c], ma=sorted(items), chi_tiet=list(items.values()), trang_thai="nháp" if (not items or any(i["nhap"] for i in items.values())) else "đủ", node=[nd.name for nd in gl.nodes if nd.extras and nd.extras.get("cum") == c and nd.mesh is not None])
    assert items, "cụm không có lưới: " + c     # v0.4.4: mọi cụm có lưới (DK-W1)
    cums.append(dd)
# tâm/hộp bao cụm từ GLB (world, home)
def acc(i):
    a = gl.accessors[i]; bv = gl.bufferViews[a.bufferView]; return np.frombuffer(bytes(blob)[bv.byteOffset + (a.byteOffset or 0):][:a.count * 12], np.float32).reshape(-1, 3)
for nd in gl.nodes:
    if nd.mesh is not None:
        V = acc(gl.meshes[nd.mesh].primitives[0].attributes.POSITION) + W0[nd.extras["joint_node"]]; b = cb.setdefault(nd.extras["cum"], [1e9] * 3 + [-1e9] * 3)
        for k in range(3): b[k] = min(b[k], V[:, k].min()); b[3 + k] = max(b[3 + k], V[:, k].max())
for dd in cums:
    if dd["ten"] in cb: b = cb[dd["ten"]]; dd["bbox_world_home_m"] = [round(float(x), 4) for x in b]; dd["tam_world_home_m"] = [round(float((b[k] + b[k + 3]) / 2), 4) for k in range(3)]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import cum_ma_chinh as MC   # ma_chinh: một nguồn với 11_manifest.py
_e = MC.check(cums, set(reg)); assert not _e, _e; cums = [MC.with_ma_chinh(x) for x in cums]
json.dump(dict(schema="faibot-m1-cum/2", ghi_chu=["Mã từ product/parts-register.csv; tên chi tiết CAD theo cad/faibotic/assembly_pl.py (P-L, S1G), bắt đầu bằng mã M1.", "nhap=true: sổ ghi nháp/CAD nháp.",
               "Mỗi cụm = tập node lá <cum>.<node_khớp>.<vật liệu> (extras.cum), nằm dưới node khớp nên đi theo khớp. Tách lớp: dịch tâm cụm (tam_world_home_m, hệ CAD) rồi đổi sang Y-up qua node m1.",
               "camera gồm camera ngực (node lift, đi theo xe nâng) và camera cổ tay (node left/right_wrist_roll). Khoang điện tử (power) là khối giữ chỗ, chưa có linh kiện chi tiết."], cum=cums), open(os.path.join(out, "cum.json"), "w"), ensure_ascii=False, indent=1)
json.dump(dict(stat=stat, ratio=stat["tri_out"] / stat["tri_in"], roller_tri=ROLLER, min_tri=MIN_TRI, crease_deg=CREASE_DEG, servo_tri=SERVO_TRI, power_ratio=POWER_RATIO, ratio_param=RATIO, nhan_thap=unl), open(os.path.join(out, "build_stat.json"), "w"), ensure_ascii=False)
print("cum", [(x["ten"], len(x["chi_tiet"])) for x in cums])
