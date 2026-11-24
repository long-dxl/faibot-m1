"""Kiểm web-assets: schema JSON, hash khớp manifest và sim/assets, gltf-validator 0 lỗi, brotli <= 1.200.000 byte, joints trùng spec.py và m1.xml, FK <= 1 mm so SDK (khung gốc tay, z tuyệt đối, FK độc lập; truyền side), nhãn cụm, power/camera có lưới.
Dùng: python 12_validate.py REPO WEB_DIR [NODE_BIN] [VALIDATOR_DIR]   Thoát 1 nếu có mục không đạt. Ghi WEB_DIR/kiem.json."""
import sys, os, json, math, hashlib, random, subprocess
import numpy as np, pygltflib as G
import xml.etree.ElementTree as ET
repo, web = sys.argv[1:3]; NODE = sys.argv[3] if len(sys.argv) > 3 else "node"; VAL = sys.argv[4] if len(sys.argv) > 4 else None
sys.path.insert(0, os.path.join(repo, "sdk")); from faibotic import spec, kinematics as K
R = {}; ok_all = True
def rec(k, ok, **kw):
    global ok_all; ok_all &= bool(ok); R[k] = dict(dat=bool(ok), **kw); print(("DAT  " if ok else "LOI  ") + k, kw if not ok or len(str(kw)) < 160 else "")
H = lambda p: hashlib.sha256(open(p, "rb").read()).hexdigest()
J = json.load(open(os.path.join(web, "joints.json"))); M = json.load(open(os.path.join(web, "manifest.json"))); CU = json.load(open(os.path.join(web, "cum.json")))
# 1 schema
import jsonschema
for f, s in (("manifest.json", "manifest"), ("joints.json", "joints"), ("cum.json", "cum")):
    try: jsonschema.validate(json.load(open(os.path.join(web, f))), json.load(open(os.path.join(web, "schema", s + ".schema.json")))); rec("schema_" + s, True)
    except Exception as e: rec("schema_" + s, False, loi=str(e)[:200])
if os.path.exists(os.path.join(web, "lab-scene.json")):
    try: jsonschema.validate(json.load(open(os.path.join(web, "lab-scene.json"))), json.load(open(os.path.join(web, "schema", "lab-scene.schema.json")))); rec("schema_lab_scene", True)
    except Exception as e: rec("schema_lab_scene", False, loi=str(e)[:200])
else: rec("lab_scene_cho_luong_C", True, ghi_chu="chưa có lab-scene.json; chỗ đã chừa trong manifest và schema")
# 2 hash
bad = [f["file"] for f in M["files"] if H(os.path.join(web, f["file"])) != f["sha256"] or os.path.getsize(os.path.join(web, f["file"])) != f["bytes"]]
rec("hash_manifest", not bad, so_file=len(M["files"]), sai=bad)
mj = json.load(open(os.path.join(repo, "sim", "assets", "meshes.json"))); bad2 = [e["file"] for e in mj["meshes"] if H(os.path.join(repo, "sim", "assets", e["file"])) != e["sha256"]]
rec("hash_sim_assets", not bad2 and M["nguon"]["sim_assets_meshes_json_sha256"] == H(os.path.join(repo, "sim", "assets", "meshes.json")), so_stl=len(mj["meshes"]), sai=bad2)
# 3 glb
glb = os.path.abspath(os.path.join(web, "m1.glb")); sz = os.path.getsize(glb)
# ngân sách theo hợp đồng web (check:m1-assets): "robot <= 1,2 MB nén" = brotli q11 <= 1.200.000 byte; kích thước thô ghi để theo dõi
JS = "const z=require('zlib'),b=require('fs').readFileSync(process.argv[1]);console.log(z.brotliCompressSync(b,{params:{[z.constants.BROTLI_PARAM_QUALITY]:11,[z.constants.BROTLI_PARAM_SIZE_HINT]:b.length}}).length)"
try: br = int(subprocess.run([NODE, "-e", JS, glb], capture_output=True, text=True, check=True).stdout.strip())
except Exception as e: br = None
rec("brotli_le_1_2MB", br is not None and br <= 1_200_000, brotli_bytes=br, raw_bytes=sz)
if VAL:
    r = subprocess.run([NODE, "run.mjs", glb], cwd=VAL, capture_output=True, text=True); v = json.loads(r.stdout)["issues"]; rec("gltf_validator", v["numErrors"] == 0 and v["numWarnings"] == 0, loi=v["numErrors"], canh_bao=v["numWarnings"], info=v["numInfos"])
else:
    rec("gltf_validator", False, ly_do="không có gltf-validator: không được tính là đạt")
gl = G.GLTF2().load_binary(glb); blob = gl.binary_blob(); nd = {n.name: n for n in gl.nodes}
def pos(n):
    p = nd[n]; return None
miss = [c["node"][0] for c in CU["cum"] if not c["node"] or c["node"][0] not in nd]; rec("node_cum_ton_tai", not miss, thieu=miss, so_node=len(gl.nodes))
cnt = {c: sum(1 for n in gl.nodes if n.mesh is not None and (n.extras or {}).get("cum") == c) for c in ("power", "camera")}
rec("power_camera_co_luoi", all(cnt.values()) and not any(n in nd for n in ("power", "camera")), node_la=cnt)
# 4 joints vs spec + m1.xml
xr = {}
for j in ET.parse(os.path.join(repo, "sim", "m1.xml")).getroot().iter("joint"):
    if j.get("range"): xr[j.get("name")] = ([float(x) for x in j.get("range").split()], [float(x) for x in (j.get("axis") or "0 0 1").split()])
bad = []
for k, jj in J["joints"].items():
    lo, hi = spec.limits(k); a, b = jj["limit_soft"]
    if k.endswith("gripper"): continue
    if abs(a - lo) > 1e-12 or abs(b - hi) > 1e-12: bad.append((k, "spec"))
    sn = jj["sim_joint"]; r_, ax = xr[sn]
    if max(abs(r_[0] - lo), abs(r_[1] - hi)) > 2e-5 or ax != jj["axis"]: bad.append((k, "m1.xml"))
c = J["couplings"]; same = (c["WRIST_COUPLE_MAX_DEG"] == [list(x) for x in spec.WRIST_COUPLE_MAX_DEG] and c["ELBOW_COUPLE_MAX_DEG"] == list(spec.ELBOW_COUPLE_MAX_DEG) and c["ELBOW_COUPLE_SHOULDER_DEG"] == list(spec.ELBOW_COUPLE_SHOULDER_DEG)
        and c["WRIST_COUPLE_SHOULDER_DEG"] == list(spec.WRIST_COUPLE_SHOULDER_DEG) and c["WRIST_COUPLE_ELBOW_DEG"] == list(spec.WRIST_COUPLE_ELBOW_DEG) and c["pan_soft_max_deg"] == spec.PAN_SOFT_MAX_DEG)
rec("joints_trung_spec_va_xml", not bad and same, lech=bad, bang_ghep_trung=same)
# 5 node translation GLB == joints.json
N = J["nodes"]; d_ = [n for n in N if n in nd and n not in ("m1",) and N[n]["parent"] == "m1" and False]
tr = max((np.abs(np.array(nd[n].translation or [0, 0, 0]) - np.array(N[n]["origin_rel_m"])).max() for n in N if n in nd and n != "m1"), default=0); rec("node_translation_glb_vs_joints", tr < 1e-6, max_m=float(tr))
# 6 FK
def rot(ax, a):
    ax = np.array(ax, float); ax /= np.linalg.norm(ax); x, y, z = ax; c, s = math.cos(a), math.sin(a); C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])
JN = {v["node"]: (k, v) for k, v in J["joints"].items()}
def fk(q):
    W = {}
    def go(n):
        if n in W: return W[n]
        par = N[n]["parent"]; P = go(par) if par else np.eye(4); T = np.eye(4)
        if par: T[:3, 3] = np.array(N[n]["origin_rel_m"])
        if n in JN:
            k, j = JN[n]; v = q.get(k, 0.0)
            if j["type"] == "revolute": T[:3, :3] = rot(j["axis"], v)
            else: T[:3, 3] += np.array(j["axis"], float) * (v if k == "lift" else v / 100.0 * j["travel_m"])
        W[n] = P @ T; return W[n]
    for n in N: go(n)
    return W
def tcp(W, s): R_ = W[f"{s}_wrist_roll"]; return R_[:3, :3] @ np.array(J["tcp"]["by_side"][s]) + R_[:3, 3]
def Ry(a): c, s_ = math.cos(a), math.sin(a); return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
def Rx(a): c, s_ = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
def Rz(a): c, s_ = math.cos(a), math.sin(a); return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
def fk_ind(q, k, lift):
    """FK viết tay, không dùng spec/kinematics/joints.json: hằng số URDF upstream AlohaMini2 + AM-ARM200 (mm) chép tay theo S1C/S1G; k = +1 trái, -1 phải (gương y).
    Gốc tay: left_base_joint (-y 59,9; z 96,196 + 477,5 - 160 ở lift SDK 0), y = ±237,5 (nửa vai trạm). Vai: 39,67 + 108,56 trên, 57,99 trước trục pan. L1 134,91."""
    R_ = Rz(q[0]); p = np.array([0.0599, k * 0.2375, 0.573696 - 0.160 + lift]) + R_ @ [0.05799, 0, 0.14823]
    R_ = R_ @ Ry(q[1]); p = p + R_ @ [0.13491, 0, 0]
    R_ = R_ @ Ry(q[2]); p = p + R_ @ [0.19841, 0, -0.00316]
    R_ = R_ @ Ry(q[3]); p = p + R_ @ [0.02813, k * -0.00381, -0.00130]      # 6+1: khớp yaw (URDF left_wrist_yaw_joint, trục +z SDK)
    R_ = R_ @ Rz(q[4]); p = p + R_ @ [0.06099, 0, -0.02202]                  # yaw -> gốc cuộn (tổng gập -> cuộn = (89,12; ∓3,81; -23,32))
    R_ = R_ @ Rx(q[5]); return p + R_ @ [0.09382, k * 0.01013, -0.00016]     # tâm kẹp AM-ARM200 (93,82; ±10,13; -0,16) mm từ gốc cuộn (AM_gripper_PL.json)
random.seed(3); e_rel = []; e_abs = []; e_ind = []; lim = spec.ARM_LIMITS
n_ = 0
while n_ < 400:
    q = [random.uniform(*lim[j]) for j in spec.ARM_JOINTS]
    if not K.within_limits(q): continue
    n_ += 1; lift = random.uniform(*spec.LIFT_RANGE)
    for s, k in (("left", 1), ("right", -1)):
        W = fk({"lift": lift, **{f"{s}_{j}": v for j, v in zip(spec.ARM_JOINTS, q)}}); t = tcp(W, s); sdk = K.arm_fk(q, s)[0]     #d m4: truyền side
        e_rel.append(np.linalg.norm((t - W[f"{s}_pan"][:3, 3]) - sdk)); e_abs.append(np.linalg.norm(t - (K.arm_mount(s, lift) + sdk)))
        e_ind.append(np.linalg.norm(t - fk_ind(q, k, lift)))
rec("FK_vs_SDK_khung_goc_tay_1mm", max(e_rel) * 1000 <= 1, max_mm=round(max(e_rel) * 1000, 4)); rec("FK_vs_SDK_z_tuyet_doi_1mm", max(e_abs) * 1000 <= 1, max_mm=round(max(e_abs) * 1000, 4), mau=len(e_abs))
rec("FK_doc_lap_hang_so_CAD_1mm", max(e_ind) * 1000 <= 1, max_mm=round(max(e_ind) * 1000, 4), ghi_chu="ma trận quay viết tay, không dùng spec/kinematics/joints.json: hằng số URDF chép tay (gốc tay (59,9; ±237,5; 413,7 + lift), vai (57,99; 148,23), L1 134,91, L2 198,41, gập -3,16, yaw (28,13; ∓3,81; -1,30), cuộn từ yaw (60,99; 0; -22,02), tâm kẹp AM-ARM200 (93,82; ±10,13; -0,16) từ gốc cuộn) mm")
# 7 hình học GLB vs STL sim (bbox từng cụm so tổng hộp bao STL)
def accv(i):
    a = gl.accessors[i]; bv = gl.bufferViews[a.bufferView]; return np.frombuffer(blob, np.float32, a.count * 3, bv.byteOffset + (a.byteOffset or 0)).reshape(-1, 3).astype(float)
import trimesh
W0 = fk({}); allg = np.concatenate([accv(gl.meshes[n.mesh].primitives[0].attributes.POSITION) + W0[n.extras["joint_node"]][:3, 3] for n in gl.nodes if n.mesh is not None])
fr = {k: np.array(v) for k, v in mj["source"]["frames"].items()}; alls = np.concatenate([np.asarray(trimesh.load(os.path.join(repo, "sim", "assets", e["file"]), process=False).vertices) + fr[e["body"]] for e in mj["meshes"]])
dbb = np.abs(np.concatenate([allg.min(0), allg.max(0)]) - np.concatenate([alls.min(0), alls.max(0)])).max() * 1000; rec("hop_bao_GLB_vs_STL_sim_1mm", dbb <= 1.0, max_mm=round(float(dbb), 3), chieu_cao_m=round(float(allg[:, 2].max()), 4))
json.dump(R, open(os.path.join(web, "kiem.json"), "w"), ensure_ascii=False, indent=1); print("TONG:", "DAT" if ok_all else "KHONG DAT"); sys.exit(0 if ok_all else 1)
