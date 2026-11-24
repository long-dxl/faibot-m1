"""Cổng hình: chụp m1.glb ở 3 góc (hero, canh, sau) bằng rasterizer phần mềm (numpy + PIL, không cần GL), tư thế ready lift 0,20 m như media/sim-snapshots.
Camera: phương vị, độ cao, tiêu cự, lấp khung theo media/renders-web/src/views.json. Màu = PBR base color, ánh sáng Lambert đơn giản: chỉ để so hình dáng, màu, tỉ lệ.
Dùng: python 13_snapshots.py WEB_DIR VIEWS_JSON [SIM_SNAP_DIR]   (ghi WEB_DIR/snapshots/m1-<view>.png và so-sanh-<view>.png nếu có ảnh sim)"""
import sys, os, json, math
import numpy as np, pygltflib as G
from PIL import Image, ImageDraw
web, views = sys.argv[1:3]; sim = sys.argv[3] if len(sys.argv) > 3 else None
J = json.load(open(os.path.join(web, "joints.json"))); V = json.load(open(views)); gl = G.GLTF2().load_binary(os.path.join(web, "m1.glb")); blob = gl.binary_blob(); N = J["nodes"]
JN = {v["node"]: (k, v) for k, v in J["joints"].items()}; q = J["poses"]["ready"]["q"]
def rot(ax, a):
    ax = np.array(ax, float); x, y, z = ax / np.linalg.norm(ax); c, s = math.cos(a), math.sin(a); C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])
W = {}
def go(n):
    if n in W: return W[n]
    par = N[n]["parent"]; P = go(par) if par else np.eye(4); T = np.eye(4)
    if par: T[:3, 3] = N[n]["origin_rel_m"]
    if n in JN:
        k, j = JN[n]; v = q[k]
        if j["type"] == "revolute": T[:3, :3] = rot(j["axis"], v)
        else: T[:3, 3] += np.array(j["axis"], float) * (v if k == "lift" else v / 100 * j["travel_m"])
    W[n] = P @ T; return W[n]
for n in N: go(n)
def acc(i, comp, dt):
    a = gl.accessors[i]; bv = gl.bufferViews[a.bufferView]; return np.frombuffer(blob, dt, a.count * comp, bv.byteOffset + (a.byteOffset or 0)).reshape(-1, comp) if comp > 1 else np.frombuffer(blob, dt, a.count, bv.byteOffset + (a.byteOffset or 0))
TRI = []; COL = []
for nd in gl.nodes:
    if nd.mesh is None: continue
    pr = gl.meshes[nd.mesh].primitives[0]; M = W[nd.extras["joint_node"]]; v = acc(pr.attributes.POSITION, 3, np.float32).astype(float) @ M[:3, :3].T + M[:3, 3]
    f = acc(pr.indices, 1, np.uint16 if gl.accessors[pr.indices].componentType == 5123 else np.uint32).reshape(-1, 3).astype(int)
    col = np.array(gl.materials[pr.material].pbrMetallicRoughness.baseColorFactor[:3]) ** (1 / 2.2); met = gl.materials[pr.material].pbrMetallicRoughness.metallicFactor
    TRI.append(v[f]); COL.append(np.tile(np.r_[col, met], (len(f), 1)))
TRI = np.concatenate(TRI); COL = np.concatenate(COL); allv = TRI.reshape(-1, 3); lo, hi = allv.min(0), allv.max(0); tgt = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2])
fn = np.cross(TRI[:, 1] - TRI[:, 0], TRI[:, 2] - TRI[:, 0]); fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)
def render(name, cfg, scale=0.5):
    W_, H_ = [int(x * scale) for x in cfg["res"]]; SS = 2; az = math.radians(cfg["az"]); h = cfg.get("h", 1.0); fl = cfg["lens"] / 36.0
    def cam(d):
        pos = tgt + np.array([d * math.cos(az), d * math.sin(az), 0]); pos[2] = h; fwd = tgt - pos; fwd /= np.linalg.norm(fwd); right = np.cross(fwd, [0, 0, 1]); right /= np.linalg.norm(right); up = np.cross(right, fwd); return pos, fwd, right, up
    def proj(P, d):
        pos, fwd, right, up = cam(d); r = P - pos; z = r @ fwd; s = fl * W_ * SS
        return np.c_[r @ right / z * s + W_ * SS / 2, -(r @ up) / z * s + H_ * SS / 2], z
    d = 3.0
    for _ in range(25):
        xy, _z = proj(allv, d); hh = (xy[:, 1].max() - xy[:, 1].min()) / (H_ * SS); d *= hh / cfg["fill"]
    xy, z = proj(allv, d); c0 = (xy.min(0) + xy.max(0)) / 2; xy = xy - c0 + np.array([W_ * SS / 2, H_ * SS / 2]); P2 = xy.reshape(-1, 3, 2); zc = z.reshape(-1, 3).mean(1)
    pos, fwd, *_ = cam(d); light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light); toc = -fwd
    sh = 0.35 + 0.45 * np.abs(fn @ toc) + 0.25 * np.clip(fn @ light, 0, 1); img = Image.new("RGB", (W_ * SS, H_ * SS), (0xd6, 0xd3, 0xcc)); dr = ImageDraw.Draw(img)
    for i in np.argsort(-zc):
        c = COL[i]; rgb = np.clip(c[:3] * sh[i] * (1.0 - 0.15 * c[3]) + 0.08 * c[3], 0, 1); dr.polygon([tuple(p) for p in P2[i]], fill=tuple((rgb * 255).astype(int)))
    img = img.resize((W_, H_), Image.LANCZOS); os.makedirs(os.path.join(web, "snapshots"), exist_ok=True); p = os.path.join(web, "snapshots", f"m1-{name}.png"); img.save(p); return img
for name, key, sf in (("hero", "hero", "sim-hero.png"), ("canh", "canh", "sim-canh.png"), ("sau", "sau", "sim-sau.png")):
    im = render(name, V[key])
    if sim and os.path.exists(os.path.join(sim, sf)):
        s = Image.open(os.path.join(sim, sf)).convert("RGB"); s = s.resize((int(s.width * im.height / s.height), im.height)); both = Image.new("RGB", (im.width + s.width, im.height)); both.paste(im, (0, 0)); both.paste(s, (im.width, 0)); both.save(os.path.join(web, "snapshots", f"so-sanh-{name}.png"))
print("snapshots xong")
