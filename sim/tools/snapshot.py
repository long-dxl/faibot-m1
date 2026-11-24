"""Chụp ảnh sim MuJoCo offscreen ở các góc của bộ render (media/renders-web/src/views.json): hero, chinh-dien, canh, sau.
Tư thế như bộ render: ready (vai -30, khuỷu 70, cổ tay 10 độ, hai tay), lift 200 mm. Nền/sàn #d6d3cc như ảnh render.
Camera quy đổi từ views.json: phương vị az (độ, quanh trục z, 0 = nhìn từ phía trước +x), độ cao camera h (m), tiêu cự lens (mm, cảm biến 36 mm
theo cạnh dài), lấp khung fill; lặp khoảng cách và tâm như render_scene.py, chủ thể = hộp bao các mesh hiển thị.
Chạy (từ gốc repo): MUJOCO_GL=egl .venv/bin/python sim/tools/snapshot.py [--out media/sim-snapshots] [--scale 0.5] [--views hero,chinh-dien,canh,sau]
  [--ref <menagerie>/robotstudio_so101/so101.xml]   ghép thêm so-sanh-so101-{hero,canh,sau}.png (M1 trái, SO-101 phải; SO-101 không tỉ lệ thật,
  mỗi bên lấp khung riêng)"""
import argparse, json, math, os, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sdk"))
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
from faibotic import model as M

POSE_DEG = {"shoulder": -30.0, "elbow": 70.0, "wrist_flex": 10.0}
LIFT = 0.20
BG = "0.839 0.827 0.8"   # #d6d3cc


STUDIO_VIS = ('<global offwidth="2600" offheight="2600"/><quality shadowsize="16384" offsamples="8"/>'
              '<headlight ambient="0.32 0.32 0.32" diffuse="0.35 0.35 0.35" specular="0.1 0.1 0.1"/>')
STUDIO_LIGHTS = ('<light pos="3.4 2.8 3.6" dir="-3.4 -2.8 -2.9" directional="true" diffuse="0.5 0.49 0.47" castshadow="true"/>'
                 '<light pos="3.2 -3.4 2.2" dir="-3.2 3.4 -1.5" directional="true" diffuse="0.25 0.25 0.26" castshadow="false"/>'
                 '<light pos="-3.4 0.8 2.6" dir="3.4 -0.8 -1.9" directional="true" diffuse="0.2 0.2 0.21" castshadow="false"/>')


def studio(xml: str, floor: bool = False) -> str:
    """Đèn và nền studio dùng chung cho M1 và mô hình tham chiếu: nền phẳng #d6d3cc (skybox một màu), 3 đèn hướng như render_scene.py.
    Mặc định không có sàn để khỏi lộ đường chân trời (ảnh render là phông cong liền); floor=True thêm sàn nhận bóng."""
    xml = xml.replace("<visual>", "<visual>" + STUDIO_VIS, 1)
    if 'type="skybox"' in xml:
        xml = xml.replace('rgb1="0.95 0.96 0.98" rgb2="0.75 0.8 0.88"', f'rgb1="{BG}" rgb2="{BG}"')
    else:
        xml = xml.replace("<asset>", f'<asset><texture name="studio_sky" type="skybox" builtin="flat" rgb1="{BG}" rgb2="{BG}" width="32" height="32"/>', 1)
    extra = STUDIO_LIGHTS
    if floor:
        xml = xml.replace("</asset>", f'<material name="studio" rgba="{BG} 1" emission="0.3" specular="0" shininess="0"/></asset>', 1)
        extra += '<geom name="studio_floor" type="plane" size="8 8 0.1" material="studio" contype="0" conaffinity="0"/>'
    return xml.replace("<worldbody>", "<worldbody>" + extra, 1)


def build(floor: bool = False):
    xml = M.robot_mjcf(scene=False)
    xml = xml.replace('<visual><global offwidth="1280" offheight="720"/><quality shadowsize="8192" offsamples="8"/></visual>', "<visual></visual>")
    # bỏ đèn mặc định của mô hình (quá gắt cho ảnh studio)
    xml = xml.replace('<light pos="1 -1 3" dir="-0.2 0.3 -1" directional="true" diffuse="0.8 0.8 0.8"/>', "")
    xml = xml.replace('<light pos="-1 1 3" dir="0.3 -0.2 -1" directional="true" diffuse="0.3 0.3 0.3" castshadow="false"/>', "")
    m = mujoco.MjModel.from_xml_string(studio(xml, floor))
    d = mujoco.MjData(m)
    d.qpos[m.joint("lift").qposadr[0]] = LIFT
    for s in ("left", "right"):
        for j, v in POSE_DEG.items():
            d.qpos[m.joint(f"{s}_{j}").qposadr[0]] = math.radians(v)
    mujoco.mj_forward(m, d)
    return m, d


def build_ref(path: str):
    """Mô hình tham chiếu (vd SO-101 của MuJoCo Menagerie, tải ngoài repo): cùng đèn/nền; tư thế keyframe đầu nếu có, không thì 0.
    File tạm _studio.xml ghi cạnh file gốc (thư mục tải về) để đường dẫn mesh tương đối vẫn đúng."""
    src = Path(path); tmp = src.with_name("_studio.xml")
    tmp.write_text(studio(src.read_text()))
    m = mujoco.MjModel.from_xml_path(str(tmp)); d = mujoco.MjData(m)
    if m.nkey:
        mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d


def corners(m, d):
    """Góc hộp bao (theo hệ mesh) của từng geom mesh hiển thị, toạ độ thế giới."""
    out = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mi = m.geom_dataid[g]; a, n = m.mesh_vertadr[mi], m.mesh_vertnum[mi]
        v = m.mesh_vert[a:a + n]
        lo, hi = v.min(0), v.max(0)
        c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        out.append(d.geom_xpos[g] + c @ d.geom_xmat[g].reshape(3, 3).T)
    return np.concatenate(out)


def frame(view, P, aspect):
    """Như render_scene.py: lặp khoảng cách theo fill, dời đích để chủ thể ra giữa khung. Trả (vị trí camera, đích, fovy độ)."""
    fovy = 2 * math.degrees(math.atan(18.0 / view["lens"])) if aspect <= 1 else 2 * math.degrees(math.atan(18.0 / view["lens"] / aspect))
    ty = math.tan(math.radians(fovy) / 2); tx = ty * aspect
    ct = P.mean(0); dirv = np.array([math.cos(math.radians(view["az"])), math.sin(math.radians(view["az"])), 0.0]); dist = 4.0
    for _ in range(10):
        cam = ct + dirv * dist; cam[2] = view["h"]
        fwd = (ct - cam) / np.linalg.norm(ct - cam); right = np.cross(fwd, [0, 0, 1.0]); right /= np.linalg.norm(right); up = np.cross(right, fwd)
        rel = P - cam; zc = rel @ fwd
        u = 0.5 + 0.5 * (rel @ right) / zc / tx; w = 0.5 + 0.5 * (rel @ up) / zc / ty
        fw, fh = u.max() - u.min(), w.max() - w.min()
        dist *= max(fh / view["fill"], fw / (view["fill"] + 0.12))
        mx, my = (u.max() + u.min()) / 2 - 0.5, (w.max() + w.min()) / 2 - 0.5
        L = np.linalg.norm(ct - cam)
        ct = ct + right * mx * 2 * L * tx + up * my * 2 * L * ty
    cam = ct + dirv * dist; cam[2] = view["h"]
    return cam, ct, fovy


def write_png(path, img):
    """PNG RGB 8 bit bằng zlib (không cần Pillow)."""
    import struct, zlib
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))
    chunk = lambda t, b: struct.pack(">I", len(b)) + t + b + struct.pack(">I", zlib.crc32(t + b) & 0xFFFFFFFF)
    Path(path).write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def render(m, d, v, P, W, H):
    cam_pos, tgt, fovy = frame(v, P, W / H)
    m.vis.global_.fovy = fovy
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.lookat[:] = tgt
    rel = tgt - cam_pos; cam.distance = float(np.linalg.norm(rel))
    cam.azimuth = math.degrees(math.atan2(rel[1], rel[0])); cam.elevation = math.degrees(math.asin(rel[2] / cam.distance))
    opt = mujoco.MjvOption()
    for k in range(6):
        opt.geomgroup[k] = k in (0, 1, 2)
    opt.sitegroup[:] = 0                    # ẩn site (tcp, neo lò xo)
    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, camera=cam, scene_option=opt)
    img = r.render(); r.close()
    return np.ascontiguousarray(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "media" / "sim-snapshots")); ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--views", default="hero,chinh-dien,canh,sau"); ap.add_argument("--floor", action="store_true")
    ap.add_argument("--ref", help="MJCF tham chiếu (vd mujoco_menagerie/robotstudio_so101/so101.xml): ghép so-sanh-so101-<góc>.png")
    ap.add_argument("--ref-views", default="hero,canh,sau"); a = ap.parse_args()
    views = json.loads((ROOT / "media" / "renders-web" / "src" / "views.json").read_text())
    m, d = build(a.floor); P = corners(m, d)
    os.makedirs(a.out, exist_ok=True)
    imgs = {}
    for name in a.views.split(","):
        v = views[name]; W, H = int(v["res"][0] * a.scale), int(v["res"][1] * a.scale)
        imgs[name] = render(m, d, v, P, W, H)
        p = Path(a.out) / f"sim-{name}.png"; write_png(p, imgs[name]); print(p, W, H)
    if a.ref:
        mr, dr = build_ref(a.ref); Pr = corners(mr, dr)
        k = (Pr[:, 2].max() - Pr[:, 2].min()) / (P[:, 2].max() - P[:, 2].min())   # độ cao camera thu theo tỉ lệ chiều cao robot
        bg = np.array([214, 211, 204], np.uint8)
        for name in a.ref_views.split(","):
            v = dict(views[name]); v["h"] = v["h"] * k
            left = imgs[name] if name in imgs else render(m, d, views[name], P, *[int(x * a.scale) for x in views[name]["res"]])
            H, W = left.shape[:2]
            right = render(mr, dr, v, Pr, W, H)
            gap = np.empty((H, max(8, W // 40), 3), np.uint8); gap[:] = bg
            p = Path(a.out) / f"so-sanh-so101-{name}.png"; write_png(p, np.concatenate([left, gap, right], axis=1)); print(p)


if __name__ == "__main__":
    main()
