"""Xuất WebP, bảng xem nhanh và MANIFEST.md sau khi render xong (đọc /tmp/.../times.log do pipeline ghi)."""
import glob, os, subprocess, datetime, json
from PIL import Image
R = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); S = os.environ.get("WORK", "/tmp/m1-render-work")
rev = open(os.path.join(R, "src", "REV")).read().strip() if os.path.exists(os.path.join(R, "src", "REV")) else "(không có git)"
ang = {"hero": ("3/4 trước, camera 1,2 m nhìn hơi từ trên xuống", "ready", 200), "can-vai-kep": ("cận vai-khuỷu-cổ tay-kẹp tay trái", "ready", 200), "chinh-dien": ("chính diện", "ready", 200), "canh": ("cạnh bên (trái)", "ready", 200),
       "sau": ("sau (3/4 sau)", "ready", 200), "cot-nang-thap": ("3/4 trước, cột nâng thấp", "ready", 0), "cot-nang-cao": ("3/4 trước, cột nâng cao", "ready", 500), "thuoc-kich-thuoc": ("trực giao cạnh, có thước", "straight", 250)}
times = {}
for l in open(S + "/times.log"):
    p = l.split()
    if len(p) == 3: times[os.path.basename(p[1])] = float(p[2])
rows = []
for k, (a, pose, lift) in ang.items():
    f = f"{R}/faibot-m1-{k}.png"
    if not os.path.exists(f): continue
    im = Image.open(f); im.save(f[:-4] + ".webp", quality=90, method=6)
    t = times.get("faibot-m1-" + k + ".png", times.get("thuoc_raw.png") if k.startswith("thuoc") else 0)
    rows.append(f"| faibot-m1-{k}.png / .webp | {a} | {im.width}x{im.height} | `{pose}` (vai/khuỷu/cổ tay độ: {dict(ready='-30/70/10', straight='0/0/0')[pose]}), lift {lift} mm | {t/60:.1f} |")
fs = sorted(glob.glob(R + "/faibot-m1-*.png")); ims = [Image.open(f).convert("RGB") for f in fs]; h = 500
ims = [i.resize((int(i.width * h / i.height), h)) for i in ims]; W = sum(i.width for i in ims[:4]); sheet = Image.new("RGB", (max(W, sum(i.width for i in ims[4:])), 2 * h), "white"); x = 0
for i in ims[:4]: sheet.paste(i, (x, 0)); x += i.width
x = 0
for i in ims[4:]: sheet.paste(i, (x, h)); x += i.width
sheet.save(S + "/sheet.png")
dims = json.load(open(S + "/dims.json"))
open(R + "/MANIFEST.md", "w").write(f"""# Bộ ảnh Faibot M1 (media/renders-web)

Ngày render: {datetime.date.today().isoformat()}. Commit CAD nguồn lúc render (`git rev-parse HEAD`): `{rev}` (cây làm việc chưa commit: cad/faibotic/assembly_full.py, base_shell.py và media/renders-web/src là bản mới nhất, chưa nằm trong commit này).
Nguồn: `cad/faibotic/assembly_full.py --pose P --lift L --mesh DIR` (lắp ráp CAD, kiểm giao cắt build123d, xuất STL) rồi `media/renders-web/src/render_scene.py -- DIR <view> <ra.png>` (Blender 4.0.2 Cycles CPU, view trong views.json); ảnh thước thêm `src/overlay.py`.

| Ảnh | Góc | Pixel | Tham số tư thế | Render (phút) |
|---|---|---|---|---|
""" + "\n".join(rows) + f"""

Số trên ảnh thước (đọc từ mesh CAD): cao tổng = chiều cao cột {dims['cao_tong_mm']} mm; đường kính đế {dims['duong_kinh_de_mm']} mm; tầm với {dims['tam_voi_tu_tam_de_mm']} mm từ tâm đế tới mũi kẹp, tay duỗi thẳng (SOR-004 524 mm tính từ trục vai).

## Chi tiết CAD và chi tiết nháp
- CAD chi tiết đã có: gá vai v0.4.3, link, chốt, lò xo, chặn trên/dưới và đệm, gá khuỷu ghép đôi, đệm khuỷu, tay AM-ARM200 (giá J5, chốt, vỏ J56, kẹp J7, ngón; Apache-2.0).
- Cột nâng (cột 4040, ray, xe nâng, vitme, hộp số): dựng từ `cad/faibotic/lift.py` bản QC2 vòng 2 (CAD đạt có điều kiện, chưa có CAD chi tiết từng mối ghép).
- Nháp dựng mới, chưa phải bản vẽ chế tạo: thân đế M1-BS-001, nắp M1-BS-002, cửa D0 M1-BS-003, bánh omni M1-BS-010; cẳng tay M1-AR-021; đế xoay vai M1-LF-024. Rãnh cấp lắp ráp trên tay đỡ vai M1-LF-023 để né chặn dưới M1-SH-017 (CAD gốc giao nhau).
- Vị trí kẹp lắp theo tiếp xúc đo bằng build123d (chạm J56 Bottom, 0 giao); ngón nằm trong rãnh kẹp theo khoảng cách đỉnh ~0,2 mm (STEP upstream không có lỗ/mặt tham chiếu: gripper/jaw/camera chỉ có mặt B-spline). Giá camera không xác định được vị trí lắp nên bỏ khỏi cảnh. Đầu camera trên cột chưa dựng.
""")
