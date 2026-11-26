# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Hoàn thiện ảnh Cycles cho web-assets/lap-rap: ghép PNG RGBA (render_scene.py --alpha) lên nền #F5F5F7, xuất WebP 1600 và 800 px;
với --markers, vẽ chấm servo trung tính (không số) và ghi JSON toạ độ pixel.

  python web_hoan_thien.py ra.png TÊN OUT_DIR [--markers ra.png.markers.json --markers-name servo-id]
Ra: OUT_DIR/TÊN.webp (rộng 1600), OUT_DIR/TÊN-800.webp; với markers: OUT_DIR/servo-id.webp, servo-id-800.webp, servo-id.json
(JSON: {"1600": {"size": [w, h], "servo": {tên: {"id", "x", "y"}}}, "800": {...}}, gốc toạ độ góc trên trái ảnh)."""
import json, os, sys
from PIL import Image, ImageDraw

BG = (245, 245, 247)


def nen(png):
    im = Image.open(png).convert("RGBA")
    out = Image.new("RGBA", im.size, BG + (255,))
    out.alpha_composite(im)
    return out.convert("RGB")


def luu(im, d, ten):
    w, h = im.size
    assert w == 1600, w
    im.save(os.path.join(d, ten + ".webp"), quality=88, method=6)
    im.resize((800, round(800 * h / w)), Image.LANCZOS).save(os.path.join(d, ten + "-800.webp"), quality=88, method=6)


def cham(im, pos, r):
    """Chấm trung tính: nền trắng, viền xám đậm; vẽ ở độ phân giải gấp 4 rồi thu nhỏ để mịn."""
    k = 4
    lay = Image.new("RGBA", (im.width * k, im.height * k), (0, 0, 0, 0)); dr = ImageDraw.Draw(lay)
    for p in pos.values():
        x, y = p["x"] * k, p["y"] * k
        dr.ellipse([x - (r + 3) * k, y - (r + 3) * k, x + (r + 3) * k, y + (r + 3) * k], fill=(255, 255, 255, 235))
        dr.ellipse([x - r * k, y - r * k, x + r * k, y + r * k], fill=(70, 74, 82, 255))
    lay = lay.resize(im.size, Image.LANCZOS)
    o = im.convert("RGBA"); o.alpha_composite(lay)
    return o.convert("RGB")


def main():
    png, ten, d = sys.argv[1:4]
    os.makedirs(d, exist_ok=True)
    im = nen(png)
    luu(im, d, ten)
    if "--markers" in sys.argv:
        mj = json.load(open(sys.argv[sys.argv.index("--markers") + 1]))
        mt = sys.argv[sys.argv.index("--markers-name") + 1] if "--markers-name" in sys.argv else "servo-id"
        a = cham(im, mj["pos"], 8)
        luu(a, d, mt)
        w, h = mj["size"]
        js = {}
        for sc in (1600, 800):
            f = sc / w
            js[str(sc)] = {"size": [sc, round(h * f)], "servo": {k: {"id": v["id"], "x": round(v["x"] * f, 1), "y": round(v["y"] * f, 1)} for k, v in mj["pos"].items()}}
        json.dump(js, open(os.path.join(d, mt + ".json"), "w"), indent=1)


if __name__ == "__main__":
    main()
