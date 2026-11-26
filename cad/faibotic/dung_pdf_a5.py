# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Dựng docs/vi/M1_assembly_A5_v0.5.pdf (A5, tiếng Việt) từ hình bước đã render và dữ liệu trong repo.

  1. python render_pdf_buoc.py MESH_DIR IMG_DIR            (hình bước, thẻ chi tiết, bìa; xem file đó)
  2. python dung_pdf_a5.py IMG_DIR OUT.pdf [--chrome PATH] [--html OUT.html]

Dữ liệu: trinh_tu_lap.yaml (trình tự, thẻ chi tiết, ốc từng bước), media/hardware/bang-oc.csv (ốc vít),
product/danh-sach-in.csv (chi tiết in 3D), media/hardware/m1-hw-*.svg (tờ HARDWARE theo cụm).
Dựng bằng HTML + Chromium --print-to-pdf, khổ A5. Chi tiết nào chưa có hình mesh thì thẻ hiện khung trống.
"""
import base64, csv, html, os, re, subprocess, sys, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
CAM = "#F28A1E"
CHROME_MAC_DINH = [os.path.expanduser("~/.cache/ms-playwright/chromium-1255/chrome-linux64/chrome"), "chromium", "google-chrome", "chromium-browser"]


def esc(s): return html.escape(str(s), quote=True)


def b64(path, mime):
    return "data:%s;base64,%s" % (mime, base64.b64encode(open(path, "rb").read()).decode())


# ---------------------------------------------------------------- biểu tượng vẽ bằng mã (nét xám, 100 x 60)
S = 'fill="none" stroke="#222" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"'
ICON = {
    "luc-giac": '<path d="M24 6 V48 H90" %s/><path d="M24 6 L28 6" %s/><circle cx="24" cy="6" r="1" %s/>' % (S, S, S),
    "tua-vit": '<rect x="6" y="22" width="34" height="16" rx="8" %s/><path d="M40 30 H92" %s/><path d="M92 26 V34" %s/>' % (S, S, S),
    "co-le": '<path d="M10 40 L58 22" %s/><path d="M58 22 L80 12 L90 22 L84 30 L66 30 Z" %s/><path d="M10 40 L8 46 L16 48 L18 42" %s/>' % (S, S, S),
    "co-le-luc": '<rect x="6" y="26" width="70" height="8" rx="4" %s/><rect x="76" y="20" width="18" height="20" rx="3" %s/><path d="M30 26 V34 M44 26 V34" %s/>' % (S, S, S),
    "mo-han": '<rect x="6" y="24" width="30" height="12" rx="6" %s/><path d="M36 30 H78" %s/><path d="M78 30 L94 30" %s/><path d="M84 22 L94 30 L84 38" %s/>' % (S, S, S, S),
    "keo": '<path d="M12 18 H56 L84 28 V32 L56 42 H12 Z" %s/><path d="M84 28 H94 V32 H84" %s/><path d="M22 18 V42" %s/>' % (S, S, S),
    "kim": '<path d="M10 52 L44 28" %s/><path d="M10 8 L44 32" %s/><path d="M44 28 L90 18 L92 24 L52 34 L90 44 L88 50 L44 34" %s/>' % (S, S, S),
    "dong-ho": '<rect x="26" y="4" width="48" height="52" rx="6" %s/><rect x="34" y="10" width="32" height="12" %s/><circle cx="50" cy="38" r="9" %s/><path d="M50 38 L55 33" %s/>' % (S, S, S, S),
    "nguon": '<rect x="10" y="12" width="80" height="36" rx="4" %s/><circle cx="30" cy="30" r="8" %s/><path d="M54 24 H80 M54 36 H80" %s/>' % (S, S, S),
    "thuoc": '<path d="M6 40 H94" %s/><path d="M6 40 V18 M6 24 H26" %s/><path d="M40 40 V28 H60 V40" %s/><path d="M14 40 V46 M24 40 V46 M34 40 V46 M44 40 V46 M54 40 V46 M64 40 V46 M74 40 V46 M84 40 V46" %s/>' % (S, S, S, S),
}
ICON_OC = {
    "screw": '<rect x="14" y="14" width="22" height="32" rx="3" %s/><path d="M36 24 H88 V36 H36" %s/><path d="M46 24 V36 M56 24 V36 M66 24 V36 M76 24 V36" %s/><path d="M20 26 H30 M20 34 H30" %s/>' % (S, S, S, S),
    "screw-m6": '<rect x="10" y="10" width="26" height="40" rx="3" %s/><path d="M36 20 H90 V40 H36" %s/><path d="M48 20 V40 M60 20 V40 M72 20 V40" %s/>' % (S, S, S),
    "screw-ph": '<path d="M14 18 Q14 12 22 12 H30 Q38 12 38 18 V42 Q38 48 30 48 H22 Q14 48 14 42 Z" %s/><path d="M38 24 H88 V36 H38" %s/><path d="M18 24 H34 M26 20 V40" %s/><path d="M48 24 V36 M58 24 V36 M68 24 V36 M78 24 V36" %s/>' % (S, S, S, S),
    "bolt": '<path d="M12 30 Q12 10 36 10 Q36 30 36 50 Q12 50 12 30 Z" %s/><path d="M36 24 H90 V36 H36" %s/><path d="M48 24 V36 M60 24 V36 M72 24 V36 M84 24 V36" %s/>' % (S, S, S),
    "nut": '<path d="M26 8 L66 8 L86 30 L66 52 L26 52 L6 30 Z" transform="translate(4 0) scale(.9 1)" %s/><circle cx="50" cy="30" r="11" %s/>' % (S, S),
    "tnut": '<path d="M14 40 H86 V28 H70 V16 H30 V28 H14 Z" %s/><circle cx="50" cy="30" r="5" %s/>' % (S, S),
    "knob": '<circle cx="50" cy="30" r="22" %s/><path d="M50 12 V48 M32 30 H68" %s/><circle cx="50" cy="30" r="8" %s/>' % (S, S, S),
    "insert": '<rect x="30" y="10" width="40" height="40" rx="4" %s/><circle cx="50" cy="30" r="10" %s/><path d="M30 20 H70 M30 40 H70" %s/>' % (S, S, S),
    "box": '<path d="M12 22 L50 8 L88 22 V46 L50 56 L12 46 Z" %s/><path d="M12 22 L50 36 L88 22 M50 36 V56" %s/>' % (S, S),
}


def svg(inner, vb="0 0 100 60", cls=""):
    return '<svg class="%s" viewBox="%s" xmlns="http://www.w3.org/2000/svg">%s</svg>' % (cls, vb, inner)


# ---------------------------------------------------------------- dữ liệu
LOAI = {"M1-LF-058", "M1-LF-059", "M1-CM-003", "M1-CM-004"}


def doc_csv(p):
    return list(csv.DictReader(open(p, encoding="utf-8-sig")))


def so_oc(rows):
    return {r["id"]: r for r in rows}


def hinh_the(img, ma, anh=None):
    p = os.path.join(ROOT, "media", "hardware", "thumbs", anh) if anh else os.path.join(img, "the-%s.png" % ma)
    return b64(p, "image/png") if os.path.exists(p) else None


def card_ct(img, ct, nho=False):
    h = hinh_the(img, ct["ma"], ct.get("anh"))
    pic = '<img src="%s">' % h if h else '<div class="trong"></div>'
    return ('<div class="the%s"><div class="anh">%s</div><div class="tx"><b>%s</b><span>%s</span></div><div class="sl">×%s</div></div>'
            % (" nho" if nho else "", pic, esc(ct["ma"]), esc(ct["ten"]), esc(ct.get("sl", 1))))


def card_oc(o, step_oc):
    sl = step_oc.get("sl")
    if sl is None:
        sl = o.get("so_luong", "") or step_oc.get("sl_mac_dinh", "")
    don_vi = o.get("don_vi", "")
    ghi = step_oc.get("ghi", "")
    if sl == "" or sl is None:
        slt = "×1 bộ" if o["kieu"] == "box" else ""
    else:
        slt = "×%s%s" % (sl, " bộ" if don_vi == "bộ" else "")
    if ghi:
        slt = (slt + " · " if slt else "") + ghi if ghi != "tiếp" else "tiếp"
    ten = re.sub(r"\s*\((tay follower|còn lại|trạm|đế ngàm|camera cổ tay|camera đầu)\)", "", o["ten"])
    return ('<div class="the oc"><div class="anh">%s</div><div class="tx"><b>%s</b><span>Ốc vít</span></div><div class="sl">%s</div></div>'
            % (svg(ICON_OC.get(o["kieu"], ICON_OC["screw"])), esc(ten), esc(slt)))


FOOT = '<div class="ft"><span>FAIBOT M1 · %s · F A I B O T I C</span></div>'


def trang(noi_dung, so, cls=""):
    return '<section class="trang %s">%s%s</section>' % (cls, noi_dung, FOOT % so)


def dots(i, n):
    return '<div class="dots">%s</div>' % "".join('<i class="%s"></i>' % ("on" if k == i else ("xong" if k < i else "")) for k in range(n))


CSS = """
@page { size: A5; margin: 0 }
* { box-sizing: border-box; margin: 0; padding: 0 }
html, body { font-family: 'Liberation Sans','Helvetica Neue',Arial,'DejaVu Sans',sans-serif; color: #161616; background: #fff }
.trang { width: 148mm; height: 210mm; position: relative; overflow: hidden; page-break-after: always; padding: 11mm 10mm 14mm }
.ft { position: absolute; left: 0; right: 0; bottom: 6mm; text-align: center; font-size: 6.2pt; letter-spacing: .6pt; color: #777 }
h1 { font-size: 15pt; font-weight: 800; letter-spacing: .5pt; text-transform: uppercase }
.sub { font-size: 7.5pt; color: #666; margin-top: 1.5mm; line-height: 1.4 }
.bia { padding: 0; background: #fff }
.bia .top { position: absolute; left: 12mm; top: 16mm; z-index: 2 }
.bia .top h1 { font-size: 34pt; letter-spacing: 1.5pt; line-height: 1 }
.bia .top p { font-size: 11pt; color: #444; margin-top: 4mm }
.bia .bar { width: 22mm; height: 1.6mm; background: %(cam)s; margin-top: 6mm }
.bia img { position: absolute; left: 14mm; bottom: 18mm; width: 120mm }
.bia .brand { position: absolute; bottom: 9mm; left: 0; right: 0; text-align: center; font-size: 8pt; letter-spacing: 3pt; color: #444 }
.luoi { display: grid; gap: 3mm; margin-top: 5mm }
.dungcu { grid-template-columns: 1fr 1fr }
.dc { border: .25mm solid #ccc; border-radius: 1.5mm; padding: 2mm 2.5mm; display: grid; grid-template-columns: 17mm 1fr; gap: 2.5mm; align-items: center; min-height: 15mm }
.dc svg { width: 17mm; height: 10mm }
.dc b { font-size: 7.4pt; display: block; line-height: 1.2 }
.dc span { font-size: 6.4pt; color: #666; display: block; margin-top: .6mm; line-height: 1.25 }
.in3d { grid-template-columns: repeat(3, 1fr); gap: 2.6mm }
.the { border: .25mm solid #d0d0d0; border-radius: 1.6mm; background: #fff; position: relative }
.in3d .the { padding: 1.6mm; height: 39mm; display: flex; flex-direction: column }
.in3d .anh { flex: 1; min-height: 0; display: flex; align-items: center; justify-content: center }
.anh { display: flex; align-items: center; justify-content: center }
.anh img { max-width: 100%%; max-height: 100%%; object-fit: contain }
.anh svg { width: 90%%; height: 90%% }
.trong { width: 60%%; height: 60%%; border: .3mm dashed #ccc; border-radius: 1mm }
.the .tx b { display: block; font-size: 6.6pt; letter-spacing: .1pt }
.the .tx span { display: block; font-size: 5.8pt; color: #666; line-height: 1.2; margin-top: .3mm }
.the .sl { position: absolute; right: 1.6mm; top: 1mm; font-size: 8pt; font-weight: 700; color: #161616 }
.hw { padding: 0 }
.hw img { width: 148mm; height: 196mm; object-fit: contain; object-position: center top; display: block }
.doc { font-size: 8pt; line-height: 1.45; margin-top: 4mm }
.doc p { margin-top: 2.2mm }
.doc li { margin: 1.6mm 0 0 4mm }
.doc b { font-weight: 700 }
.cach { display: grid; grid-template-columns: 60mm 1fr; gap: 3mm; margin-top: 4mm }
.cach .hn img.hinh { height: 118mm; object-fit: contain }
.cot2 { display: flex; flex-direction: column; gap: 3mm }
.cach .k { border: .25mm solid #ccc; border-radius: 2mm; padding: 2.5mm; position: relative }
.cach .k h3 { font-size: 8pt; margin-bottom: 1.6mm }
.cach .k p { font-size: 6.8pt; color: #444; line-height: 1.35; margin-top: 1.6mm }
.cach .k img.hinh { width: 100%%; display: block }
.cach .the { height: 26mm; padding: 1.6mm; display: flex; flex-direction: column; width: 32mm; margin: 0 }
.cach .the .anh { height: 14mm; flex: none }
.cach .the .tx { margin-top: 1mm }
.vong { display: inline-flex; width: 11mm; height: 11mm; border-radius: 50%%; background: #111; color: #fff; align-items: center; justify-content: center; font-weight: 800; font-size: 12pt; flex: none }
.cum { font-size: 6.6pt; letter-spacing: 1.2pt; text-transform: uppercase; color: #777 }
.bt-h { display: flex; align-items: center; gap: 3.5mm; height: 14mm }
.bt-h .vong { width: 11mm; height: 11mm }
.bt-h h2 { font-size: 11pt; line-height: 1.15; font-weight: 800 }
.dots { display: flex; gap: 1.1mm; position: absolute; top: 5.4mm; right: 10mm }
.dots i { width: 1.7mm; height: 1.7mm; border-radius: 50%%; background: #fff; border: .25mm solid #aaa; display: block }
.dots i.xong { background: #aaa }
.dots i.on { background: %(cam)s; border-color: %(cam)s }
.bt-body { display: grid; grid-template-columns: 37mm 1fr; gap: 4mm; margin-top: 3mm; height: 158mm }
.bt-body.cb { height: 137mm }
.cot { display: flex; flex-direction: column; gap: 1.8mm; min-height: 0 }
.cot .the { display: grid; grid-template-columns: 1fr; grid-template-rows: 1fr auto; padding: 1.5mm; flex: 0 0 auto; height: 31mm }
.cot .the .anh { min-height: 0; height: 100%% }
.cot .the .anh img { max-height: 100%% }
.cot .the.nho { height: 13.2mm; grid-template-rows: 1fr }
.cot .the.nho { grid-template-columns: 13mm 1fr; gap: 1.5mm; align-items: center }
.cot .the.nho .anh { height: 10.5mm }
.cot .the.nho .tx { min-width: 0; padding-right: 6mm }
.cot .the.nho .sl { top: 1mm }
.cot .the.oc .anh svg { width: 80%%; height: 80%% }
.hinhbuoc { border-left: .25mm solid #e4e4e4; display: flex; align-items: center; justify-content: center; min-height: 0 }
.hinhbuoc img { width: 100%%; max-height: 100%%; object-fit: contain }
.canhbao { position: absolute; left: 10mm; right: 10mm; bottom: 24mm; border: .35mm solid %(cam)s; background: #fff5ea; border-radius: 1.5mm; padding: 1.6mm 2.4mm; font-size: 6.8pt; line-height: 1.35; color: #222 }
.canhbao b { color: #c2410c; margin-right: 1mm }
.cb-tong { border: .35mm solid %(cam)s; background: #fff5ea; border-radius: 1.5mm; padding: 2mm 2.6mm; margin-top: 3mm; font-size: 7.6pt; line-height: 1.4 }
.mota { position: absolute; left: 10mm; right: 10mm; bottom: 13mm; font-size: 7.6pt; line-height: 1.4; color: #333; border-top: .25mm solid #ddd; padding-top: 2mm }
.ghichu { font-size: 6.2pt; color: #888; margin-top: 2mm }
.idt { width: 100%%; border-collapse: collapse; margin-top: 4mm; font-size: 8pt }
.idt td, .idt th { border: .25mm solid #ccc; padding: 1.6mm 2mm; text-align: left }
.idt th { background: #f3f3f3 }
.tomtat { display: grid; grid-template-columns: 1fr 1fr; gap: 3mm; margin-top: 4mm }
.tomtat .k { border: .25mm solid #ccc; border-radius: 1.5mm; padding: 2.5mm; font-size: 7.4pt; line-height: 1.4 }
""" % {"cam": CAM}


W_HW, H_MAX = 900, 1180       # rộng SVG (px) và cao tối đa của một trang A5 ở rộng đó


def hardware_pages():
    """Dùng lại build_hardware_card.py (cùng BOM, the.csv, bang-oc.csv), đổi bố cục: 3 cột thẻ, rộng 900, chia nhiều trang khi dài."""
    sys.path.insert(0, os.path.join(ROOT, "media", "hardware"))
    import build_hardware_card as B
    bom, the, oc, leader = B.load_all()
    cards = B.build_cards(bom, the, oc, leader)
    files = B.spec_files(the, leader)
    T = B.THEMES["light"]
    cols = {"thumb": 4, "text": 3, "oc": 2}
    out = []
    for name in ("m1-hw-de", "m1-hw-truc-nang", "m1-hw-tram", "m1-hw-dien"):
        title, sub, secs, _w, _foot, hero, intro = B.sheets(cards)[name]
        secs = [(l, k, c, cols[k]) for l, k, c, _ in secs if c]
        # chia: mỗi phần tử là (nhãn, loại, thẻ, cột); cắt nhỏ phần quá dài
        units = []
        for l, k, c, n in secs:
            per = {"thumb": 4, "text": 3, "oc": 8}[k]
            for i in range(0, len(c), per):
                units.append((l if i == 0 else " ", k, c[i:i + per], n))
        pages, cur = [], []
        first = True
        def tao(us, dau):
            gop = []
            for u in us:       # nối các hàng cùng một phần (nhãn " ") để không chèn khoảng cách giữa hàng
                if gop and u[0] == " " and gop[-1][1] == u[1]:
                    gop[-1] = (gop[-1][0], gop[-1][1], gop[-1][2] + u[2], gop[-1][3])
                else:
                    gop.append(u)
            us = gop
            return B.build_sheet(title if dau else title + " (tiếp)", sub, us, files, T, W_HW, margin=40, gap=12,
                                 hero=(hero[0], 170, 206, hero[3]) if dau else None, intro=intro if dau else "")
        for u in units:
            thu = cur + [u]
            if cur and tao(thu, not pages)[2] > H_MAX:
                pages.append(cur); cur = [u]
            else:
                cur = thu
        if cur:
            pages.append(cur)
        for i, us in enumerate(pages):
            out.append(tao(us, i == 0)[0])
    return out


def dung(img, out_pdf, chrome=None, html_out=None):
    cfg = yaml.safe_load(open(os.path.join(HERE, "trinh_tu_lap.yaml"), encoding="utf-8"))
    oc = so_oc(doc_csv(os.path.join(ROOT, "media", "hardware", "bang-oc.csv")))
    buoc = cfg["buoc"]
    P = []
    n = [1]

    def page(c, cls=""):
        n[0] += 1
        P.append(trang(c, n[0], cls))

    # ---- bìa
    bia = b64(os.path.join(img, "bia.png"), "image/png")
    P.append('<section class="trang bia"><div class="top"><h1>FAIBOT M1</h1><p>Hướng dẫn lắp ráp · v0.5</p><div class="bar"></div></div>'
             '<img src="%s"><div class="brand">F A I B O T I C</div></section>' % bia)

    # ---- trước khi bắt đầu
    page('<h1>Trước khi bắt đầu</h1><div class="doc">'
         '<p>Tài liệu này dẫn lắp một robot Faibot M1 cấu hình chuẩn: đế di động, trục nâng, trạm hai tay và phần điện của trạm. Các bước theo thứ tự lắp.</p>'
         '<p><b>Lưu ý bản v0.5</b></p><ul>'
         '<li>Hình dựng từ mô hình CAD. Bản này chưa lắp thử trên robot thật; số liệu mô-men siết chưa chốt, ghi chú đo và ghi khi lắp.</li>'
         '<li>Một số chi tiết trong hình chưa được vẽ, ví dụ thân servo của tay. Hình sẽ cập nhật ở các bản sau.</li>'
         '<li>In chi tiết nhựa trước (trang CHI TIẾT IN 3D). Gán ID cho từng servo, mỗi lần một servo trên bus.</li>'
         '<li>Không đấu pin thật khi chưa qua cổng kiểm điện. Mọi lần cấp điện thử dùng nguồn bàn 12 V có giới hạn dòng.</li></ul>'
         '<div class="cb-tong"><b>An toàn điện, đọc trước khi lắp</b><ul>'
         '<li>Vòng E-stop của trạm: sau khi tháo trạm, nửa giắc 21W4 trên xe vẫn mang 12 V ở chân 1 và chân 9. Không chạm, không để vật dẫn rơi vào.</li>'
         '<li>Hai nắp giắc: nắp che thường ngày (không cầu nối) dùng hằng ngày. Nắp bảo trì (có cầu nối, khác màu) chỉ kỹ thuật viên dùng, không để lại trên robot.</li>'
         '<li>DOCK: hai tay chỉ có điện khi cả hai khoá kéo của trạm đã đóng.</li>'
         '<li>Bố trí chân 21W4 còn là giả định: đối chiếu giắc thật trước khi hàn.</li></ul></div>'
         '<p><b>Thứ tự</b></p><ul><li>Đế di động</li><li>Trục nâng, kèm đế ngàm trên xe nâng</li><li>Trạm hai tay, dựng trên bàn: khung, gá vai, khoá, cột camera, hai tay, rồi giắc, E-stop, hộp điện</li><li>Đặt trạm lên đế ngàm, khoá và kiểm DOCK</li></ul>'
         '</div>')

    # ---- dụng cụ
    dc = [
        ("luc-giac", "Lục giác 1,5 mm", "Ốc M2"),
        ("luc-giac", "Lục giác 2,5 mm", "Ốc M3 (phần lớn ốc của tay)"),
        ("luc-giac", "Lục giác 3 mm", "Ốc M4"),
        ("luc-giac", "Lục giác 4 mm", "Ốc và núm khoá M5"),
        ("luc-giac", "Lục giác 5 mm", "Ốc M6, vít đầu dù M8×35"),
        ("co-le", "Cờ lê 13 mm", "Đai ốc M8"),
        ("tua-vit", "Tua vít Phillips PH0, PH1", "Ốc M2×12 của camera"),
        ("co-le-luc", "Cờ lê lực hoặc tua vít lực", "Đo mô-men siết rồi ghi vào phiếu"),
        ("mo-han", "Mỏ hàn, đầu ép hạt chèn nhiệt M3", "Ép hạt chèn nhiệt (heat-set) vào cổ tay"),
        ("keo", "Keo epoxy AB", "Dán các mối nối trụ"),
        ("kim", "Kìm tuốt dây, kìm bấm cos", "Dây nguồn và đầu cos; kèm ống co nhiệt, súng khò"),
        ("dong-ho", "Đồng hồ vạn năng", "Kiểm thông mạch, trước khi cấp điện"),
        ("nguon", "Nguồn bàn 12 V có giới hạn dòng", "Cấp điện thử lần đầu"),
        ("thuoc", "Thước kẹp, thước đo góc, ke", "Đo khoảng vai, kiểm vuông khung"),
    ]
    page('<h1>Dụng cụ</h1><p class="sub">Cỡ lục giác theo ốc trong bảng ốc vít của robot.</p><div class="luoi dungcu">%s</div>' %
         "".join('<div class="dc">%s<div><b>%s</b><span>%s</span></div></div>' % (svg(ICON[k]), esc(t), esc(m)) for k, t, m in dc))

    # ---- bảng ID servo (18 servo, 6+1)
    idr = [("Xoay đế tay", "STS3095", 1, 7), ("Vai", "STS3095", 2, 8), ("Khuỷu", "STS3095", 3, 9), ("Gập cổ tay", "STS3215", 4, 10),
           ("Yaw cổ tay", "STS3215", 21, 22), ("Cuộn cổ tay", "STS3215", 5, 11), ("Kẹp", "STS3215", 6, 12)]
    mc = [("Bánh omni 0", "STS3215", 13), ("Bánh omni 1", "STS3215", 14), ("Bánh omni 2", "STS3215", 15), ("Trục nâng", "STS3095", 16)]
    tr = "".join("<tr><td>%s</td><td>%s</td><td>%d</td><td>%d</td></tr>" % r for r in idr)
    tm = "".join("<tr><td>%s</td><td>%s</td><td colspan=2>%d</td></tr>" % r for r in mc)
    page('<h1>Bảng ID servo</h1><p class="sub">18 servo: 7 STS3095 và 11 STS3215, tất cả 12 V. Gán ID trước khi lắp, mỗi lần một servo trên bus, rồi dán nhãn ID lên thân servo.</p>'
         '<table class="idt"><tr><th>Khớp</th><th>Servo</th><th>Tay trái</th><th>Tay phải</th></tr>%s%s</table>'
         '<div class="cb-tong"><b>Lưu ý</b><ul><li>Yaw cổ tay (ID 21 trái, 22 phải) là servo của tay 6+1 như AM-ARM200 gốc, lắp vào hốc yaw trong vỏ cổ tay.</li>'
         '<li>Hướng dẫn gốc của trụ dùng ID khác cho bánh và trục nâng; robot này theo bảng trên.</li></ul></div>' % (tr, tm))

    # ---- chi tiết in 3D
    ds = [r for r in doc_csv(os.path.join(ROOT, "product", "danh-sach-in.csv"))
          if r["tinh_vao_tong"] in ("có", "chưa") and not r["cum"].startswith("Bộ đế bàn")
          and "đề xuất" not in r["ma"]]
    cards, seen = [], set()
    for r in ds:
        m = re.search(r"M1-[A-Z]{2}-\d{3}[a-z]?", r["ma"])
        if not m:
            continue
        ma = m.group(0)
        if ma in LOAI:      # chi tiết đã bỏ khỏi cấu hình P-L (khung T trái/phải, giá camera ngực)
            continue
        hau = ""
        if "vách" in r["ma"]: hau = " vách"
        if "nắp" in r["ma"] and "BS-036" in ma: hau = " nắp"
        key = ma + hau
        if key in seen:
            continue
        seen.add(key)
        ten = re.sub(r"\s*\(.*?\)", "", r["ten"]).strip()
        ten = ten.split(":")[-1].strip() if ma.startswith("M1-BS-036") else ten
        ten = ten.split(",")[0].strip()
        cards.append(dict(ma=ma + hau if hau else ma, thumb=ma, ten=ten[:46], sl=r["so_luong_moi_robot"]))
    cards += [dict(ma="M1-TR-024 (đề xuất)", thumb="M1-TR-024 (đề xuất)", ten="Nắp che giắc thường ngày", sl=1),
              dict(ma="M1-TR-025 (đề xuất)", thumb="M1-TR-025 (đề xuất)", ten="Nắp giắc bảo trì", sl=1)]
    per = 12
    chunks = [cards[i:i + per] for i in range(0, len(cards), per)]
    for ci, ch in enumerate(chunks):
        page('<h1>Chi tiết in 3D</h1><p class="sub">%d chi tiết, vật liệu PETG. Số bên phải thẻ là số lượng cho một robot.%s</p><div class="luoi in3d">%s</div>'
             % (len(cards), "" if ci == 0 else " (tiếp)",
                "".join(card_ct(img, dict(ma=c["thumb"], ten=c["ten"], sl=c["sl"])).replace(esc(c["thumb"]), esc(c["ma"]), 1) for c in ch)))

    # ---- HARDWARE theo cụm (thẻ dựng lại từ cùng dữ liệu của media/hardware, cỡ A5)
    for svg_txt in hardware_pages():
        page('<img src="data:image/svg+xml;base64,%s"><div class="ghichu" style="position:absolute;left:10mm;right:10mm;bottom:11mm">* Số lượng giả định hoặc đang khớp lại. ×? chưa xác định. Số lượng cho một robot chuẩn.</div>' % base64.b64encode(svg_txt.encode("utf-8")).decode(), "hw")

    # ---- cách đọc
    ex = buoc[5]
    exi = b64(os.path.join(img, "buoc-%s.png" % ex["id"]), "image/png")
    ct0 = ex["chi_tiet"][0]
    page('<h1>Cách đọc hình</h1><p class="sub">Mỗi bước một trang. Cột trái liệt kê thứ cần dùng, bên phải là hình nổ.</p>'
         '<div class="cach">'
         '<div class="k hn"><h3>Hình nổ</h3><img class="hinh" src="%s">'
         '<p><b style="color:%s">Cam</b>: chi tiết mới của bước, kéo ra khỏi vị trí lắp.<br><b style="color:#888">Xám</b>: phần đã lắp ở các bước trước.<br>Nét đứt và mũi tên chỉ đường đưa chi tiết về chỗ.</p></div>'
         '<div class="cot2">'
         '<div class="k"><h3>Thẻ chi tiết</h3>%s<p>Ảnh, mã chi tiết, và số lượng ×N cho bước này.</p></div>'
         '<div class="k"><h3>Thẻ ốc vít</h3>%s<p>Loại ốc, cỡ, và số lượng cần dùng.</p></div>'
         '<div class="k"><h3>Số bước</h3><div style="display:flex;gap:3mm;align-items:center"><span class="vong">%d</span><span class="cum">%s</span></div><p>Số bước nằm trong vòng tròn đen, kèm tên cụm đang lắp.</p></div>'
         '<div class="k"><h3>Chấm tiến độ</h3><div style="position:relative;height:3mm">%s</div><p>Chấm cam là bước đang xem, chấm xám là bước đã xong. Dãy chấm nằm ở góc trên bên phải mỗi trang.</p></div>'
         '</div></div>' % (exi, CAM, card_ct(img, ct0), card_oc(oc["F04"], {}), 6, esc(cfg["cum"]["truc"]),
                           dots(5, len(buoc)).replace('class="dots"', 'class="dots" style="top:0;left:0;right:auto"')))

    # ---- các bước
    tong = len(buoc)
    for i, b in enumerate(buoc):
        nct = len(b["chi_tiet"]) + len(b.get("oc", []))
        cbao = b.get('canh_bao')
        nho = nct > 5 or bool(cbao)
        cot = [card_ct(img, ct, nho) for ct in b["chi_tiet"]]
        for o in b.get("oc", []):
            cot.append(card_oc(oc[o["id"]], o).replace('class="the oc"', 'class="the oc%s"' % (" nho" if nho else ""), 1))
        hinh = b64(os.path.join(img, "buoc-%s.png" % b["id"]), "image/png")
        page('<div class="bt-h"><span class="vong">%d</span><div><div class="cum">%s</div><h2>%s</h2></div></div>%s'
             '<div class="bt-body%s"><div class="cot">%s</div><div class="hinhbuoc"><img src="%s"></div></div>'
             '%s<div class="mota">%s</div>' % (i + 1, esc(cfg["cum"][b["cum"]]), esc(b["ten"]), dots(i, tong), " cb" if cbao else "", "".join(cot), hinh,
             ('<div class="canhbao"><b>Lưu ý</b> %s</div>' % esc(cbao)) if cbao else "", esc(b["mo_ta"])))

    # ---- sau khi lắp
    page('<h1>Sau khi lắp</h1><div class="doc"><p>Làm theo thứ tự này trước khi cho robot chạy.</p><ul>'
         '<li>Đo và ghi mô-men siết các mối chính, ghi vào phiếu kiểm.</li>'
         '<li>Quay tay từng bánh, con lăn không kẹt. Ba bánh chạm sàn cùng lúc.</li>'
         '<li>Kiểm vòng E-stop bằng đồng hồ khi chưa cấp điện: thông khi hai khoá đóng, hở khi mở một khoá hoặc nhấn E-stop.</li>'
         '<li>Đo khoảng vai của trạm: 0,475 m.</li>'
         '<li>Cấp điện lần đầu bằng nguồn bàn 12 V có giới hạn dòng. Không đấu pin thật khi chưa qua cổng kiểm điện.</li></ul>'
         '<div class="cb-tong"><b>Nhắc lại về giắc và nắp</b><ul>'
         '<li>Sau khi tháo trạm, nửa giắc 21W4 trên xe vẫn mang 12 V ở chân 1 và chân 9. Đậy nắp che thường ngày (không cầu nối).</li>'
         '<li>Nắp bảo trì (có cầu nối, khác màu) chỉ kỹ thuật viên dùng, tháo ra và mang đi, không để lại trên robot.</li>'
         '<li>DOCK: hai tay chỉ có điện khi cả hai khoá kéo đã đóng.</li></ul></div>'
         '<p>Đi dây, vòng dịch vụ trục nâng, Pi 5 và bo bus đế theo hướng dẫn đi dây riêng.</p></div>')

    doc = '<!doctype html><html lang="vi"><head><meta charset="utf-8"><title>Faibot M1 · Hướng dẫn lắp ráp · v0.5</title><style>%s</style></head><body>%s</body></html>' % (CSS, "".join(P))
    hp = html_out or out_pdf + ".html"
    open(hp, "w", encoding="utf-8").write(doc)
    exe = chrome
    if not exe:
        for c in CHROME_MAC_DINH:
            if os.path.exists(c) or subprocess.run(["which", c], capture_output=True).returncode == 0:
                exe = c; break
    r = subprocess.run([exe, "--headless=new", "--no-sandbox", "--disable-gpu", "--no-pdf-header-footer", "--print-to-pdf=" + os.path.abspath(out_pdf),
                        "file://" + os.path.abspath(hp)], capture_output=True, text=True)
    if not os.path.exists(out_pdf):
        sys.exit("Chromium không ra PDF: " + r.stderr[-400:])
    return n[0]


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("img"); ap.add_argument("out"); ap.add_argument("--chrome"); ap.add_argument("--html")
    a = ap.parse_args()
    print("trang:", dung(a.img, a.out, a.chrome, a.html))
