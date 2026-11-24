# Công cụ render ảnh sản phẩm (media/renders-web/src)

Bộ ảnh P-L chưa render (việc S4, `docs/M1-DONE.md`). Bộ 8 ảnh cũ và MANIFEST của nó ở `archive/v0.4/media/renders-web/`, không dùng cho web hay README.

Thư mục này giữ công cụ dùng chung:

| File | Dùng cho |
|---|---|
| `views.json` | góc camera (phương vị, độ cao, tiêu cự, lấp khung) cho render Blender, `sim/tools/snapshot.py` và `web-assets/src/13_snapshots.py` |
| `render_scene.py` | dựng cảnh Blender 4.0.2 Cycles: đèn studio, nền #d6d3cc (shadow catcher + compositor), vật liệu theo bảng màu dưới |
| `overlay.py` | vẽ thước lên ảnh trực giao, số đọc từ mesh |
| `finish.py` | WebP + MANIFEST (mẫu văn bản cũ, cần sửa trước lượt P-L) |

Quy tắc giữ nguyên: chỉ WebP vào git, PNG gốc ngoài git kèm SHA-256 trong MANIFEST; không nhãn trên ảnh; chạy render trên máy render, không trên máy điều phối.

Lượt P-L dự kiến: mesh từ `cad/faibotic/export_sim_meshes.py --pl` (cùng nguồn với `sim/assets` và `web-assets/m1.glb`), hoặc render thẳng GLB bằng `web-assets/src/15_render_glb.py` (ảnh cổng hình S1G ở `media/qc/s1-glb/`).

## Màu (nguồn: software/core/packages/ui-tokens/src/tokens.css; brand/ chỉ có logo và navy #1E3A5F)
Navy chi tiết in #1e3a5f (primary), ngà nhựa #f1efe8, nền ảnh #d6d3cc, nhôm anodized xám nhạt, servo/camera/con lăn/đệm đen mờ (linh kiện mua sẵn). Không cam.
