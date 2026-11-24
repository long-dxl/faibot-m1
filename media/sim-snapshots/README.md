# Ảnh chụp sim Faibot M1 (MuJoCo)

Ảnh sim 4 góc của cấu hình P-L chưa chụp lại (việc S4). Bộ ảnh cũ và ảnh so sánh SO-101 cũ ở `archive/v0.4/media/sim-snapshots/`.

- `lab/`: khung chụp từ bài lab (`demos/lab_*.py --snap`, ghi đè mỗi lần chạy). Ảnh hiện có là lượt chụp trước P-L; chạy lại bài lab trên máy có GL offscreen để cập nhật.

## Chụp lại
```sh
# mesh sim lấy từ CAD P-L (cùng nguồn với web-assets/m1.glb)
python cad/faibotic/export_sim_meshes.py --pl --out sim/assets
cd sdk && python -m faibotic.model --out ../sim && cd ..
# 4 góc theo media/renders-web/src/views.json, cần EGL hoặc OSMesa
MUJOCO_GL=egl .venv/bin/python sim/tools/snapshot.py --scale 0.5
```
So sánh với SO-101 (MuJoCo Menagerie `robotstudio_so101`, Apache-2.0): `sim/tools/snapshot.py --ref <đường dẫn so101.xml>`.
