# web-assets: tài sản 3D cho trang Faibot M1 (cùng nguồn với sim)

Bản P-L, kẹp gốc AM-ARM200 (24/11/2026; S1G 23/11 có kẹp cũ). Một nguồn hình học: CAD `cad/faibotic/assembly_pl.py` (thân) + STEP AM-ARM200 (cả kẹp J7, `cad/faibotic/gripper_am.py`) → `sim/assets/*.stl` (`cad/faibotic/export_sim_meshes.py --pl`) → `m1.glb`. Cây node và trục khớp lấy từ `sim/m1.xml`, giới hạn từ `sdk/faibotic/spec.py`. Không xuất lại từ STEP. Bản cũ (dựng từ STEP riêng) ở `archive/v0.4/web-assets/legacy-web-sim/`.

| File | Nội dung |
|---|---|
| `m1.glb` | robot P-L 6+1 (GLB-V05F), thô 3,67 MB (trần 4 MB), brotli 1,16 MB (trần 1,2 MB), 89.698 tam giác (trần 150.000), 73 node (17 node khung + node lá theo cụm, mỗi servo một node lá), 10/10 cụm có lưới, PBR màu brand |
| `joints.json` | node ↔ khớp SDK/MJCF, trục (từ m1.xml), giới hạn mềm (spec.py), bảng ghép, tư thế home/ready/straight/reach/rest, TCP |
| `cum.json` | 10 cụm → mã M1 (`product/parts-register.csv`) → chi tiết CAD; `ma_chinh` theo `src/cum_ma_chinh.py` |
| `manifest.json` | file, sha256, nguồn, ngân sách, mã chính, decimate, `lab-scene.json` |
| `lab-scene.json` | chép nguyên byte từ `sim/lab/lab-scene.json` (nguồn `sim/lab/build_lab.py`) |
| `schema/*.schema.json` | JSON Schema cho manifest, joints, cum, lab-scene |
| `limits.mjs` | `elbowMax`, `wristMax`, `clampArm` (bản JS của spec.py; giới hạn P-L vẫn lấy của P-A, DK-N38) |
| `kiem.json`, `build_stat.json` | kết quả `12_validate.py` và thống kê dựng |
| `LICENSE`, `NOTICE` | `m1.glb`: CC BY-NC-SA 4.0 (phần Faibotic vẽ), phần AM-ARM200 (gồm kẹp J7), AlohaMini2, Menagerie bên trong giữ Apache-2.0; tệp dữ liệu và `src/`: Apache-2.0 (`LICENSE`). Danh sách đầy đủ: `NOTICE` ở gốc repo. Phục vụ cùng GLB |

**Giấy phép kẹp:** GLB dùng kẹp gốc AM-ARM200 (thân J7, hàm J7, giá camera OB_F_Camera; Apache-2.0), không còn hình kẹp song song cũ (giấy phép upstream đang đổi; lưu nội bộ). DK-N44 đóng.

## Dùng trong three.js
- Node khớp tên theo khớp SDK: `base`, `lift`, `left_pan`, `left_shoulder`, `left_elbow`, `left_wrist_flex`, `left_wrist_yaw`, `left_wrist_roll`, `left_gripper` (và `right_*`). Gốc node = gốc body MJCF; trục local theo `joints.json`, không quay ở tư thế 0: `node.quaternion.setFromAxisAngle(new THREE.Vector3(...axis), q)`; lift và ngón: tịnh tiến dọc trục. Node `m1` quay CAD (z lên, x trước) sang Y-up; robot nhìn về -Z.
- Kẹp: `joints.<bên>_gripper` là pct 0..100, khớp quay: hàm động ở node `<bên>_gripper` (gốc tại trục kẹp) quay `pct/100 * open_rad` (64 độ) quanh `axis`; thân kẹp, ngón cố định và giá camera nằm ở node `<bên>_wrist_roll`. 0 = hai ngón chạm; 100 = đầu ngón cách ~97 mm.
- Cụm: node lá `<cum>.<node_khớp>.<vật liệu>` (`extras.cum`), nằm dưới node khớp nên đi theo khớp. Tách lớp: dịch theo `tam_world_home_m` của cụm trong `cum.json`.
- Đế di chuyển (base_x, base_y, base_yaw) và bánh quay không có node: web dời node `m1`. Yaw cổ tay (6+1): node `<bên>_wrist_yaw` quay quanh +z của node cổ tay, giữa `wrist_flex` và `wrist_roll`.

## GLB-V05F (24/11/2026): tay 6+1 như AM-ARM200 gốc
- Khớp yaw cổ tay mở: node `<bên>_wrist_yaw` (gốc lệch (28,13; ∓3,81; -1,30) mm so với trục gập), vỏ J56 + servo cuộn + servo yaw (link URDF `left_wrist_yaw`) đi theo node này; thân kẹp, camera cổ tay và servo kẹp ở `<bên>_wrist_roll`.
- Servo yaw STS3215 (ID 21/22) tách từ chồng servo y < 20 mm của mesh `left_wrist_yaw` (`derived/left_wrist_yaw__servo__left_wrist_yaw.stl`). Khoá yaw M1-AR-025 bỏ.
- 18 servo (7 STS3095 + 11 STS3215), mỗi servo một node lá. Bù ngân sách: thân servo lấy từ mesh URDF rút còn ≤ 1.000 tam giác (tham số thứ 12 của 10_build).
- GLB 3.668.788 byte, brotli 1.163.359, 89.698 tam giác, 73 node. Lệnh: `python src/10_build_m1_glb.py REPO parts_zero.npz OUT 0.10 24 120 0.3 30 30 CAD_MESH_DIR 1600 1000`.

## v0.5e (24/11/2026): cổ tay, kẹp và 2 servo theo mesh URDF follower
- Cổ tay J5 (`left_Wrist_Pitch_Roll`, node wrist_flex), vỏ J56 (`left_wrist_yaw`, yaw khoá nên gộp vào node wrist_flex), thân kẹp J7 (`left_Fixed_Jaw`, node wrist_roll), hàm (`left_Moving_Jaw`, node gripper, đặt ở góc khép URDF 1,12757 rad = kẹp 0 SDK), camera cổ tay (`left_camera`, node wrist_roll, cụm camera) lấy từ mesh URDF follower thay cho STEP/sim.
- Servo cuộn và servo kẹp (STS3215) tách từ mesh `left_wrist_yaw` (3 chồng servo). Servo yaw là chi tiết riêng nên ĐÃ BỎ (M1 khoá yaw, BOM không có servo yaw). Servo kẹp nằm trong mesh wrist_yaw upstream nhưng gắn vào node wrist_roll để quay theo khớp cuộn. Khoá yaw M1-AR-025 giữ từ sim.
- Bù ngân sách: STS3095 1.200 tam giác, cánh tay ≤ 4.000, cẳng tay ≤ 3.500. Thân trắng sau trạm (khung xe nâng) chưa làm: không còn chỗ trong brotli 1,2 MB.
- GLB 3.784.548 byte, brotli 1.192.206, 91.928 tam giác, 69 node, 16 servo (trước: 3.525.896, 1.177.794, 88.756, 73).

## v0.5d (24/11/2026): chi tiết hơn, trần mới
- Tay J1–J4: mesh link URDF follower giữ ≥ 70% tam giác (cẳng tay tối đa 4.500), servo STS3095 ~1.500 tam giác, STS3215 3.124 (giữ tai bắt, nắp, còi; không gộp lưới thành khối), bu lông/ổ bi 150 tam giác mỗi chiếc.
- Trụ cột AlohaMini2 (M1-LF-051..055): lấy mesh CAD từng chi tiết (`cad/faibotic/assembly_pl.py --lift -160 --mesh DIR`, tham số thứ 10 của 10_build), rút quadric còn ≤ 1.600 tam giác mỗi chi tiết, không gộp lưới (mesh sim gộp lưới 1.200 tam giác gây cạnh răng cưa). Sim không đổi.
- Mọi mesh: hàn đỉnh trước khi tính pháp tuyến, pháp tuyến mượt theo góc gấp 30 độ.
- Trần: tam giác 40.000 → 150.000, thô 1,5 → 4 MB (`11_manifest.py`); trần web giữ brotli ≤ 1,2 MB. Loader M1 của web (`robot3d.ts`) chưa có MeshoptDecoder/DRACOLoader nên GLB không nén meshopt/Draco.
- GLB 3.525.896 byte, brotli 1.177.794, 88.756 tam giác, 73 node. Lệnh: `python src/10_build_m1_glb.py REPO parts_zero.npz OUT 0.10 24 120 0.3 30 30 CAD_MESH_DIR 1600`.

## v0.5c (24/11/2026): J1–J4 và servo tay theo mesh URDF AM-ARM200 follower (chỉ GLB)
J1 (đế tay), J2, J3, J4 và thân servo pan/vai/khuỷu (STS3095), gập (STS3215) lấy từ mesh link URDF upstream follower (có sẵn servo, tai bắt, ổ bi), đặt bằng FK URDF ở tư thế 0 SDK (cùng động học SDK/MJCF), tay phải gương y. Nguồn tách: `cad/upstream/AM-ARM/am-arm200/urdf/derived/` (`cad/faibotic/extract_urdf_servos.py`). Cổ tay, kẹp, servo cuộn/kẹp, bánh, trục nâng giữ từ sim/assets. Mesh Pro (`am-arm200-pro`) không dùng: J1→J2 cao 62,6 mm (follower 39,7), trục vai lệch (0; -29,0; 58,2) so với (0; -58,0; 108,6) mm, cổ tay lệch 2,2–3,6 mm. sim/MJCF chưa có thân servo pan/vai/khuỷu (DK-N62). GLB 1,40 MB, brotli 0,48 MB, 30.847 tam giác, 73 node.

### v0.5c bước trước: khối STS3095 dựng theo kích thước (đã thay)
Mỗi servo (16, như BOM A01 + A02 và `spec.SERVO_IDS`) là một node lá riêng `<cum>.<node>.servo_body.<khoá SDK>` với `extras.servo`, `extras.servo_id`, `extras.model` (STS3095/STS3215); node lá khác giữ tên `<cum>.<node>.<vật liệu>`. Vật liệu `servo_body` đổi sang xám đen ánh kim (#2b2d31). `scripts/kiem_cheo.py` dòng web↔BOM đếm servo GLB theo loại = BOM. 67 node, GLB 1,45 MB, brotli 0,58 MB, 35.271 tam giác.
STEP follower AM-ARM200 không có servo, nên J3 trông rời J2 và J4 (khe 3–5 cm ở khuỷu). GLB thêm thân STS3095 (hình theo `cad/faibotic/servos.py`, 30 tam giác + còi) ở 3 khớp mỗi tay: còi trên trục khớp, thân ở node cha (pan: `lift`, vai: `<bên>_pan`, khuỷu: `<bên>_shoulder`), cụm `shoulder_*`, mã M1-SV-002. Thêm 6 node lá `shoulder_*.<node>.servo_body` (59 → 65 node). Vị trí [giả định], dò bằng `media/qc/glb-v05b/servo_fit.py`; sim/MJCF chưa có (DK-N62). GLB 1,44 MB, brotli 0,58 MB, 35.271 tam giác.

## v0.5b (24/11/2026): mượt hơn, không đổi hợp đồng
Cây node, tên node, joints.json, lab-scene.json giữ nguyên byte. Pháp tuyến mượt theo góc gấp 30 độ (thay `smooth_shaded` của trimesh), tỉ lệ rút 0,06 → 0,12 cho thành phần > 160 tam giác. GLB 1,34 → 1,42 MB, brotli 0,49 → 0,58 MB, 32.269 → 34.743 tam giác. Trụ cột đã rút ở mesh sim (≤ 1.200 tam giác/chi tiết) nên chỉ mượt nhờ pháp tuyến. Ảnh kiểm: `media/qc/glb-v05b/`.

## Thay đổi hợp đồng (kẹp gốc AM-ARM200, 24/11/2026)
1. Bỏ node `left_gripper_b`, `right_gripper_b`. `joints.<bên>_gripper` thành `revolute` (trục ±z khung tay), bỏ `travel_m`, `mimic_node`, `mimic_axis`, `servo_rad_per_m`; thêm `open_rad` (1,117 rad), `max_tip_gap_m` (0,0973). Node `<bên>_gripper` là con của `<bên>_wrist_roll`, gốc tại trục kẹp (21,6; ±20,2; -2,0) mm.
2. `tcp.by_side` = (0,09382; ±0,01013; -0,00016) m: tâm kẹp gần cổ tay hơn 31,7 mm và lệch ngang 10,1 mm về phía ngón cố định (kẹp cũ: (0,12547; 0; 0)).
3. Cụm `gripper_*` = thân J7 M1-AR-007, hàm J7 M1-AR-008, giá camera M1-AR-027, servo kẹp; `ma_chinh` M1-AR-007 (trước M1-GR-001). Camera cổ tay đặt theo khung left_camera URDF [vị trí đề xuất].
4. `lab-scene.json`: `gripper_max_gap_m` 0,084 → 0,0973.
5. GLB 1,34 MB, brotli 0,49 MB, 32.269 tam giác, 59 node (trước: 1,43 MB, 0,51 MB, 33.265, 73 node). Ảnh cổng hình và video có kẹp mới cần render lại (Blender chưa có trên máy tính toán).

## Thay đổi hợp đồng so với bản trước P-L (S1G)
Giữ nguyên: tên 10 cụm (`base`, `power`, `lift`, `shoulder_L/R`, `arm_L/R`, `gripper_L/R`, `camera`), tên node khớp cũ, schema `faibot-m1-joints/2`, `faibot-m1-cum/2`, ngân sách.
1. Thêm node `left_gripper_b`, `right_gripper_b` (ngón b của kẹp cũ, con của `<bên>_wrist_roll`). `joints.<bên>_gripper` thêm `mimic_node`, `mimic_axis`, `servo_rad_per_m`; loại khớp là `prismatic` (trục ngón a). Hành trình mỗi ngón 42 mm (trước 45 mm giả định).
2. Hình học khớp: gốc tay (`<bên>_pan`) ở (0,0599; ±0,18726; 0,4137 + lift) m; trục vai cao hơn 0,14823 và ra trước 0,05799; L1 0,13491 (trước: (0,12; ±0,17; 0,505), L1 0,140). `lift` 0..0,45 m (trước 0..0,50); lift 0 = lift CAD -160 mm.
3. Nội dung cụm theo P-L: `base` = tấm thép đối trọng 2 lớp, nan, giá bánh, sàn, bánh omni 4 in, servo bánh, khay; `power` = khoang điện tử giữ chỗ M1-BS-038 (trước: vỏ pin, K1, bảng đi dây); `lift` = trụ AlohaMini2 (node base) + khung T, thanh ngang, bánh răng, servo STS3095, chốt hãm (node lift); `shoulder_*` = đế J1 (node lift), khung xoay J2, vai J3; `arm_*` = J4, J5, J56, khoá yaw, servo gập/cuộn; `gripper_*` = kẹp cũ + gá M1-GR-002 + servo kẹp; `camera` = camera ngực (node lift, đi theo xe nâng; trước: camera đầu trên cột, node base) + camera cổ tay trên kẹp.
4. `ma_chinh` mới: base M1-BS-032; power M1-BS-038; lift M1-LF-057 + M1-SV-002; shoulder M1-SH-005 + M1-SH-006; arm M1-AR-001 + M1-SV-001; gripper M1-GR-001; camera M1-CM-001.
5. `poses.rest` lift 0,45 (trước 0,50). `tcp.by_side` = (0,12547; 0; 0) cho cả hai tay (TCP kẹp cũ trên trục cuộn).
6. Bỏ `snapshots/` (ảnh rasterizer bản trước P-L). Ảnh cổng hình P-L: `media/qc/s1-glb/` (Blender, `src/15_render_glb.py`).
7. `lab-scene.json` đổi theo bố cục P-L (S1G): robot đứng cách mép bàn 0,19 m (trước 0,25), lift làm việc 0,45, tủ ngăn kéo cao 0,16 (trước 0,20), núm hộp cách bản lề 0,08 (trước 0,11), hộp ở y -0,58 (trước -0,75), tủ lạnh trên giá 4 chân cao 0,25 m (độ cao kệ giữ nguyên).
8. Camera sim `head` (MJCF) nay là camera ngực trên xe nâng, nghiêng 35 độ (vị trí CAD), không còn ở đỉnh cột.

## Quy trình tái lập và kiểm (máy tính toán, mỗi lệnh nặng `flock /tmp/heavy.lock nice -n 10`)
```
python cad/faibotic/pl_tfix.py; python cad/faibotic/assembly_pl.py --masses; python cad/faibotic/pl_sim_dims.py   # (venv build123d) khung T, khối lượng, kích thước sim
python cad/faibotic/export_sim_meshes.py --pl --labels parts_zero.npz      # mesh sim + nhãn cùng lượt
(cd sdk && python -m faibotic.model --out ../sim); python sim/lab/build_lab.py
python src/10_build_m1_glb.py REPO parts_zero.npz OUT 0.11 24 120 0.3 30 30   # m1.glb, joints.json, cum.json, build_stat.json
python src/11_manifest.py OUT REPO/sim/assets <commit> OUT/build_stat.json
python src/12_validate.py REPO OUT node <thư mục gltf-validator>
python src/20_test_limits.py REPO c.json && node src/20_cmp.mjs c.json OUT/joints.json
blender -b -P src/15_render_glb.py -- OUT media/qc/s1-glb --pose ready
```
Tham số 10_build: tỉ lệ 0,12 cho thành phần > 160 tam giác (tối thiểu 30; trước v0.5b 0,06), pháp tuyến mượt theo góc gấp 30 độ, con lăn omni 24, thân servo 120, cụm power 0,3. Thành phần chạm hộp bao toàn robot giữ biên ≤ 0,8 mm (tăng dần số tam giác đích). Phụ thuộc: numpy, scipy, trimesh, fast-simplification, pygltflib, jsonschema; node 22 + gltf-validator.

## Kết quả kiểm
`12_validate.py`: tất cả đạt. gltf-validator 0 lỗi 0 cảnh báo; brotli 485.940 byte; joints trùng spec và m1.xml; FK GLB = SDK (0 mm), FK viết tay theo hằng số URDF ≤ 1 mm; hộp bao GLB lệch STL sim 0,78 mm. `limits.mjs` = Python ở 3.000 điểm.

## Chưa có / giả định
- Thân servo STS3095 ở pan, vai, khuỷu chưa vẽ (STEP follower không có vị trí servo). Servo trục nâng STS3095 là khối giữ chỗ, chưa lọt hốc khung T (DK-N41).
- Tay phải là gương y tay trái (như MJCF); robot thật lắp hai tay AM-ARM200 giống nhau.
- Khoang điện tử là khối giữ chỗ; khối lượng thân in theo infill giả định.
- Cổ tay J4–J5 chồng khi gập từ ~30 độ (DK-N38), giới hạn khớp vẫn của P-A.
