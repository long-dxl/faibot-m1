"""Ghi manifest.json (file, sha256, kích thước, nguồn commit, ngân sách tam giác, mã chính từng cụm) và chép lab-scene.json + schema từ sim/lab.
Dùng: python 11_manifest.py WEB_ASSETS_DIR SIM_ASSETS_DIR SOURCE_COMMIT [STAT_JSON]
lab-scene.json và schema/lab-scene.schema.json chép nguyên byte từ REPO/sim/lab/ (nguồn sim/lab/build_lab.py; REPO = SIM_ASSETS_DIR/../..)."""
import sys, os, json, hashlib, shutil
d, sim, commit = sys.argv[1:4]; st = json.load(open(sys.argv[4])) if len(sys.argv) > 4 else {}
repo = os.path.dirname(os.path.dirname(os.path.abspath(sim)))
MAX_TRIANGLES = 150000         # v0.5d (24/11): 40.000 -> 150.000 theo yêu cầu web; ngân sách m1.glb cho web (check:m1-assets: budget.max_triangles | max_triangles)
MAX_BYTES_GLB_BROTLI = 1_200_000   # hợp đồng web: m1.glb nén brotli q11 <= 1,2 MB (12_validate, check:m1-assets)
MAX_BYTES_GLB = 4_000_000          # trần kích thước THÔ (v0.5d: 1,5 -> 4 MB; trần thật cho web là brotli 1,2 MB); trước đây trường này ghi 1,2 MB nhưng ý là brotli
H = lambda p: hashlib.sha256(open(p, "rb").read()).hexdigest()
lab_src = os.path.join(repo, "sim", "lab")
if os.path.exists(os.path.join(lab_src, "lab-scene.json")):
    shutil.copyfile(os.path.join(lab_src, "lab-scene.json"), os.path.join(d, "lab-scene.json"))
    shutil.copyfile(os.path.join(lab_src, "lab-scene.schema.json"), os.path.join(d, "schema", "lab-scene.schema.json"))
ROLE = {"m1.glb": "robot GLB (node theo khớp và cụm)", "joints.json": "cây khớp, trục, giới hạn mềm từ spec.py, bảng ghép, tư thế", "cum.json": "cụm -> mã M1-xx -> chi tiết", "limits.mjs": "bản JS của giới hạn ghép (spec.py)", "LICENSE": "Apache-2.0 (AM-ARM, AlohaMini, Menagerie robotstudio_so101)", "NOTICE": "ghi công bản dẫn xuất"}
files = [dict(file=f, role=r, sha256=H(os.path.join(d, f)), bytes=os.path.getsize(os.path.join(d, f))) for f, r in ROLE.items()]
for f in sorted(os.listdir(os.path.join(d, "snapshots"))) if os.path.isdir(os.path.join(d, "snapshots")) else []:
    files.append(dict(file="snapshots/" + f, role="ảnh cổng hình", sha256=H(os.path.join(d, "snapshots", f)), bytes=os.path.getsize(os.path.join(d, "snapshots", f))))
tri = st.get("stat", {}).get("tri_out")
assert tri is None or tri <= MAX_TRIANGLES, (tri, MAX_TRIANGLES)
cum = json.load(open(os.path.join(d, "cum.json"), encoding="utf-8"))["cum"]
ls = os.path.join(d, "lab-scene.json")
if os.path.exists(ls):
    L = json.load(open(ls, encoding="utf-8"))
    lab = dict(file="lab-scene.json", trang_thai="đã xuất (luồng C, sim/lab/build_lab.py)", nguon="sim/lab/lab-scene.json", schema="schema/lab-scene.schema.json",
               sha256=H(ls), bytes=os.path.getsize(ls), schema_sha256=H(os.path.join(d, "schema", "lab-scene.schema.json")), max_triangles=L.get("max_triangles"))
else:
    lab = dict(file="lab-scene.json", trang_thai="chờ luồng C xuất", schema="schema/lab-scene.schema.json", sha256=None)
m = dict(schema="faibot-m1-manifest/1", ngay=os.environ.get("M1_NGAY", "23/11/2026"), nguon=dict(commit=commit, ghi_chu="commit CAD nguồn mà sim/assets được xuất từ đó (export_sim_meshes.py); sim/assets và m1.glb của lượt dựng có thể chưa commit lúc ghi, đối chiếu bằng sha256 bên dưới. m1.glb dựng từ chính sim/assets/*.stl và cây body/khớp của sim/m1.xml, không xuất lại từ STEP",
         sim_assets_meshes_json_sha256=H(os.path.join(sim, "meshes.json")), sim_assets=[dict(file=e["file"], sha256=e["sha256"]) for e in json.load(open(os.path.join(sim, "meshes.json")))["meshes"]]),
         max_triangles=MAX_TRIANGLES, budget=dict(max_triangles=MAX_TRIANGLES, max_bytes_glb=MAX_BYTES_GLB, max_bytes_glb_brotli=MAX_BYTES_GLB_BROTLI, tam_giac_glb=tri),
         ma_chinh={c["ten"]: c.get("ma_chinh", []) for c in cum},
         decimate=dict(ti_le_tam_giac=round(st.get("ratio", 0), 4), tam_giac_goc=st.get("stat", {}).get("tri_in"), tam_giac_glb=tri, tham_so=st.get("ratio_param"), tam_giac_moi_con_lan=st.get("roller_tri"), ghi_chu="decimate quadric (fast-simplification) theo từng thành phần liên thông; thành phần <= 160 tam giác giữ nguyên"),
         files=files, lab_scene=lab)
open(os.path.join(d, "manifest.json"), "w").write(json.dumps(m, ensure_ascii=False, indent=1) + "\n"); print("manifest", len(files), "file")
