"""Phòng lab sim/lab: cảnh sinh đúng nguồn, JSON cho web đủ trường, ba demo hoàn thành với seed cố định (không cần render)."""
import json
import subprocess
import sys
from pathlib import Path

import mujoco
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim" / "lab"))

import build_lab as L  # noqa: E402
import lab_common as LC  # noqa: E402


def test_lab_files_match_generator():
    p = subprocess.run([sys.executable, str(ROOT / "sim/lab/build_lab.py"), "--check"], capture_output=True, text=True)
    assert p.returncode == 0, p.stdout + p.stderr


def test_lab_scene_json_fields():
    """Khoá theo hợp đồng check:m1-assets của web + cây hình khối items[]."""
    d = json.loads((ROOT / "sim/lab/lab-scene.json").read_text())
    assert d["schema"].startswith("faibot-m1-lab-scene/") and d["units"]["length"] == "m" and isinstance(d["seed"], int)
    assert d["fridge"]["door"]["joint"]["type"] == "hinge" and d["fridge"]["door"]["joint"]["range"][1] >= 1.047
    assert len(d["cabinet"]["drawers"]) == 2 and all(x["joint"]["type"] == "slide" for x in d["cabinet"]["drawers"])
    assert d["box"]["lid"]["joint"]["type"] == "hinge"
    trays = {t["id"] for t in d["trays"]}
    assert len(trays) == 3 and len({t["color"] for t in d["trays"]}) == 3 and len({t["label"] for t in d["trays"]}) == 3
    objs = d["objects"]
    assert 6 <= len(objs) <= 10 and all(o["tray"] in trays for o in objs)
    assert any(o.get("inside") == "fridge" for o in objs)
    assert d["tasks"]["pick_place"]["k"] == 3 == len(d["tasks"]["pick_place"]["objects"])
    assert {"sorting", "open_close"} <= set(d["tasks"])
    ids = {o["id"] for o in d["items"]}
    assert {"table", "fridge", "drawer_cabinet", "lid_box", "sort_tray", "target_zone"} <= ids
    for o in d["items"]:
        for ln in o["links"]:
            for g in ln["geoms"]:
                assert g["color"].startswith("#") and len(g["color"]) == 7
    assert d["table"]["height"] == 0.75


def test_tray_shapes_lid_start_budget():
    """Khay khác màu và khác hình (S5); MJCF qpos0 của nắp = box.lid.start; cảnh trong ngân sách tam giác."""
    d = json.loads((ROOT / "sim/lab/lab-scene.json").read_text())
    tr = d["trays"]
    assert len({t["shape"] for t in tr}) == len(tr) and len({t["color"] for t in tr}) == len(tr)
    assert {t["shape"] for t in tr} <= set(L.SHAPE_LABEL)
    lid = d["box"]["lid"]
    assert lid["start"] == lid["joint"]["start"] and lid["joint"]["range"][0] <= lid["start"] <= lid["joint"]["range"][1]
    assert "hinge" in lid and "knob" in lid and "open_min" in lid["joint"] and "hinge" in d["fridge"]["door"]
    m = mujoco.MjModel.from_xml_path(str(ROOT / "sim/lab/lab.xml"))
    assert abs(m.qpos0[m.joint("lid_hinge").qposadr[0]] - lid["start"]) < 1e-9
    assert d["budget"]["triangles_est"] <= d["max_triangles"]


def test_web_assets_copy_matches():
    """web-assets/ (bàn giao cho web) giữ đúng bản sinh ở sim/lab và hash trong manifest.json."""
    import hashlib
    w = ROOT / "web-assets"
    if not (w / "manifest.json").exists():
        pytest.skip("chưa có web-assets/")
    assert (w / "lab-scene.json").read_bytes() == (ROOT / "sim/lab/lab-scene.json").read_bytes()
    assert (w / "schema/lab-scene.schema.json").read_bytes() == (ROOT / "sim/lab/lab-scene.schema.json").read_bytes()
    m = json.loads((w / "manifest.json").read_text())["lab_scene"]
    assert m["sha256"] == hashlib.sha256((w / "lab-scene.json").read_bytes()).hexdigest()


def test_objects_fit_gripper():
    """Mọi vật có ít nhất một chiều ngang <= 60 mm (khe kẹp tối đa 90 mm, chừa biên)."""
    for _, shape, size, _ in L.OBJECTS:
        w = min(size[0], size[1]) if shape == "box" else 2 * size[0]
        assert w <= 0.06


def test_lab_model_loads_and_is_stable():
    m = mujoco.MjModel.from_xml_path(str(ROOT / "sim/lab/lab.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    p0 = {o[0]: d.body(o[0]).xpos.copy() for o in L.OBJECTS}
    for _ in range(1000):
        mujoco.mj_step(m, d)
    for k, p in p0.items():
        assert abs(d.body(k).xpos - p).max() < 0.003, k
    for j in ("lid_hinge", "drawer_0_slide", "drawer_1_slide", "fridge_hinge"):
        assert abs(d.joint(j).qpos[0]) < 0.01, j


def test_lab_pick_place_completes():
    import lab_pick_place as D
    r = LC.make_robot()
    LC.scatter(r, 0)
    res = D.run(r, log=lambda *a: None)
    r.close()
    assert res["pass"] and res["k"] == 3 and res["done"] == 3, res


def test_lab_sorting_completes():
    import lab_sorting as D
    r = LC.make_robot()
    LC.scatter(r, 1)
    res = D.run(r, log=lambda *a: None)
    r.close()
    assert res["pass"] and res["done"] == len(L.OBJECTS), res


def test_lab_open_close_completes():
    import lab_open_close as D
    r = LC.make_robot()
    res = D.run(r, log=lambda *a: None)
    r.close()
    assert res["pass"], res
    for k in ("drawer", "lid", "fridge"):
        assert res["detail"][k]["open_ok"] and res["detail"][k]["closed_ok"], (k, res["detail"][k])


def test_selfplay_pick_place_seeds():
    """S1 "test tự chơi" (bản rút gọn trong bộ test: 3 seed; chạy đủ 20 seed: demos/selfplay_pick_place.py --seeds 20, M1_SELFPLAY_SEEDS=20).
    Mỗi seed: gắp đặt 2 vật trên bàn, nhấc 5 cm giữ 2 s trượt <= 2 mm, bước khớp <= 2 độ/khung, không NaN."""
    import os
    import selfplay_pick_place as S
    n = int(os.environ.get("M1_SELFPLAY_SEEDS", "3"))
    res = S.run(range(1, 1 + n))
    assert res["pass_count"] >= (18 * n + 19) // 20 and res["slip_max_mm"] <= 2.0 and res["max_joint_step_deg"] <= 2.0, res
