"""S1G: hình va chạm và khối lượng thân trong sim (spec.py) khớp CAD P-L (cad/export/M1-PL-sim-dims.json, sinh bởi cad/faibotic/pl_sim_dims.py),
và gốc tay SDK trùng khớp URDF AlohaMini2 mà CAD dùng. Một nguồn hình học: CAD -> spec -> MJCF -> GLB."""
import json
from pathlib import Path

import numpy as np
import pytest

from faibotic import spec

ROOT = Path(__file__).resolve().parents[1]
P = ROOT / "cad" / "export" / "M1-PL-sim-dims.json"
pytestmark = pytest.mark.skipif(not P.exists(), reason="chưa sinh M1-PL-sim-dims.json")
D = json.loads(P.read_text()) if P.exists() else {}
MM = 5e-4


@pytest.mark.parametrize("key", ["BASE_RADIUS", "SPOKE_R", "SPOKE_Z", "COUNTERWEIGHT_HALF", "COUNTERWEIGHT_POS", "TRAY_HALF", "TRAY_POS",
                                 "COLUMN_XR", "COLUMN_YH", "COLUMN_TOP", "COLUMN_FOOT_XR", "COLUMN_FOOT_YH", "COLUMN_FOOT_Z",
                                 "TMID_XR", "TMID_YH", "TMID_Z", "STATION_RAIL_XR", "STATION_RAIL_YH", "STATION_RAIL_Z",
                                 "STATION_EAR_XR", "STATION_EAR_Y", "STATION_EAR_Z", "HEAD_MAST_XR", "HEAD_MAST_YH", "HEAD_MAST_Z",
                                 "HEAD_GIMBAL_XR", "HEAD_GIMBAL_YH", "HEAD_GIMBAL_Z", "DOCK_XR", "DOCK_YH", "DOCK_Z", "SHOULDER_Y",
                                 "H_J1_ABOVE_TABLE", "HEAD_CAM_TILT_DEG", "HEAD_CAM_AXIS", "HEAD_CAM_POS", "HEAD_CAM_XYAXES",
                                 "STATION_COM", "WHEEL_RADIUS"])
def test_dims_match_cad(key):
    assert np.allclose(np.ravel(getattr(spec, key)), np.ravel(D[key]), atol=MM), (key, getattr(spec, key), D[key])


def test_base_z_and_lift_zero():
    assert (spec.BASE_Z0, spec.BASE_Z1) == pytest.approx(tuple(D["BASE_Z"]), abs=MM)
    assert spec.ARM_BASE_Z0 == pytest.approx(D["goc_xe_nang_z_m"], abs=MM)
    assert spec.LIFT_CAD_OFFSET * 1000 == pytest.approx(D["lift_cad_mm_o_lift_sdk_0"])


@pytest.mark.parametrize("key", ["COUNTERWEIGHT_MASS", "SPOKE_MASS", "COLUMN_MASS", "TRAY_MASS", "BASE_MASS", "CARRIAGE_MASS", "STATION_MASS"])
def test_masses_match_cad(key):
    assert getattr(spec, key) == pytest.approx(D["kg"][key], abs=0.01)


def test_counterweight_is_q8_default():
    """DK-N40, Q8 (a): đối trọng thiết kế ~8 kg (tấm chính 8,5 mm + tấm phụ 6 mm)."""
    assert 7.5 <= spec.COUNTERWEIGHT_MASS <= 8.5
