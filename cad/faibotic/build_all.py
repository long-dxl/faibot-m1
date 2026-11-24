# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Dựng cụm, kiểm giao cắt/khe hở, xuất STEP/STL + metrics.json. Chạy: python build_all.py <thư mục xuất>
"""
import json, sys, os, itertools
from build123d import export_step, export_stl, Part, Compound
import shoulder as sh
import power_deck as pdk
from params import *

out = sys.argv[1] if len(sys.argv) > 1 else "../export"
os.makedirs(out, exist_ok=True)


def vol_common(a, b):
    try:
        r = a & b
        return float(r.volume) if r is not None else 0.0
    except Exception:
        return float("nan")


def check(parts: dict, allowed_touch=()):
    rows = []
    for (n1, a), (n2, b) in itertools.combinations(parts.items(), 2):
        v = vol_common(a, b)
        d = a.distance_to(b)
        rows.append(dict(a=n1, b=n2, common_mm3=round(v, 4), min_dist_mm=round(float(d), 3)))
    return rows


def exp(name, shape):
    export_step(shape, f"{out}/{name}.step")
    export_stl(shape, f"{out}/{name}.stl")


metrics = {}
# --- cụm vai
parts = dict(servo_L=sh.servo(-1), servo_R=sh.servo(1), cradle=sh.cradle(), link=sh.link(0))
for n, p in parts.items():
    exp(n, p)
exp("shoulder_assembly", Compound(list(parts.values())))
metrics["shoulder_static"] = check(parts)
# quét link -100..+90 (spec.py ARM_LIMITS shoulder), so với servo + cradle
sweep = []
for ang in (-100, -75, -50, -25, 0, 25, 50, 75, 90):
    L = sh.link(ang)
    for n in ("servo_L", "servo_R", "cradle"):
        sweep.append(dict(angle=ang, vs=n, common_mm3=round(vol_common(L, parts[n]), 4), min_dist_mm=round(float(L.distance_to(parts[n])), 3)))
metrics["shoulder_sweep_link"] = sweep
# --- đế nguồn
deck, meta = pdk.deck()
dm = pdk.dummies(meta)
pp = dict(deck=deck, **dm)
exp("power_deck", deck)
exp("power_assembly", Compound(list(pp.values())))
metrics["power"] = check(pp)
metrics["power_meta"] = {k: (list(v) if isinstance(v, tuple) else v) for k, v in meta.items()}
metrics["volumes_mm3"] = {n: round(float(p.volume), 1) for n, p in {**parts, "deck": deck}.items()}
json.dump(metrics, open(f"{out}/metrics.json", "w"), indent=1)
bad = [r for r in metrics["shoulder_sweep_link"] + metrics["shoulder_static"] + metrics["power"] if not (r["common_mm3"] <= 1e-6)]
print("GIAO CAT >0:", bad)
print(json.dumps(metrics["volumes_mm3"]))
