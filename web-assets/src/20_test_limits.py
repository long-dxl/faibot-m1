"""Bước 5: sinh >= 2000 điểm (gồm biên vai -80 x khuỷu >= 120, mọi tổ hợp nút bảng, ngoài dải) và đáp án Python (spec/limits) -> cases.json.
Rồi so với limits.mjs bằng 05_cmp.mjs. Dùng: python 05_test_limits.py REPO OUT.json"""
import sys, os, json, math, random, itertools
sys.path.insert(0, os.path.join(sys.argv[1], "sdk")); from faibotic import spec, limits
random.seed(5); C = []
r = math.radians; S = spec.WRIST_COUPLE_SHOULDER_DEG; E = spec.WRIST_COUPLE_ELBOW_DEG
for s, e in itertools.product(S + spec.ELBOW_COUPLE_SHOULDER_DEG, E + (120.0, 125.0, 127.0)):
    for w in (-100, 0, 45, 100, 120): C.append((s, e, w))
for e in (118, 119, 120, 121, 122.5, 125, 127, 130):
    for w in (-100, 50, 100, 120): C.append((-80.0, e, w)); C.append((-83.0, e, w)); C.append((-80.0000001, e, w))
while len(C) < 3000: C.append((random.uniform(-95, 60), random.uniform(-20, 140), random.uniform(-110, 130)))
out = []
for s, e, w in C:
    q = limits.clamp({"left_shoulder": r(s), "left_elbow": r(e), "left_wrist_flex": r(w)}, {"left_pan": 0.0, "left_wrist_roll": 0.0})
    out.append(dict(s=r(s), e=r(e), w=r(w), es=spec.elbow_max(r(s)), wm=spec.wrist_max(r(s), r(e)), clamp=q))
json.dump(out, open(sys.argv[2], "w")); print(len(out), "diem")
