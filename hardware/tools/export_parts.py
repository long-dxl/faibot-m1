#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Make the generated part of hardware/ from the CAD sources in cad/ and sim/.

  python hardware/tools/export_parts.py export       build123d needed. Builds the v0.5 P-L assembly (lift 0, pose ready), writes
                                                     assembly/*.step, parts/faibotic/*.step, parts/purchased/*.step, copies parts/inherited/
                                                     byte for byte from cad/upstream, re-opens every file and writes tools/export-report.json.
  python hardware/tools/export_parts.py tables       no build123d. Reads export-report.json and writes parts.csv, guide-part-map.json,
                                                     urdf/ (copy of sim/) and the table in README.md.
  python hardware/tools/export_parts.py --table      only the README table.
  python hardware/tools/export_parts.py --check-only read-only checks (no build123d, no PyYAML). Exit 1 on any failure.

Run `export` on a machine with build123d, then `tables` and `--check-only` anywhere.
"""
from __future__ import annotations

import csv
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

HW = Path(__file__).resolve().parents[1]
ROOT = HW.parent
TOOLS = HW / "tools"
REPORT = TOOLS / "export-report.json"
UP_AM = ROOT / "cad/upstream/AM-ARM/am-arm200/stp/follower"
UP_AM2 = ROOT / "cad/upstream/AlohaMini/AlohaMini2/hardware/mobile_base2/stp"
INHERIT_DIR = {"AM-ARM": ("am-arm200", UP_AM), "AlohaMini": ("alohamini2", UP_AM2)}
ORIGIN = {"AM-ARM": "AM-ARM 5b03af1", "AlohaMini": "AlohaMini 59f024b"}
MAX_MB = 50
TABLE_START, TABLE_END = "<!-- parts-table:start -->", "<!-- parts-table:end -->"
GROUPS = ["Base", "Lift", "Station", "Arm", "Gripper", "Wrist camera", "Head camera", "Cameras", "Actuators", "Power", "Electronics"]
CSV_COLS = ["file", "part_number", "name", "qty", "type", "group", "license", "origin", "source", "material", "process", "status", "notes"]
LIC = {"printed": "CC-BY-NC-SA-4.0", "machined": "CC-BY-NC-SA-4.0", "cut": "CC-BY-NC-SA-4.0", "inherited": "Apache-2.0", "purchased": "CC-BY-NC-SA-4.0"}
# a few instances share one part file; the left and right copies and the numbered copies are folded into one key
MERGE = {"M1-LF-041_switch_bot": "M1-LF-041_switch", "M1-LF-041_switch_top": "M1-LF-041_switch",
         "M1-CM-001_head_cam": "M1-CM-001_camera", "M1-CM-001_wrist_cam": "M1-CM-001_camera",
         "M1-SV-001_wheel": "M1-SV-001_sts3215", "M1-SV-001_gripper": "M1-SV-001_sts3215",
         "M1-SV-001_wrist_yaw": "M1-SV-001_sts3215"}   # 6+1: wrist yaw servo as in the original AM-ARM200 (replaces yaw lock M1-AR-025)


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read_csv(p: Path) -> list[dict]:
    with open(p, encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def instance_key(name: str) -> str:
    """Instance name in the assembly -> key of the unique part."""
    if name == "M1-BS-031_spoke0":
        return "M1-BS-031_spoke_front"
    if re.fullmatch(r"M1-BS-031_spoke[12]", name):
        return "M1-BS-031_spoke_side"
    n = re.sub(r"_(L|R)(?=_xe$|$)", "", name)
    n = re.sub(r"omni\d_", "omni_", n)
    n = re.sub(r"_(roller|plate)\d+$", r"_\1", n)
    n = re.sub(r"(wheel_mount_(wall|cap))\d$", r"\1", n)
    n = re.sub(r"(M1-SV-001_(wheel|gripper))\d?$", r"\1", n)
    n = re.sub(r"(M1-BS-039_cw_plate)2$", r"\1", n)
    return MERGE.get(n, n)


CLUSTERS = ("base", "lift", "station", "arm")


def cluster_of(name: str) -> str:
    if name.startswith(("M1-LF-", "M1-SV-002")):
        return "lift"
    if name.startswith(("M1-TR-", "M1-CM-001_head")):
        return "station"
    if name.startswith(("M1-SH-", "M1-AR-", "M1-CM-001_wrist", "M1-SV-001_gripper", "M1-SV-001_wrist_yaw")):
        return "arm"
    return "base"


def names() -> dict[str, dict]:
    return {r["key"]: r for r in read_csv(TOOLS / "part-names.en.csv")}


def upstream_for(key: str):
    """(origin, upstream file) when the key is a byte-for-byte upstream STEP, else None."""
    if "_" not in key:
        return None
    base = key.split("_", 1)[1]
    for origin, (_, d) in INHERIT_DIR.items():
        f = d / (base + ".stp")
        if f.exists():
            return origin, f
    return None


def slug(s: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-")


def stem_for(row: dict) -> str:
    return f"{row['part_number']}-{slug(row['name'])}" if row["part_number"] else slug(row["name"])


# ------------------------------------------------------------------ export (build123d)
def export() -> None:
    sys.path.insert(0, str(ROOT / "cad/faibotic"))
    sys.path.insert(0, str(ROOT / "sdk"))
    from build123d import Compound, Pos, export_step, import_step  # noqa: E402
    import assembly_pl as A  # noqa: E402

    nm = names()
    parts = A.build(0.0, "ready")
    groups: dict[str, list] = {}
    for n, mat, s in parts:
        k = instance_key(n)
        if k not in nm:
            sys.exit(f"missing in part-names.en.csv: {k} (instance {n})")
        groups.setdefault(k, []).append((n, s))
    for k in nm:
        if k not in groups:
            sys.exit(f"part-names.en.csv has a key that is not in the assembly: {k}")

    report = {"source": "cad/faibotic/assembly_pl.py build(lift=0, pose='ready')", "units": "mm", "assembly": {}, "parts": {}}
    for sub in ("faibotic", "purchased", "inherited/am-arm200", "inherited/alohamini2"):
        d = HW / "parts" / sub
        if d.exists():
            for f in d.glob("*"):
                if f.suffix.lower() in (".step", ".stp"):
                    f.unlink()
        d.mkdir(parents=True, exist_ok=True)

    errors = []
    for k, inst in groups.items():
        row = nm[k]
        up = upstream_for(k)
        qty = len(inst)
        rep = inst[0][1]
        entry = {"key": k, "qty": qty, "instances": [n for n, _ in inst], "volume_mm3": round(rep.volume, 3)}
        if (row["type"] == "inherited") != (up is not None):
            errors.append(f"{k}: type {row['type']} but upstream match is {up}")
            continue
        if up:
            origin, src = up
            dst = HW / "parts/inherited" / INHERIT_DIR[origin][0] / src.name
            shutil.copyfile(src, dst)
            entry.update(file=str(dst.relative_to(HW)), source=str(src.relative_to(ROOT)), sha256=sha256(dst), sha256_source=sha256(src), origin=origin)
            if entry["sha256"] != entry["sha256_source"]:
                errors.append(f"{k}: sha256 differs after copy")
        else:
            stem = stem_for(row)
            sub = "purchased" if row["type"] == "purchased" else "faibotic"
            dst = HW / "parts" / sub / f"{stem}.step"
            b = rep.bounding_box()
            moved = Pos(-b.min.X, -b.min.Y, -b.min.Z) * rep      # origin at the minimum corner of the bounding box
            moved.label = stem
            try:
                export_step(moved, str(dst))
            except Exception as ex:
                errors.append(f"{k}: export failed {ex}")
                continue
            back = import_step(str(dst))
            vb = sum(s.volume for s in back.solids()) if hasattr(back, "solids") else back.volume
            bb2 = back.bounding_box()
            size_in = [round(b.max.X - b.min.X, 3), round(b.max.Y - b.min.Y, 3), round(b.max.Z - b.min.Z, 3)]
            size_out = [round(bb2.max.X - bb2.min.X, 3), round(bb2.max.Y - bb2.min.Y, 3), round(bb2.max.Z - bb2.min.Z, 3)]
            rel = abs(vb - rep.volume) / max(rep.volume, 1e-9)
            names_in_file = sorted({str(getattr(x, "label", "")) for x in [back]})
            entry.update(file=str(dst.relative_to(HW)), volume_reopened_mm3=round(vb, 3), volume_rel_err=rel, size_mm=size_in, size_reopened_mm=size_out,
                         product_name=names_in_file[0] if names_in_file else "", stem=stem)
            if rel > 1e-3:
                errors.append(f"{k}: volume differs {rel:.2e} after re-open")
            if any(abs(a - c) > 0.1 for a, c in zip(size_in, size_out)):
                errors.append(f"{k}: bounding box differs after re-open {size_in} vs {size_out}")
        entry["bytes"] = dst.stat().st_size
        if entry["bytes"] > MAX_MB * 1e6:
            errors.append(f"{k}: file larger than {MAX_MB} MB")
        report["parts"][k] = entry
    # assembly, split in four clusters in one shared frame (the single file is above the 50 MB limit); after the parts, because labelling re-parents shapes
    asm_dir = HW / "assembly"
    asm_dir.mkdir(parents=True, exist_ok=True)
    for f in asm_dir.glob("*.step"):
        f.unlink()
    clusters: dict[str, list] = {c: [] for c in CLUSTERS}
    for n, mat, s_ in parts:
        s_.label = n
        clusters[cluster_of(n)].append(s_)
    report["assembly"] = {"instances": len(parts), "files": {}}
    for c, kids in clusters.items():
        cp = asm_dir / f"faibot-m1-v0.5-mobile-{c}.step"
        comp = Compound(children=kids, label=f"faibot-m1-v0.5-mobile-{c}")
        export_step(comp, str(cp))
        bb = comp.bounding_box()
        report["assembly"]["files"][f"assembly/{cp.name}"] = {"bytes": cp.stat().st_size, "instances": len(kids),
                                                            "bbox": [round(v, 1) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
    asm_path = max(asm_dir.glob("*.step"), key=lambda q: q.stat().st_size)
    if asm_path.stat().st_size > MAX_MB * 1e6:
        errors.append("an assembly file is larger than 50 MB")
    report["errors"] = errors
    REPORT.write_text(json.dumps(report, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print("parts", len(report["parts"]), "instances", len(parts), "assembly MB", round(asm_path.stat().st_size / 1e6, 1), "errors", len(errors))
    for e in errors:
        print("ERROR", e)
    sys.exit(1 if errors else 0)


# ------------------------------------------------------------------ tables
STATUS = (("dùng", "used"), ("CAD nháp", "draft CAD"), ("nháp", "draft"))
PROC = (("cắt laser", "laser cut"), ("FDM", "FDM print"), ("cắt", "cut"), ("mua", "purchased"))
MATERIAL = (("PETG", "PETG"), ("thép", "steel"), ("nhôm", "aluminium"))


def tr(v: str, table, default="unconfirmed") -> str:
    v = (v or "").strip()
    for k, e in table:
        if v.startswith(k) or (k in v and k != "mua"):
            return e
    return default


def load_rows() -> list[dict]:
    rep = json.loads(REPORT.read_text(encoding="utf-8"))
    nm = names()
    reg = {r["ma"]: r for r in read_csv(ROOT / "product/parts-register.csv")}
    rows = []
    for k, e in rep["parts"].items():
        n = nm[k]
        r = reg[n["part_number"]]
        typ = n["type"]
        origin = ORIGIN[e["origin"]] if typ == "inherited" else "Faibotic"  # purchased: simplified envelope drawn by Faibotic
        source = e["source"] if typ == "inherited" else "cad/faibotic/assembly_pl.py"
        notes = n["notes"]
        if typ != "inherited":
            notes = (notes + "; " if notes else "") + "origin at the bounding-box corner"
        rows.append(dict(file=e["file"], part_number=n["part_number"], name=n["name"], qty=e["qty"], type=typ, group=n["group"], license=LIC[typ], origin=origin,
                         source=source, material=tr(r["vat_lieu"], MATERIAL), process=tr(r["cong_nghe"], PROC), status=tr(r["trang_thai"], STATUS), notes=notes))
    rows.sort(key=lambda x: (GROUPS.index(x["group"]), x["part_number"], x["file"]))
    return rows


def render_table(rows: list[dict]) -> str:
    out = ["| File | Part number | Name | Qty | Type | Group | License |", "| --- | --- | --- | ---: | --- | --- | --- |"]
    for r in rows:
        out.append(f"| [`{Path(r['file']).name}`]({r['file']}) | {r['part_number']} | {r['name']} | {r['qty']} | {r['type']} | {r['group']} | {r['license']} |")
    return "\n".join(out)


def write_table(rows: list[dict]) -> None:
    p = HW / "README.md"
    if not p.exists():
        return
    s = p.read_text(encoding="utf-8")
    a, b = s.index(TABLE_START) + len(TABLE_START), s.index(TABLE_END)
    p.write_text(s[:a] + "\n" + render_table(rows) + "\n" + s[b:], encoding="utf-8")


def guide_map(rows: list[dict]) -> dict:
    import yaml
    y = yaml.safe_load((ROOT / "cad/faibotic/trinh_tu_lap.yaml").read_text(encoding="utf-8"))
    cum = {"de": "Base", "truc": "Lift", "tram": "Station", "dien": "Electronics"}
    by_pn: dict[str, list[dict]] = {}
    for r in rows:
        by_pn.setdefault(r["part_number"], []).append(r)
    steps = []
    for b in y["buoc"]:
        parts = []
        for c in b.get("chi_tiet", []):
            files = by_pn.get(c["ma"], [])
            if not files:
                parts.append({"part_number": c["ma"], "file": None, "qty": c["sl"]})
            elif len(files) == 1:
                parts.append({"part_number": c["ma"], "file": files[0]["file"], "qty": c["sl"]})
            else:
                for f in files:
                    parts.append({"part_number": c["ma"], "file": f["file"], "qty": f["qty"]})
        steps.append({"id": b["id"], "group": cum.get(b["cum"], b["cum"]), "title_vi": b["ten"], "parts": parts,
                      "fasteners": [{"id": o["id"], "qty": o.get("sl", 1)} for o in b.get("oc", [])]})
    return {"source": "cad/faibotic/trinh_tu_lap.yaml", "note": "part_number without a file: bought part with no model in the CAD", "steps": steps}


def tables() -> None:
    rows = load_rows()
    with open(HW / "parts.csv", "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, CSV_COLS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    (HW / "guide-part-map.json").write_text(json.dumps(guide_map(rows), indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    write_table(rows)
    sync_urdf()
    print("parts.csv", len(rows), "rows; guide-part-map.json; urdf synced")


def urdf_meshes() -> list[str]:
    return sorted(set(re.findall(r'filename="assets/([^"]+)"', (ROOT / "sim/m1.urdf").read_text(encoding="utf-8"))))


def sync_urdf() -> None:
    d = HW / "urdf"
    (d / "assets").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / "sim/m1.urdf", d / "m1.urdf")
    keep = set(urdf_meshes())
    for f in (d / "assets").glob("*"):
        if f.name not in keep:
            f.unlink()
    for m in keep:
        shutil.copyfile(ROOT / "sim/assets" / m, d / "assets" / m)


# ------------------------------------------------------------------ checks
BOM_MAP = {"M1-BS-035": ("A05", "eq", "M1-BS-035_omni_hub"), "M1-CM-001": ("A06", "eq", None), "M1-SV-001": ("A02", "le", None), "M1-SV-002": ("A01", "le", None),
           "M1-LF-004": ("N10", "eq", None), "M1-TR-008": ("T02a", "eq", None), "M1-TR-009": ("T04", "eq", None)}


def check() -> list[tuple[str, bool, str]]:
    res: list[tuple[str, bool, str]] = []

    def ok(name, cond, detail=""):
        res.append((name, bool(cond), detail))

    pc = HW / "parts.csv"
    if not pc.exists():
        return [("parts.csv exists", False, "missing")]
    rows = read_csv(pc)
    reg = {r["ma"] for r in read_csv(ROOT / "product/parts-register.csv")}
    ok("parts.csv columns", list(rows[0].keys()) == CSV_COLS, ",".join(rows[0].keys()))
    missing = [r["file"] for r in rows if r["file"] and not (HW / r["file"]).exists()]
    ok("parts.csv files exist", not missing, str(missing[:3]))
    big = [r["file"] for r in rows if r["file"] and (HW / r["file"]).exists() and (HW / r["file"]).stat().st_size > MAX_MB * 1e6]
    asm = sorted((HW / "assembly").glob("*.step"))
    ok("every STEP below 50 MB", not big and len(asm) == len(CLUSTERS) and all(a.stat().st_size <= MAX_MB * 1e6 for a in asm), f"{big} assembly files {len(asm)}")
    ok("parts.csv part numbers in register", all(r["part_number"] in reg for r in rows), str([r["part_number"] for r in rows if r["part_number"] not in reg]))
    ok("parts.csv no price or vendor columns", not ({"price", "gia", "url", "vendor", "supplier"} & set(rows[0].keys())))  # public-scan: ok
    bad = [r["file"] for r in rows if r["type"] == "inherited" and sha256(HW / r["file"]) != sha256(ROOT / r["source"])]
    ok("inherited files are byte copies of cad/upstream", not bad, str(bad[:3]))
    # quantities
    qsum: dict[str, int] = {}
    for r in rows:
        qsum[r["part_number"]] = qsum.get(r["part_number"], 0) + int(r["qty"])
    ds = {r["ma"]: int(float(r["so_luong_moi_robot"])) for r in read_csv(ROOT / "product/danh-sach-in.csv") if r["so_luong_moi_robot"].strip()}
    diff = {k: (qsum[k], ds[k]) for k in qsum if k in ds and qsum[k] != ds[k]}
    ok("qty matches product/danh-sach-in.csv", not diff, str(diff))
    rep = json.loads(REPORT.read_text(encoding="utf-8")) if REPORT.exists() else None
    ok("export-report.json present, 0 errors", rep is not None and not rep.get("errors"), "")
    if rep:
        qrep: dict[str, int] = {}
        for k, e in rep["parts"].items():
            qrep[e["file"]] = e["qty"]
        d2 = [(r["file"], r["qty"], qrep.get(r["file"])) for r in rows if int(r["qty"]) != qrep.get(r["file"])]
        ok("qty matches the assembly count in export-report.json", not d2 and sum(qrep.values()) == rep["assembly"]["instances"] == sum(v["instances"] for v in rep["assembly"]["files"].values()), str(d2[:3]))
    bom = {r["ma"]: float(r["so_luong_1_robot"].split()[0]) for r in read_csv(ROOT / "bom/bom-alibaba-PL.csv") if r["so_luong_1_robot"].strip()}  # public-scan: ok
    bad_bom = []
    for pn, (bc, mode, key) in BOM_MAP.items():
        if pn not in qsum or bc not in bom:
            continue
        q = qsum[pn] if key is None else next((int(r["qty"]) for r in rows if r["name"] == "Omni wheel hub"), 0)
        if (mode == "eq" and q != bom[bc]) or (mode == "le" and q > bom[bc]):
            bad_bom.append((pn, q, bc, bom[bc]))
    ok("qty consistent with bom/bom-alibaba-PL.csv (quantity columns only)", not bad_bom, str(bad_bom))  # public-scan: ok
    # urdf
    u = HW / "urdf"
    ok("urdf/m1.urdf equals sim/m1.urdf", (u / "m1.urdf").exists() and sha256(u / "m1.urdf") == sha256(ROOT / "sim/m1.urdf"))
    bad_m = [m for m in urdf_meshes() if not (u / "assets" / m).exists() or sha256(u / "assets" / m) != sha256(ROOT / "sim/assets" / m)]
    ok("urdf/assets equal sim/assets", not bad_m, str(bad_m[:3]))
    # README table
    rd = HW / "README.md"
    if rd.exists():
        s = rd.read_text(encoding="utf-8")
        a, b = s.find(TABLE_START), s.find(TABLE_END)
        ok("README table equals parts.csv", a >= 0 and s[a + len(TABLE_START):b].strip() == render_table(rows).strip())
    # licences
    for dst, src in (("LICENSE", "LICENSE-hardware"), ("parts/inherited/LICENSE", "cad/upstream/LICENSE"), ("urdf/LICENSE", "LICENSE")):
        ok(f"hardware/{dst} equals {src}", (HW / dst).exists() and sha256(HW / dst) == sha256(ROOT / src))
    # guide map
    gm = HW / "guide-part-map.json"
    if gm.exists():
        g = json.loads(gm.read_text(encoding="utf-8"))
        have = {p["part_number"] for s in g["steps"] for p in s["parts"]}
        txt = (ROOT / "cad/faibotic/trinh_tu_lap.yaml").read_text(encoding="utf-8")
        want = {m.strip() for m in re.findall(r"\bma:\s*([^,}\n]+)", txt)}
        ok("guide-part-map.json lists every part of trinh_tu_lap.yaml", want <= have, str(sorted(want - have)))
        fm = [p["file"] for s in g["steps"] for p in s["parts"] if p["file"] and not (HW / p["file"]).exists()]
        ok("guide-part-map.json files exist", not fm, str(fm[:3]))
        unused = [r["file"] for r in rows if r["file"] not in {p["file"] for s in g["steps"] for p in s["parts"]}]
        res.append(("guide-part-map.json: files not used by any step (info)", True, f"{len(unused)}"))
    else:
        ok("guide-part-map.json exists", False)
    return res


def main(argv: list[str]) -> int:
    cmd = argv[1] if len(argv) > 1 else "export"
    if cmd == "export":
        export()
    elif cmd == "tables":
        tables()
    elif cmd == "--table":
        write_table(load_rows())
    elif cmd == "--check-only":
        bad = 0
        for name, good, detail in check():
            print(("PASS " if good else "FAIL ") + name + (f"  [{detail}]" if detail and not good else ""))
            bad += 0 if good else 1
        print("hardware checks:", "FAIL" if bad else "ok")
        return 1 if bad else 0
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
