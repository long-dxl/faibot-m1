# Tools for the generated files

`export_parts.py` makes everything in `hardware/` except the README text and the licence files. The CAD sources stay where they are (`cad/faibotic/`, `cad/upstream/`); this folder only turns them into one STEP file per part.

## What you need

- `export`: Python 3.12 with [build123d](https://github.com/gumyr/build123d). It builds the whole robot, so it takes about half a minute. Run it on a machine that has build123d.
- `tables` and `--check-only`: the Python standard library only. `tables` also needs PyYAML to read the assembly sequence.

## Commands

Run from the repository root.

```
python hardware/tools/export_parts.py export        # STEP files and tools/export-report.json
python hardware/tools/export_parts.py tables        # parts.csv, guide-part-map.json, urdf/, the README table
python hardware/tools/export_parts.py --table       # only the README table
python hardware/tools/export_parts.py --check-only  # read-only checks, exit 1 on a failure
```

`export` does this:

1. Builds the v0.5 mobile configuration with `cad/faibotic/assembly_pl.py` (lift at 0, pose `ready`).
2. Groups the 159 placed bodies into 64 unique parts. Left and right copies, numbered wheels and rollers share one file.
3. Writes the assembly as four STEP files in one shared frame (`base`, `lift`, `station`, `arm`). One combined file is 70 MB, above the 50 MB limit.
4. Writes each Faibotic part and each purchased-part model into `parts/faibotic/` and `parts/purchased/`, with the origin at the minimum corner of its bounding box and the file name as STEP product name.
5. Copies every inherited part byte for byte from `cad/upstream/` into `parts/inherited/` and compares the SHA-256 sums.
6. Re-opens every file it wrote and compares volume (0.1 %) and bounding box (0.1 mm) with the part in the assembly, then writes `export-report.json`.

`--check-only` also checks that the quantities in `parts.csv` match the assembly count, `product/danh-sach-in.csv` and the quantity column of the BOM CSV, that `urdf/` equals `sim/`, that the README table equals `parts.csv`, and that every part in `cad/faibotic/trinh_tu_lap.yaml` appears in `guide-part-map.json`. The repository check `scripts/kiem_cheo.py` runs the same function.

## When part names change

`part-names.en.csv` is the only source of English names, groups and types. Its `key` column is the part name used by `assembly_pl.py` with the side and the copy number removed. `export` stops when the assembly has a key that is missing from the file, or the file has a key the assembly no longer has.

## Git LFS

Not used. The largest file is 37 MB. `git lfs version` is not available on the build machine either. See "Large files" in [../README.md](../README.md).
