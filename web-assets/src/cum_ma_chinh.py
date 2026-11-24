"""Mã chính (ma_chinh) của từng cụm: mã web hiện đầu thẻ cụm (khung/servo chính, không phải đệm, chặn, chốt).
Một nguồn cho 10_build_m1_glb.py (khi dựng lại cum.json) và 11_manifest.py (khoá manifest.ma_chinh).
Dùng riêng (không dựng lại m1.glb): python cum_ma_chinh.py REPO WEB_DIR   ghi ma_chinh vào WEB_DIR/cum.json, kiểm mã có trong
product/parts-register.csv và nằm trong ma của cụm; mã 1 nếu sai. Từ CAD v0.4.4 mọi cụm có lưới (power, camera có ma_chinh)."""
import csv, json, os, sys

MA_CHINH = {   # S1G (P-L)
    "base": ["M1-BS-032"],                    # tấm thép đối trọng chính (moay-ơ nối nan, đế trụ)
    "power": ["M1-BS-038"],                   # khoang điện tử (giữ chỗ)
    "lift": ["M1-LF-057", "M1-SV-002"],       # khung T giữa + servo trục nâng STS3095
    "shoulder_L": ["M1-SH-005", "M1-SH-006"],  # đế J1 + vai J3 AM-ARM200
    "shoulder_R": ["M1-SH-005", "M1-SH-006"],
    "arm_L": ["M1-AR-001", "M1-SV-001"],      # khuỷu J4 + servo gập/cuộn STS3215
    "arm_R": ["M1-AR-001", "M1-SV-001"],
    "gripper_L": ["M1-AR-007"],               # thân kẹp J7 AM-ARM200
    "gripper_R": ["M1-AR-007"],
    "camera": ["M1-CM-001"],                  # camera USB 720p (ngực + cổ tay)
}


def register_codes(repo: str) -> set:
    with open(os.path.join(repo, "product", "parts-register.csv"), encoding="utf-8") as f:
        return {r["ma"] for r in csv.DictReader(f)}


def check(cums: list, reg: set) -> list:
    err = []
    for c in cums:
        mc = MA_CHINH.get(c["ten"])
        if mc is None:
            err.append(f"{c['ten']}: chưa có trong MA_CHINH")
            continue
        err += [f"{c['ten']}: {m} không có trong parts-register.csv" for m in mc if m not in reg]
        err += [f"{c['ten']}: {m} không nằm trong ma của cụm" for m in mc if m not in c["ma"]]
    return err


def with_ma_chinh(c: dict) -> dict:
    """Trả bản sao cụm có ma_chinh đặt ngay sau ma (giữ thứ tự khoá còn lại)."""
    out = {}
    for k, v in c.items():
        if k == "ma_chinh":
            continue
        out[k] = v
        if k == "ma":
            out["ma_chinh"] = list(MA_CHINH[c["ten"]])
    return out


if __name__ == "__main__":
    repo, web = sys.argv[1:3]
    p = os.path.join(web, "cum.json")
    d = json.load(open(p, encoding="utf-8"))
    e = check(d["cum"], register_codes(repo))
    if e:
        print("\n".join(e)); sys.exit(1)
    d["cum"] = [with_ma_chinh(c) for c in d["cum"]]
    json.dump(d, open(p, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("ma_chinh", {c["ten"]: c["ma_chinh"] for c in d["cum"]})
