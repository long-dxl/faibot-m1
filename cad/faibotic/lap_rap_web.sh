#!/bin/bash
# Dựng cả bộ ảnh lắp ráp web (web-assets/lap-rap). Cần: mesh (assembly_pl.py --mesh MESH --lift 0 --pose ready), Blender 4.0.x, Pillow, numpy, PyYAML,
# và servo-markers.json (servo_markers.py, chạy ở máy có build123d).
#   lap_rap_web.sh MESH OUT SERVO_MARKERS.json [buoc|cyc|all]
# Ra: OUT/web/*.webp (+ servo-id.json). Chạy nặng: flock /tmp/heavy.lock nice -n 10; chờ khoá quá 5 phút thì nice -n 19.
set -e
MESH=$(realpath "$1"); OUT=$(realpath -m "$2"); MK=$(realpath "$3"); STAGE=${4:-all}
HERE=$(cd "$(dirname "$0")" && pwd); SRC=$HERE/../../media/renders-web/src
PY=${PY:-python3}; BL=${BLENDER:-/usr/bin/blender}; SAMPLES=${SAMPLES:-36}
mkdir -p "$OUT/web" "$OUT/cyc"
run() { set +e; flock -w 300 -E 99 /tmp/heavy.lock nice -n 10 "$@"; rc=$?; set -e; if [ $rc = 99 ]; then nice -n 19 "$@"; else return $rc; fi; }
$PY "$HERE/render_pdf_buoc.py" "$MESH" "$OUT" --chi sel --style web
if [ "$STAGE" = buoc ] || [ "$STAGE" = all ]; then
  run $PY "$HERE/render_pdf_buoc.py" "$MESH" "$OUT/buoc" --style web --chi buoc --blender "$BL"
  cp "$OUT"/buoc/web/* "$OUT/web/"
fi
[ "$STAGE" = buoc ] && exit 0
cyc() {   # TÊN VIEW SEL [tham số render_scene.py...]
  local n="$1" v="$2" s="$3"; shift 3  # public-scan: ok
  run "$BL" -b -P "$SRC/render_scene.py" -- "$MESH" "$v" "$OUT/cyc/$n.png" --samples "$SAMPLES" --alpha --sel "$OUT/sel-$s.json" "$@" > "$OUT/cyc/$n.log" 2>&1
  $PY "$HERE/web_hoan_thien.py" "$OUT/cyc/$n.png" "$n" "$OUT/web" $MKARG
}
FLOOR=0.5047      # tram-u: sàn studio sát đáy trạm (z nhỏ nhất 506,7 mm)
cyc tram-gan tram-gan tram-gan
cyc estop estop estop
cyc tram-u tram-u tram-u --floor-z "$FLOOR"
MKARG="--markers $OUT/cyc/sau-khi-lap.png.markers.json"
cyc sau-khi-lap sau-khi-lap sau --markers "$MK"
MKARG=""
goc() { $PY -c "import yaml;g=yaml.safe_load(open('$HERE/trinh_tu_lap.yaml'))['web_goc']['$1'];print('--el',g[0],'--az',g[1])"; }
cyc cum-de cum de $(goc de)
cyc cum-truc-nang cum truc $(goc truc)
cyc cum-tram cum tram $(goc tram)
cyc cum-dien cum dien $(goc dien)
echo DONE
