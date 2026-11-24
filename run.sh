#!/usr/bin/env bash
# Faibotic M1, một lệnh:
#   ./run.sh              mở cửa sổ mô phỏng + teleop bàn phím
#   ./run.sh --selftest   tự kiểm không cửa sổ (mã thoát 0 = đạt)
#   ./run.sh --repl       gõ lệnh SDK, xem robot chạy
#   ./run.sh --demo       chạy demo gắp đặt và kiểm toạ độ (in JSON)
#   ./run.sh --test       chạy pytest (tests và sdk/tests)
#   ./run.sh --lab-pick   phòng lab: gắp đặt (không cửa sổ, in JSON, mã 0 = hoàn thành; thêm --viewer để xem)
#   ./run.sh --lab-sort   phòng lab: phân loại theo màu vào khay
#   ./run.sh --lab-open   phòng lab: mở/đóng ngăn kéo, nắp hộp, cửa tủ lạnh
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ]; then
  PY=""
  for c in python3.12 python3.13 python3.14 python3; do
    if command -v "$c" >/dev/null 2>&1; then PY="$c"; break; fi
  done
  [ -n "$PY" ] || { echo "Cần Python 3.10 trở lên (macOS: brew install python)"; exit 1; }
  echo "Lần đầu: tạo môi trường .venv bằng $PY và cài thư viện (khoảng 1 phút)..."
  "$PY" -m venv .venv
  .venv/bin/python -m pip install -q --upgrade pip
  .venv/bin/python -m pip install -q -r requirements.txt
fi

PYBIN=.venv/bin/python
case "${1:-}" in
  --selftest) shift; exec "$PYBIN" demos/selftest.py "$@" ;;
  --demo)     shift; exec "$PYBIN" demos/check_demo.py "$@" ;;
  --test)     shift; exec "$PYBIN" -m pytest -q tests sdk/tests "$@" ;;
  --lab-pick|--lab-sort|--lab-open)
    case "$1" in --lab-pick) S=demos/lab_pick_place.py ;; --lab-sort) S=demos/lab_sorting.py ;; *) S=demos/lab_open_close.py ;; esac
    shift
    if [[ " $* " == *" --viewer "* && "$(uname)" = "Darwin" ]]; then exec .venv/bin/mjpython "$S" "$@"; fi
    exec "$PYBIN" "$S" "$@" ;;
  --repl)
    shift
    if [ "$(uname)" = "Darwin" ]; then exec .venv/bin/mjpython -i examples/repl.py "$@"; fi
    exec "$PYBIN" -i examples/repl.py "$@" ;;
  *) exec "$PYBIN" demos/teleop.py "$@" ;;
esac
