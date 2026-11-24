import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))
sys.path.insert(0, str(ROOT / "demos"))
# Máy không có OpenGL offscreen (EGL/OSMesa): đặt M1_SKIP_RENDER=1 để bỏ qua test camera.
