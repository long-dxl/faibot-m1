# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Cửa sổ xem MuJoCo tối giản của Faibotic (GLFW), dùng cho teleop bàn phím.

Khác viewer có sẵn của MuJoCo: biết phím đang được giữ (giữ = chạy, nhả = dừng) và không
dính phím tắt mặc định của MuJoCo (W bật khung dây, S tắt bóng...). Chạy bằng `python`
thường, không cần `mjpython`. Chuột: kéo trái = xoay, kéo phải = dời, cuộn = thu phóng.
"""
from __future__ import annotations

import unicodedata

import glfw
import mujoco
import numpy as np


def ascii_vi(text: str) -> str:
    """Font của MuJoCo chỉ có ASCII: bỏ dấu tiếng Việt cho chữ trên cửa sổ."""
    text = text.replace("đ", "d").replace("Đ", "D").replace("°", " deg")
    return "".join(c for c in unicodedata.normalize("NFD", text) if not unicodedata.combining(c))


class GLViewer:
    def __init__(self, model, data, title: str = "Faibotic M1", width: int = 1280, height: int = 800):
        if not glfw.init():
            raise RuntimeError("không khởi tạo được GLFW (cần màn hình)")
        glfw.window_hint(glfw.SAMPLES, 4)
        self.window = glfw.create_window(width, height, title, None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("không tạo được cửa sổ OpenGL")
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        self.model, self.data = model, data
        self.scene = mujoco.MjvScene(model, maxgeom=10000)
        self.opt = mujoco.MjvOption()
        self.cam = mujoco.MjvCamera()
        self.ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.keys_down: set[int] = set()
        self.events: list[int] = []
        self._btn = {glfw.MOUSE_BUTTON_LEFT: False, glfw.MOUSE_BUTTON_RIGHT: False}
        self._last = (0.0, 0.0)
        self.set_free_camera()
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_mouse_button_callback(self.window, self._on_button)
        glfw.set_cursor_pos_callback(self.window, self._on_move)
        glfw.set_scroll_callback(self.window, self._on_scroll)

    # ---------- camera ----------
    def set_free_camera(self, lookat=(0.8, 0.0, 0.6), distance=2.8, azimuth=60.0, elevation=-25.0):
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.lookat[:] = lookat
        self.cam.distance, self.cam.azimuth, self.cam.elevation = distance, azimuth, elevation

    def set_fixed_camera(self, name: str):
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name)

    def track_body(self, name: str):
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    # ---------- sự kiện ----------
    def _on_key(self, win, key, scancode, action, mods):
        if action == glfw.PRESS:
            self.keys_down.add(key)
            self.events.append(key)
        elif action == glfw.RELEASE:
            self.keys_down.discard(key)

    def _on_button(self, win, button, action, mods):
        if button in self._btn:
            self._btn[button] = action == glfw.PRESS
        self._last = glfw.get_cursor_pos(win)

    def _on_move(self, win, x, y):
        dx, dy = x - self._last[0], y - self._last[1]
        self._last = (x, y)
        if not any(self._btn.values()):
            return
        if self.cam.type == mujoco.mjtCamera.mjCAMERA_FIXED:
            return
        _, h = glfw.get_window_size(win)
        act = (mujoco.mjtMouse.mjMOUSE_ROTATE_V if self._btn[glfw.MOUSE_BUTTON_LEFT]
               else mujoco.mjtMouse.mjMOUSE_MOVE_V)
        mujoco.mjv_moveCamera(self.model, act, dx / h, dy / h, self.scene, self.cam)

    def _on_scroll(self, win, dx, dy):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * dy, self.scene, self.cam)

    def pop_events(self) -> list[int]:
        ev, self.events = self.events, []
        return ev

    # ---------- vẽ ----------
    def render(self, texts: list[tuple[str, str, str]] | None = None) -> None:
        """texts: danh sách (vị trí 'tl'|'tr'|'bl'|'br', cột trái, cột phải)."""
        w, h = glfw.get_framebuffer_size(self.window)
        rect = mujoco.MjrRect(0, 0, w, h)
        mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(rect, self.scene, self.ctx)
        pos = {"tl": mujoco.mjtGridPos.mjGRID_TOPLEFT, "tr": mujoco.mjtGridPos.mjGRID_TOPRIGHT,
               "bl": mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, "br": mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT}
        for where, a, b in texts or []:
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, pos[where], rect, ascii_vi(a), ascii_vi(b), self.ctx)
        self._rect = rect
        glfw.swap_buffers(self.window)
        glfw.poll_events()

    def screenshot(self, path: str) -> None:
        """Lưu khung vừa vẽ (gọi ngay sau render) ra PNG qua ffmpeg."""
        import subprocess
        rect = self._rect
        rgb = np.zeros((rect.height, rect.width, 3), dtype=np.uint8)
        glfw.make_context_current(self.window)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.ctx)
        # vẽ lại vào back buffer rồi đọc
        mujoco.mjr_render(rect, self.scene, self.ctx)
        mujoco.mjr_readPixels(rgb, None, rect, self.ctx)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                        "-s", f"{rect.width}x{rect.height}", "-i", "-", "-vf", "vflip", "-frames:v", "1", path],
                       input=np.ascontiguousarray(rgb).tobytes(), check=True)

    def should_close(self) -> bool:
        return glfw.window_should_close(self.window)

    def close(self) -> None:
        if self.window:
            self.ctx.free()
            glfw.destroy_window(self.window)
            self.window = None
            glfw.terminate()
