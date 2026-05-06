from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np
from fastapi import Body, FastAPI, HTTPException
from fastapi.responses import JSONResponse, Response

from camera_core import XimeaCameraController

app = FastAPI(title="XIMEA Camera Service", version="0.3.0")


class CameraRuntime:
    """
    职责：
    1. 管理 XIMEA 相机生命周期
    2. 后台持续抓取预览帧
    3. 缓存最新预览 JPEG
    4. 提供单帧抓拍能力
    5. 提供本地参数面板事件驱动
    6. 恢复 / 保存相机参数
    """

    def __init__(self) -> None:
        self.ctrl = XimeaCameraController(exposure_us=20000, settings_path="camera_settings.json")
        self.lock = threading.Lock()
        self.preview_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

        self.is_open = False
        self.preview_running = False

        self.latest_frame_bgr: Optional[np.ndarray] = None
        self.latest_preview_jpeg: Optional[bytes] = None
        self.latest_timestamp: Optional[float] = None
        self.last_error: Optional[str] = None

    def open_camera(self) -> None:
        with self.lock:
            if self.is_open:
                return

            self.ctrl.open()
            self.ctrl.apply_saved_or_default_profile()
            self.ctrl.start_rgb_mode(bit_depth=24)

            self.is_open = True
            self.last_error = None

        self.start_preview_thread()

    def close_camera(self) -> None:
        self.stop_event.set()

        if self.preview_thread and self.preview_thread.is_alive():
            self.preview_thread.join(timeout=2.0)

        with self.lock:
            if self.is_open:
                try:
                    self.ctrl.persist_current_state()
                except Exception:
                    pass
                try:
                    self.ctrl.stop_acquisition()
                except Exception:
                    pass
                try:
                    self.ctrl.close()
                except Exception:
                    pass

            self.is_open = False
            self.preview_running = False
            self.preview_thread = None
            self.latest_frame_bgr = None
            self.latest_preview_jpeg = None
            self.latest_timestamp = None

    def start_preview_thread(self) -> None:
        if self.preview_thread and self.preview_thread.is_alive():
            return

        self.stop_event.clear()
        self.preview_thread = threading.Thread(
            target=self._preview_loop,
            name="ximea-preview",
            daemon=True,
        )
        self.preview_thread.start()

    def _preview_loop(self) -> None:
        self.preview_running = True

        while not self.stop_event.is_set():
            try:
                frame = self.ctrl.grab_frame()
                frame_bgr = self._normalize_to_bgr8(frame)

                # 驱动本地参数面板
                self.ctrl.process_panel_events()

                preview = self._resize_for_preview(frame_bgr, max_width=960)

                ok, enc = cv2.imencode(".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if not ok:
                    raise RuntimeError("Failed to encode preview JPEG")

                with self.lock:
                    self.latest_frame_bgr = frame_bgr
                    self.latest_preview_jpeg = enc.tobytes()
                    self.latest_timestamp = time.time()
                    self.last_error = None

                time.sleep(0.03)

            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                time.sleep(0.2)

        self.preview_running = False

    def get_preview_bytes(self) -> bytes:
        with self.lock:
            if self.latest_preview_jpeg is None:
                raise RuntimeError("No preview frame available")
            return self.latest_preview_jpeg

    def capture_still_png(self) -> bytes:
        with self.lock:
            if self.latest_frame_bgr is None:
                raise RuntimeError("No camera frame available")
            frame = self.latest_frame_bgr.copy()

        ok, enc = cv2.imencode(".png", frame)
        if not ok:
            raise RuntimeError("Failed to encode capture PNG")
        return enc.tobytes()

    def show_panel(self) -> None:
        with self.lock:
            if not self.is_open:
                raise RuntimeError("Camera is not open")
            self.ctrl.show_parameter_panel()

    def close_panel(self) -> None:
        with self.lock:
            self.ctrl.close_parameter_panel()

    def get_status(self) -> dict:
        with self.lock:
            base = {
                "is_open": self.is_open,
                "preview_running": self.preview_running,
                "latest_timestamp": self.latest_timestamp,
                "last_error": self.last_error,
            }

        try:
            runtime = self.ctrl.query_runtime_status() if self.is_open else {}
            if self.is_open:
                runtime["persisted_settings"] = self.ctrl.load_persisted_settings()
        except Exception as exc:
            runtime = {"status_error": str(exc)}

        base.update(runtime)
        return base

    @staticmethod
    def _normalize_to_bgr8(frame: np.ndarray) -> np.ndarray:
        if frame is None:
            raise ValueError("Empty frame")

        if frame.ndim == 2:
            if frame.dtype == np.uint16:
                frame8 = cv2.convertScaleAbs(frame, alpha=255.0 / 65535.0)
            else:
                frame8 = frame.astype(np.uint8, copy=False)
            return cv2.cvtColor(frame8, cv2.COLOR_GRAY2BGR)

        if frame.dtype == np.uint16:
            frame8 = cv2.convertScaleAbs(frame[..., :3], alpha=255.0 / 65535.0)
        else:
            frame8 = frame[..., :3].astype(np.uint8, copy=False)

        # XIMEA RGB24 -> OpenCV BGR
        return frame8

    @staticmethod
    def _resize_for_preview(frame_bgr: np.ndarray, max_width: int = 960) -> np.ndarray:
        h, w = frame_bgr.shape[:2]
        if w <= max_width:
            return frame_bgr
        scale = max_width / float(w)
        new_size = (int(w * scale), int(h * scale))
        return cv2.resize(frame_bgr, new_size, interpolation=cv2.INTER_AREA)


runtime = CameraRuntime()


@app.get("/health")
def health():
    return {"ok": True}


@app.post("/camera/open")
def camera_open():
    try:
        runtime.open_camera()
        return {"ok": True, "message": "camera opened"}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/close")
def camera_close():
    try:
        runtime.close_camera()
        return {"ok": True, "message": "camera closed"}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.get("/camera/status")
def camera_status():
    return JSONResponse(runtime.get_status())


@app.get("/camera/frame")
def camera_frame():
    try:
        data = runtime.get_preview_bytes()
        return Response(content=data, media_type="image/jpeg")
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


@app.post("/camera/capture")
def camera_capture():
    try:
        data = runtime.capture_still_png()
        return Response(content=data, media_type="image/png")
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


@app.post("/camera/panel/open")
def camera_panel_open():
    try:
        runtime.show_panel()
        return {"ok": True, "message": "parameter panel opened"}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/panel/close")
def camera_panel_close():
    try:
        runtime.close_panel()
        return {"ok": True, "message": "parameter panel closed"}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/manual_image")
def camera_manual_image(payload: dict = Body(...)):
    """
    手动图像参数：
    - exposure_ms
    - gain_db
    - framerate_hz

    只传想改的项即可。
    成功后自动保存到 camera_settings.json。
    """
    try:
        exposure_ms = payload.get("exposure_ms", None)
        gain_db = payload.get("gain_db", None)
        framerate_hz = payload.get("framerate_hz", None)

        exposure_ms = None if exposure_ms is None else float(exposure_ms)
        gain_db = None if gain_db is None else float(gain_db)
        framerate_hz = None if framerate_hz is None else float(framerate_hz)

        runtime.ctrl.apply_manual_settings(
            exposure_ms=exposure_ms,
            gain_db=gain_db,
            framerate_hz=framerate_hz,
            persist=True,
        )

        return {
            "ok": True,
            "exposure_ms": exposure_ms,
            "gain_db": gain_db,
            "framerate_hz": framerate_hz,
        }
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/awb/enable")
def camera_awb_enable():
    try:
        runtime.ctrl.enable_auto_white_balance()
        return {"ok": True, "auto_wb": True}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/awb/disable")
def camera_awb_disable():
    try:
        runtime.ctrl.disable_auto_white_balance()
        wb = runtime.ctrl.get_white_balance()
        return {"ok": True, "auto_wb": False, **wb}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/awb/freeze")
def camera_awb_freeze():
    try:
        wb = runtime.ctrl.freeze_auto_white_balance()
        return {"ok": True, "auto_wb": False, **wb}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/manual_wb")
def camera_manual_wb(payload: dict = Body(...)):
    """
    手动白平衡。
    调完后预览会立刻变化。
    """
    try:
        wb_r = float(payload.get("wb_r", 2.2))
        wb_g = float(payload.get("wb_g", 1.0))
        wb_b = float(payload.get("wb_b", 1.6))

        runtime.ctrl.disable_auto_white_balance()
        runtime.ctrl.set_manual_white_balance(wb_r, wb_g, wb_b)

        return {"ok": True, "wb_r": wb_r, "wb_g": wb_g, "wb_b": wb_b, "auto_wb": False}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/camera/settings/save")
def camera_settings_save():
    try:
        status = runtime.ctrl.persist_current_state()
        return {"ok": True, "saved": True, "status": status}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))