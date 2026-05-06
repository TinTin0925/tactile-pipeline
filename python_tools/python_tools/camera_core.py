from __future__ import annotations

from typing import Callable, Dict, Optional, Set
import json
import time
from pathlib import Path

import cv2
import numpy as np

import sys
sys.path.insert(0, r"C:\XIMEA\API\Python\v3")

from ximea import xiapi


XI_ACQ_TIMING_MODE_FRAME_RATE = "XI_ACQ_TIMING_MODE_FRAME_RATE"
XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT = "XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT"

DEFAULT_LIMITS = {
    "exposure": {"min": 1.0, "max": 1_000_000.0},   # us
    "gain": {"min": 0.0, "max": 72.0},              # dB
    "framerate": {"min": 1.0, "max": 50.0},         # Hz
}

DEFAULT_SETTINGS = {
    "exposure_ms": 20.0,
    "gain_db": 0.0,
    "framerate_hz": 10.0,
    "auto_wb": True,
    "wb_r": 2.2,
    "wb_g": 1.0,
    "wb_b": 1.6,
}


class XimeaCameraController:
    RGB_FORMATS = {
        64: "XI_RGB64",
        48: "XI_RGB48",
        32: "XI_RGB32",
        24: "XI_RGB24",
    }

    RAW_FORMATS = {
        8: "XI_RAW8",
        16: "XI_RAW16",
    }

    def __init__(self, exposure_us: int = 20000, settings_path: str = "camera_settings.json") -> None:
        self._cam = xiapi.Camera()
        self._image = xiapi.Image()
        self._opened = False
        self._acquiring = False
        self._current_mode: Optional[str] = None

        self._default_exposure = exposure_us
        self._settings_path = Path(settings_path)

        self._param_panel: Optional["ParameterPanel"] = None
        self._limits_cache: Optional[Dict[str, Dict[str, Optional[float]]]] = None

        self._frame_rate_mode_supported = True
        self._frame_rate_mode_warning_printed = False
        self._requested_framerate: Optional[float] = 10.0

        self._measured_fps: Optional[float] = None
        self._last_frame_timestamp: Optional[float] = None

    # -------------------------
    # 基础单位转换
    # -------------------------
    @staticmethod
    def us_to_ms(value: Optional[float]) -> Optional[float]:
        return None if value is None else value / 1000.0

    @staticmethod
    def ms_to_us(value: Optional[float]) -> Optional[float]:
        return None if value is None else value * 1000.0

    # -------------------------
    # 配置持久化
    # -------------------------
    def load_persisted_settings(self) -> Dict[str, float | bool]:
        if self._settings_path.exists():
            try:
                data = json.loads(self._settings_path.read_text(encoding="utf-8"))
                merged = dict(DEFAULT_SETTINGS)
                merged.update(data)
                return merged
            except Exception:
                return dict(DEFAULT_SETTINGS)
        return dict(DEFAULT_SETTINGS)

    def save_persisted_settings(self, settings: Dict[str, float | bool]) -> None:
        current = self.load_persisted_settings()
        current.update(settings)
        self._settings_path.write_text(
            json.dumps(current, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def persist_current_state(self) -> Dict[str, Optional[float]]:
        status = self.query_runtime_status()

        settings = {
            "exposure_ms": status.get("exposure_ms") if status.get("exposure_ms") is not None else DEFAULT_SETTINGS["exposure_ms"],
            "gain_db": status.get("gain_db") if status.get("gain_db") is not None else DEFAULT_SETTINGS["gain_db"],
            "framerate_hz": status.get("framerate_hz") if status.get("framerate_hz") is not None else DEFAULT_SETTINGS["framerate_hz"],
            "auto_wb": bool(status.get("auto_wb")),
            "wb_r": status.get("wb_r") if status.get("wb_r") is not None else DEFAULT_SETTINGS["wb_r"],
            "wb_g": status.get("wb_g") if status.get("wb_g") is not None else DEFAULT_SETTINGS["wb_g"],
            "wb_b": status.get("wb_b") if status.get("wb_b") is not None else DEFAULT_SETTINGS["wb_b"],
        }
        self.save_persisted_settings(settings)
        return status

    # -------------------------
    # 生命周期
    # -------------------------
    def open(self) -> None:
        if self._opened:
            return
        self._cam.open_device()
        self._cam.set_exposure(self._default_exposure)
        self._ensure_frame_rate_mode()
        self._apply_requested_fps()
        self._opened = True

    def close(self) -> None:
        if not self._opened:
            return
        if self._acquiring:
            self.stop_acquisition()
        if self._param_panel:
            self._param_panel.close()
            self._param_panel = None
        self._cam.close_device()
        self._opened = False

    # -------------------------
    # 白平衡控制
    # -------------------------
    def enable_auto_white_balance(self) -> None:
        self.open()
        self._cam.enable_auto_wb()
        self.save_persisted_settings({"auto_wb": True})

    def disable_auto_white_balance(self) -> None:
        if not self._opened:
            return
        self._cam.disable_auto_wb()
        self.save_persisted_settings({"auto_wb": False})

    def set_manual_white_balance(self, wb_r: float, wb_g: float = 1.0, wb_b: float = 1.0) -> None:
        self.open()
        self._cam.set_wb_kr(float(wb_r))
        self._cam.set_wb_kg(float(wb_g))
        self._cam.set_wb_kb(float(wb_b))
        self.save_persisted_settings({
            "auto_wb": False,
            "wb_r": float(wb_r),
            "wb_g": float(wb_g),
            "wb_b": float(wb_b),
        })

    def get_white_balance(self) -> Dict[str, Optional[float]]:
        self.open()

        def safe(fn):
            try:
                return float(fn())
            except Exception:
                return None

        return {
            "wb_r": safe(self._cam.get_wb_kr),
            "wb_g": safe(self._cam.get_wb_kg),
            "wb_b": safe(self._cam.get_wb_kb),
        }

    def freeze_auto_white_balance(self) -> Dict[str, Optional[float]]:
        """
        读取当前自动白平衡结果，转成固定手动白平衡并保存。
        适合在曝光/帧率调好、画面颜色正常后调用。
        """
        self.open()

        wb = self.get_white_balance()
        wb_r = wb.get("wb_r")
        wb_g = wb.get("wb_g")
        wb_b = wb.get("wb_b")

        if wb_r is None or wb_g is None or wb_b is None:
            raise RuntimeError("无法读取当前自动白平衡系数")

        self.disable_auto_white_balance()
        self.set_manual_white_balance(wb_r=wb_r, wb_g=wb_g, wb_b=wb_b)
        return {
            "wb_r": wb_r,
            "wb_g": wb_g,
            "wb_b": wb_b,
        }

    # -------------------------
    # 曝光 / 增益 / 帧率
    # -------------------------
    def enable_auto_exposure(
        self,
        target_level: Optional[int] = None,
        skip_frames: Optional[int] = None,
        show_gui: bool = True,
    ) -> None:
        self.open()
        self._cam.enable_aeag()
        if target_level is not None:
            self._cam.set_aeag_level(target_level)
        if skip_frames is not None:
            self._cam.set_aeag_skip_frames_count(skip_frames)
        if show_gui:
            self._launch_parameter_panel()

    def disable_auto_exposure(self) -> None:
        if not self._opened:
            return
        self._cam.disable_aeag()

    def apply_manual_settings(
        self,
        exposure_ms: Optional[float] = None,
        gain_db: Optional[float] = None,
        framerate_hz: Optional[float] = None,
        persist: bool = True,
    ) -> None:
        self.open()
        limits = self.query_parameter_limits()

        if exposure_ms is not None or gain_db is not None:
            self.disable_auto_exposure()

        if exposure_ms is not None:
            exposure_us = max(1.0, float(self.ms_to_us(exposure_ms)))
            self._write_param_within_limits(
                self._cam.set_exposure,
                exposure_us,
                limits.get("exposure"),
                "曝光(ms)",
                display_converter=self.us_to_ms,
            )

        if gain_db is not None:
            self._write_param_within_limits(
                self._cam.set_gain,
                float(gain_db),
                limits.get("gain"),
                "增益(dB)",
            )

        if framerate_hz is not None:
            self._requested_framerate = float(framerate_hz)
            self._ensure_frame_rate_mode()
            self._write_param_within_limits(
                self._cam.set_framerate,
                float(framerate_hz),
                limits.get("framerate"),
                "帧率(Hz)",
            )
            self._verify_framerate_applied(float(framerate_hz))

        if persist:
            status = self.query_runtime_status()
            saved = {
                "exposure_ms": status.get("exposure_ms") if status.get("exposure_ms") is not None else DEFAULT_SETTINGS["exposure_ms"],
                "gain_db": status.get("gain_db") if status.get("gain_db") is not None else DEFAULT_SETTINGS["gain_db"],
                "framerate_hz": status.get("framerate_hz") if status.get("framerate_hz") is not None else DEFAULT_SETTINGS["framerate_hz"],
            }
            self.save_persisted_settings(saved)

    def apply_saved_or_default_profile(self) -> Dict[str, float | bool]:
        """
        启动相机时调用：
        - 固定曝光 / 增益 / 帧率
        - 白平衡按保存状态恢复（auto 或 manual）
        """
        self.open()
        s = self.load_persisted_settings()

        self.apply_manual_settings(
            exposure_ms=float(s["exposure_ms"]),
            gain_db=float(s["gain_db"]),
            framerate_hz=float(s["framerate_hz"]),
            persist=False,
        )

        if bool(s.get("auto_wb", True)):
            self.enable_auto_white_balance()
        else:
            self.disable_auto_white_balance()
            self.set_manual_white_balance(
                wb_r=float(s["wb_r"]),
                wb_g=float(s["wb_g"]),
                wb_b=float(s["wb_b"]),
            )

        return s

    def _write_param_within_limits(
        self,
        setter: Callable[[float], None],
        value: float,
        limits: Optional[Dict[str, Optional[float]]],
        label: str,
        display_converter: Optional[Callable[[float], Optional[float]]] = None,
    ) -> None:
        if limits:
            min_v = limits.get("min")
            max_v = limits.get("max")
            if min_v is not None and value < min_v:
                display = display_converter(min_v) if display_converter else min_v
                if display is None:
                    display = min_v
                raise ValueError(f"{label} 不能小于 {display:.2f}")
            if max_v is not None and value > max_v:
                display = display_converter(max_v) if display_converter else max_v
                if display is None:
                    display = max_v
                raise ValueError(f"{label} 不能大于 {display:.2f}")
        setter(value)

    # -------------------------
    # 状态 / 限制
    # -------------------------
    def query_runtime_status(self) -> Dict[str, Optional[float]]:
        self.open()

        def safe_call(fn: Callable[[], float]) -> Optional[float]:
            try:
                return float(fn())
            except Exception:
                return None

        framerate = safe_call(self._cam.get_framerate)
        if framerate is not None and framerate > 10_000:
            framerate = None

        return {
            "exposure_ms": self.us_to_ms(safe_call(self._cam.get_exposure)),
            "gain_db": safe_call(self._cam.get_gain),
            "framerate_hz": framerate,
            "measured_fps": self.get_measured_fps(),
            "aeag": bool(self._cam.is_aeag()),
            "auto_wb": bool(self._cam.is_auto_wb()),
            "wb_r": safe_call(self._cam.get_wb_kr),
            "wb_g": safe_call(self._cam.get_wb_kg),
            "wb_b": safe_call(self._cam.get_wb_kb),
        }

    def query_parameter_limits(self, force_refresh: bool = False) -> Dict[str, Dict[str, Optional[float]]]:
        if self._limits_cache is not None and not force_refresh:
            return self._limits_cache

        self.open()

        def safe(fn: Callable[[], float]) -> Optional[float]:
            try:
                return float(fn())
            except Exception:
                return None

        limits = {
            "exposure": {
                "min": safe(self._cam.get_exposure_minimum),
                "max": safe(self._cam.get_exposure_maximum),
            },
            "gain": {
                "min": safe(self._cam.get_gain_minimum),
                "max": safe(self._cam.get_gain_maximum),
            },
            "framerate": {
                "min": safe(self._cam.get_framerate_minimum),
                "max": safe(self._cam.get_framerate_maximum),
            },
        }

        for key, fallback in DEFAULT_LIMITS.items():
            entry = limits.get(key)
            if entry is None:
                limits[key] = fallback.copy()
                continue
            if entry.get("min") is None:
                entry["min"] = fallback["min"]
            else:
                entry["min"] = max(entry["min"], fallback["min"])
            if entry.get("max") is None:
                entry["max"] = fallback["max"]
            else:
                entry["max"] = min(entry["max"], fallback["max"])

        self._limits_cache = limits
        return limits

    # -------------------------
    # 采集
    # -------------------------
    def start_rgb_mode(self, bit_depth: int = 24) -> None:
        fmt = self.RGB_FORMATS.get(bit_depth)
        if fmt is None:
            raise ValueError(f"不支持的 RGB 位深：{bit_depth}")
        self._start_acquisition(fmt, mode="RGB")

    def preview_rgb(self, bit_depth: int = 24, window_name: str = "XIMEA RGB Preview") -> None:
        self._preview(lambda: self.start_rgb_mode(bit_depth), window_name)

    def start_raw_mode(self, bit_depth: int = 8) -> None:
        fmt = self.RAW_FORMATS.get(bit_depth)
        if fmt is None:
            raise ValueError(f"不支持的 RAW 位深：{bit_depth}")
        self._start_acquisition(fmt, mode="RAW")

    def preview_raw(self, bit_depth: int = 8, window_name: str = "XIMEA RAW Preview") -> None:
        self._preview(lambda: self.start_raw_mode(bit_depth), window_name, allow_gui=False)

    def _start_acquisition(self, img_format: str, mode: str) -> None:
        self.open()
        if self._acquiring:
            self.stop_acquisition()
        self._cam.set_imgdataformat(img_format)
        self._cam.start_acquisition()
        self._acquiring = True
        self._current_mode = mode

    def stop_acquisition(self) -> None:
        if not self._acquiring:
            return
        self._cam.stop_acquisition()
        self._acquiring = False
        self._current_mode = None
        self._last_frame_timestamp = None
        self._measured_fps = None

    def grab_frame(self):
        if not self._acquiring:
            raise RuntimeError("请先启动 RGB 或 RAW 模式")
        self._cam.get_image(self._image)
        frame = self._image.get_image_data_numpy()
        self._update_measured_fps()
        return frame

    def _preview(self, starter: Callable[[], None], window_name: str, allow_gui: bool = True) -> None:
        starter()
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        print(f"🎥 {window_name} started. Press 'q' to quit.")
        try:
            while True:
                frame = self.grab_frame()
                cv2.imshow(window_name, self._prepare_for_display(frame))
                if allow_gui and self._param_panel:
                    self._param_panel.process_events()
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
        finally:
            self.stop_acquisition()
            cv2.destroyWindow(window_name)
            print("✅ Preview stopped.")

    @staticmethod
    def _prepare_for_display(frame):
        if frame.dtype == np.uint16:
            return cv2.convertScaleAbs(frame, alpha=255.0 / 65535.0)
        return frame

    def _update_measured_fps(self) -> None:
        now = time.perf_counter()
        if self._last_frame_timestamp is None:
            self._last_frame_timestamp = now
            return
        dt = now - self._last_frame_timestamp
        self._last_frame_timestamp = now
        if dt <= 0:
            return
        inst = 1.0 / dt
        alpha = 0.2
        if self._measured_fps is None:
            self._measured_fps = inst
        else:
            self._measured_fps = alpha * inst + (1 - alpha) * self._measured_fps

    def get_measured_fps(self) -> Optional[float]:
        return self._measured_fps

    # -------------------------
    # 帧率模式
    # -------------------------
    def _ensure_frame_rate_mode(self) -> None:
        if not self._frame_rate_mode_supported:
            return
        desired_modes = (
            XI_ACQ_TIMING_MODE_FRAME_RATE,
            XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT,
        )
        try:
            current_mode = self._cam.get_acq_timing_mode()
        except Exception:
            current_mode = None
        if current_mode in desired_modes:
            return
        last_error: Optional[Exception] = None
        for mode in desired_modes:
            try:
                self._cam.set_acq_timing_mode(mode)
                return
            except Exception as exc:
                last_error = exc
        self._frame_rate_mode_supported = False
        if last_error and not self._frame_rate_mode_warning_printed:
            print(f"[XimeaCameraController] 无法切换到帧率控制模式：{last_error}")
            self._frame_rate_mode_warning_printed = True

    def _apply_requested_fps(self) -> None:
        if self._requested_framerate is None or not self._frame_rate_mode_supported:
            return
        try:
            current = float(self._cam.get_framerate())
        except Exception:
            current = None
        try:
            if current is None or abs(current - self._requested_framerate) > 0.05 * self._requested_framerate:
                self._cam.set_framerate(self._requested_framerate)
        except Exception as exc:
            print(f"[XimeaCameraController] 无法应用默认帧率 {self._requested_framerate:.2f} Hz：{exc}")

    def _verify_framerate_applied(self, requested: float) -> None:
        try:
            current = float(self._cam.get_framerate())
        except Exception:
            return
        threshold = max(1.0, 0.05 * requested)
        if abs(current - requested) <= threshold:
            return
        modes_to_try = (
            XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT,
            XI_ACQ_TIMING_MODE_FRAME_RATE,
        ) if self._frame_rate_mode_supported else ()
        for mode in modes_to_try:
            try:
                self._cam.set_acq_timing_mode(mode)
                self._cam.set_framerate(requested)
                current = float(self._cam.get_framerate())
                if abs(current - requested) <= threshold:
                    return
            except Exception:
                continue
        print(
            f"[XimeaCameraController] 期望帧率 {requested:.2f} Hz，但硬件当前返回 {current:.2f} Hz，"
            "请确认相机是否支持该帧率或降低曝光再试。"
        )

    # -------------------------
    # 参数面板
    # -------------------------
    def show_parameter_panel(self) -> None:
        self.open()
        self._launch_parameter_panel()

    def close_parameter_panel(self) -> None:
        if self._param_panel:
            self._param_panel.close()
            self._param_panel = None

    def process_panel_events(self) -> None:
        if self._param_panel and not self._param_panel.closed:
            self._param_panel.process_events()

    def _launch_parameter_panel(self) -> None:
        if self._param_panel and not self._param_panel.closed:
            self._param_panel.lift()
            return
        limits = self.query_parameter_limits()
        self._param_panel = ParameterPanel(self, limits=limits)

    def _notify_panel_closed(self, panel: "ParameterPanel") -> None:
        if self._param_panel is panel:
            self._param_panel = None


class ParameterPanel:
    STATUS_REFRESH_INTERVAL = 0.25

    def __init__(
        self,
        controller: XimeaCameraController,
        limits: Optional[Dict[str, Dict[str, Optional[float]]]] = None,
    ) -> None:
        import tkinter as tk
        from tkinter import messagebox, ttk

        self._tk = tk
        self._ttk = ttk
        self._messagebox = messagebox
        self._controller = controller
        self._limits = limits or controller.query_parameter_limits()
        self._closed = False
        self._last_status_ts = 0.0

        self._root = tk.Tk()
        self._root.title("XIMEA 参数控制台")
        self._root.geometry("620x520")
        self._root.resizable(False, False)
        self._root.attributes("-topmost", True)
        self._root.protocol("WM_DELETE_WINDOW", self.close)

        self._status_vars = {
            "exposure": tk.StringVar(value="--"),
            "gain": tk.StringVar(value="--"),
            "framerate": tk.StringVar(value="--"),
            "measured_fps": tk.StringVar(value="--"),
            "aeag": tk.StringVar(value="--"),
            "auto_wb": tk.StringVar(value="--"),
            "wb_r": tk.StringVar(value="--"),
            "wb_g": tk.StringVar(value="--"),
            "wb_b": tk.StringVar(value="--"),
        }

        self._manual_entries = {
            "exposure": tk.StringVar(),
            "gain": tk.StringVar(),
            "framerate": tk.StringVar(),
            "wb_r": tk.StringVar(),
            "wb_g": tk.StringVar(),
            "wb_b": tk.StringVar(),
        }

        self._entry_widgets: Dict[str, "tk.Entry"] = {}
        self._entry_editing_keys: Set[str] = set()
        self._manual_dirty_keys: Set[str] = set()

        self._slider_vars = {
            "exposure": tk.DoubleVar(),
            "gain": tk.DoubleVar(),
            "framerate": tk.DoubleVar(),
        }
        self._slider_widgets: Dict[str, "ttk.Scale"] = {}
        self._active_slider_keys: Set[str] = set()
        self._suppress_slider_callback = False

        self._build_ui()
        self._refresh_status(force=True)
        self._root.update_idletasks()

    @property
    def closed(self) -> bool:
        return self._closed

    def lift(self) -> None:
        if not self._closed:
            self._root.deiconify()
            self._root.lift()

    def process_events(self) -> None:
        if self._closed:
            return
        now = time.time()
        if now - self._last_status_ts >= self.STATUS_REFRESH_INTERVAL:
            self._refresh_status()
            self._last_status_ts = now
        try:
            self._root.update_idletasks()
            self._root.update()
        except self._tk.TclError:
            self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._root.destroy()
        except self._tk.TclError:
            pass
        self._controller._notify_panel_closed(self)

    def _build_ui(self) -> None:
        frm = self._ttk.Frame(self._root, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        self._root.columnconfigure(0, weight=1)
        self._root.rowconfigure(0, weight=1)

        self._ttk.Label(frm, text="当前运行状态", font=("Microsoft YaHei", 10, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w"
        )

        labels = [
            ("实时曝光 (ms)", "exposure"),
            ("实时增益 (dB)", "gain"),
            ("目标帧率 (Hz)", "framerate"),
            ("实际帧率 (Hz)", "measured_fps"),
            ("自动曝光", "aeag"),
            ("自动白平衡", "auto_wb"),
            ("WB R", "wb_r"),
            ("WB G", "wb_g"),
            ("WB B", "wb_b"),
        ]

        for idx, (label, key) in enumerate(labels, start=1):
            self._ttk.Label(frm, text=label).grid(row=idx, column=0, sticky="w", pady=2)
            self._ttk.Label(frm, textvariable=self._status_vars[key], width=18).grid(
                row=idx, column=1, sticky="w", pady=2
            )

        sep = self._ttk.Separator(frm, orient="horizontal")
        sep.grid(row=idx + 1, column=0, columnspan=4, sticky="ew", pady=(8, 6))

        self._ttk.Label(frm, text="手动调节", font=("Microsoft YaHei", 10, "bold")).grid(
            row=idx + 2, column=0, columnspan=4, sticky="w", pady=(2, 6)
        )

        manual_rows = [
            ("曝光 (ms)", "exposure", "ms"),
            ("增益 (dB)", "gain", "dB"),
            ("帧率 (Hz)", "framerate", "Hz"),
        ]
        base_row = idx + 3

        for offset, (label, key, unit) in enumerate(manual_rows):
            row_idx = base_row + offset
            self._ttk.Label(frm, text=label).grid(row=row_idx, column=0, sticky="w", pady=3)

            entry = self._ttk.Entry(frm, textvariable=self._manual_entries[key], width=12)
            entry.grid(row=row_idx, column=1, sticky="w", pady=3, padx=(4, 4))
            self._entry_widgets[key] = entry
            entry.bind("<FocusIn>", lambda _e, key=key: self._mark_dirty(key, source="entry"))
            entry.bind("<FocusOut>", lambda _e, key=key: self._entry_editing_keys.discard(key))

            display_limits = self._get_display_limits(key, self._limits.get(key))
            range_text = self._format_range(display_limits, unit)
            self._ttk.Label(frm, text=f"范围: {range_text}", width=18, foreground="#666666").grid(
                row=row_idx, column=2, sticky="w"
            )

            if key in self._slider_vars:
                self._build_slider(frm, row_idx, key)

        wb_title_row = base_row + len(manual_rows) + 1
        self._ttk.Label(frm, text="手动白平衡", font=("Microsoft YaHei", 10, "bold")).grid(
            row=wb_title_row, column=0, columnspan=4, sticky="w", pady=(12, 6)
        )

        wb_rows = [
            ("WB R", "wb_r"),
            ("WB G", "wb_g"),
            ("WB B", "wb_b"),
        ]
        wb_base_row = wb_title_row + 1
        for offset, (label, key) in enumerate(wb_rows):
            row_idx = wb_base_row + offset
            self._ttk.Label(frm, text=label).grid(row=row_idx, column=0, sticky="w", pady=3)
            entry = self._ttk.Entry(frm, textvariable=self._manual_entries[key], width=12)
            entry.grid(row=row_idx, column=1, sticky="w", pady=3, padx=(4, 4))
            self._entry_widgets[key] = entry

        button_row = wb_base_row + len(wb_rows) + 1

        self._ttk.Button(frm, text="应用曝光/增益/帧率", command=self._apply_manual).grid(
            row=button_row, column=0, columnspan=4, sticky="ew", pady=(8, 4)
        )

        self._ttk.Button(frm, text="开启自动白平衡", command=self._enable_auto_wb).grid(
            row=button_row + 1, column=0, columnspan=2, sticky="ew", pady=2
        )
        self._ttk.Button(frm, text="冻结当前自动白平衡", command=self._freeze_awb).grid(
            row=button_row + 1, column=2, columnspan=2, sticky="ew", pady=2
        )

        self._ttk.Button(frm, text="写入手动白平衡", command=self._apply_manual_wb).grid(
            row=button_row + 2, column=0, columnspan=2, sticky="ew", pady=2
        )
        self._ttk.Button(frm, text="保存当前全部参数", command=self._persist_all).grid(
            row=button_row + 2, column=2, columnspan=2, sticky="ew", pady=2
        )

        frm.columnconfigure(3, weight=1)

    def _build_slider(self, parent, row: int, key: str) -> None:
        limits = self._get_display_limits(key, self._limits.get(key))
        slider_var = self._slider_vars[key]
        slider = self._ttk.Scale(
            parent,
            from_=0,
            to=1,
            orient="horizontal",
            variable=slider_var,
            command=lambda val, key=key: self._on_slider_change(key, val),
        )
        slider.state(("disabled",))
        if limits and limits.get("min") is not None and limits.get("max") is not None:
            min_v = limits["min"]
            max_v = limits["max"]
            slider.configure(from_=min_v, to=max_v)
            slider.state(("!disabled",))
            initial = self._initial_slider_value(key, limits, min_v)
            self._suppress_slider_callback = True
            slider.set(initial)
            self._suppress_slider_callback = False
            self._manual_entries[key].set(self._format_slider_value(key, initial))
            if key == "framerate":
                self._manual_dirty_keys.add(key)

        slider.bind("<ButtonPress-1>", lambda _event, key=key: self._mark_dirty(key, source="slider"))
        slider.bind("<ButtonRelease-1>", lambda _event, key=key: self._on_slider_release(key))
        slider.grid(row=row, column=3, sticky="ew", padx=(6, 0))
        self._slider_widgets[key] = slider

    def _on_slider_release(self, key: str) -> None:
        self._active_slider_keys.discard(key)
        self._manual_dirty_keys.add(key)
        self._apply_manual(only_keys={key}, silent=True)

    def _mark_dirty(self, key: str, source: str) -> None:
        if source == "entry":
            self._entry_editing_keys.add(key)
        elif source == "slider":
            self._active_slider_keys.add(key)
        self._manual_dirty_keys.add(key)

    def _initial_slider_value(self, key: str, limits: Dict[str, Optional[float]], fallback: float) -> float:
        current = self._controller.query_runtime_status()
        key_map = {
            "exposure": "exposure_ms",
            "gain": "gain_db",
            "framerate": "framerate_hz",
        }
        status_key = key_map.get(key)
        if status_key and current.get(status_key) is not None:
            target = float(current[status_key])
        else:
            defaults = {"framerate": 10.0}
            target = defaults.get(key, fallback)
        return self._clamp_value(target, limits)

    def _refresh_status(self, force: bool = False) -> None:
        try:
            status = self._controller.query_runtime_status()
        except Exception as exc:
            if force:
                print(f"[ParameterPanel] 初始化状态失败：{exc}")
            for key in self._status_vars:
                self._status_vars[key].set("错误")
            return

        self._status_vars["exposure"].set(self._format_value(status.get("exposure_ms"), digits=2))
        self._status_vars["gain"].set(self._format_value(status.get("gain_db"), digits=2))
        self._status_vars["framerate"].set(self._format_value(status.get("framerate_hz"), digits=2))
        self._status_vars["measured_fps"].set(self._format_value(status.get("measured_fps"), digits=2))
        self._status_vars["aeag"].set("开启" if status.get("aeag") else "关闭")
        self._status_vars["auto_wb"].set("开启" if status.get("auto_wb") else "关闭")
        self._status_vars["wb_r"].set(self._format_value(status.get("wb_r"), digits=3))
        self._status_vars["wb_g"].set(self._format_value(status.get("wb_g"), digits=3))
        self._status_vars["wb_b"].set(self._format_value(status.get("wb_b"), digits=3))
        self._update_manual_inputs(status)

    def _on_slider_change(self, key: str, value: str) -> None:
        if self._suppress_slider_callback:
            return
        try:
            numeric = float(value)
        except ValueError:
            return
        self._manual_entries[key].set(self._format_slider_value(key, numeric))

    @staticmethod
    def _format_value(value: Optional[float], digits: int) -> str:
        if value is None:
            return "--"
        fmt = f"{{:.{digits}f}}"
        return fmt.format(value)

    @staticmethod
    def _format_range(limit: Optional[Dict[str, Optional[float]]], unit: str) -> str:
        if not limit:
            return "未知"
        min_v = limit.get("min")
        max_v = limit.get("max")
        if min_v is None or max_v is None:
            return "未知"
        digits = 2
        fmt = f"{{:.{digits}f}}"
        return f"{fmt.format(min_v)}~{fmt.format(max_v)} {unit}"

    @staticmethod
    def _format_slider_value(key: str, value: float) -> str:
        digits_map = {"exposure": 2, "gain": 2, "framerate": 2}
        digits = digits_map.get(key, 2)
        fmt = f"{{:.{digits}f}}"
        return fmt.format(value)

    def _get_display_limits(self, key: str, limits: Optional[Dict[str, Optional[float]]]) -> Optional[Dict[str, Optional[float]]]:
        if not limits:
            return limits
        converted = dict(limits)
        if key == "exposure":
            converted["min"] = self._controller.us_to_ms(converted.get("min"))
            converted["max"] = self._controller.us_to_ms(converted.get("max"))
        return converted

    def _update_manual_inputs(self, status: Dict[str, Optional[float]]) -> None:
        key_map = {
            "exposure": ("exposure_ms", 2),
            "gain": ("gain_db", 2),
            "framerate": ("framerate_hz", 2),
            "wb_r": ("wb_r", 3),
            "wb_g": ("wb_g", 3),
            "wb_b": ("wb_b", 3),
        }

        for key, (status_key, digits) in key_map.items():
            value = status.get(status_key)
            formatted = ""
            if value is not None:
                if key in ("exposure", "gain", "framerate"):
                    display_limits = self._get_display_limits(key, self._limits.get(key))
                    display_value = self._clamp_value(value, display_limits)
                    formatted = (
                        self._format_slider_value(key, display_value)
                        if key in self._slider_vars
                        else self._format_value(display_value, digits)
                    )
                else:
                    formatted = self._format_value(value, digits)

            entry = self._entry_widgets.get(key)
            if entry and key not in self._entry_editing_keys and key not in self._manual_dirty_keys:
                self._manual_entries[key].set(formatted)

            if (
                key in self._slider_vars
                and value is not None
                and key not in self._active_slider_keys
                and key not in self._manual_dirty_keys
            ):
                slider = self._slider_widgets.get(key)
                if slider:
                    display_limits = self._get_display_limits(key, self._limits.get(key))
                    clamped = self._clamp_value(value, display_limits)
                    self._suppress_slider_callback = True
                    slider.set(clamped)
                    self._suppress_slider_callback = False

    @staticmethod
    def _clamp_value(value: float, limits: Optional[Dict[str, Optional[float]]]) -> float:
        if not limits:
            return value
        min_v = limits.get("min")
        max_v = limits.get("max")
        if min_v is not None and value < min_v:
            value = min_v
        if max_v is not None and value > max_v:
            value = max_v
        return value

    def _apply_manual(self, only_keys: Optional[Set[str]] = None, silent: bool = False) -> None:
        def parse_float(value: str) -> Optional[float]:
            value = value.strip()
            return float(value) if value else None

        try:
            exposure = parse_float(self._manual_entries["exposure"].get())
            gain = parse_float(self._manual_entries["gain"].get())
            framerate = parse_float(self._manual_entries["framerate"].get())
        except ValueError:
            if not silent:
                self._messagebox.showerror("输入错误", "请确保曝光/增益/帧率均为数字。")
            return

        if only_keys is not None:
            if "exposure" not in only_keys:
                exposure = None
            if "gain" not in only_keys:
                gain = None
            if "framerate" not in only_keys:
                framerate = None

        if exposure is None and gain is None and framerate is None:
            if not silent:
                self._messagebox.showinfo("提示", "未填写任何参数，已忽略。")
            return

        try:
            self._controller.apply_manual_settings(
                exposure_ms=exposure,
                gain_db=gain,
                framerate_hz=framerate,
                persist=True,
            )
            if not silent:
                self._messagebox.showinfo("成功", "曝光/增益/帧率已写入并保存。")
        except Exception as exc:
            if silent:
                print(f"[ParameterPanel] 写入失败（{only_keys}）：{exc}")
            else:
                self._messagebox.showerror("写入失败", str(exc))

    def _enable_auto_wb(self) -> None:
        try:
            self._controller.enable_auto_white_balance()
            self._messagebox.showinfo("成功", "自动白平衡已开启。")
        except Exception as exc:
            self._messagebox.showerror("失败", str(exc))

    def _freeze_awb(self) -> None:
        try:
            wb = self._controller.freeze_auto_white_balance()
            self._messagebox.showinfo(
                "成功",
                f"自动白平衡已冻结：R={wb['wb_r']:.3f}, G={wb['wb_g']:.3f}, B={wb['wb_b']:.3f}"
            )
        except Exception as exc:
            self._messagebox.showerror("失败", str(exc))

    def _apply_manual_wb(self) -> None:
        def parse_required(value: str, name: str) -> float:
            value = value.strip()
            if not value:
                raise ValueError(f"{name} 不能为空")
            return float(value)

        try:
            wb_r = parse_required(self._manual_entries["wb_r"].get(), "WB R")
            wb_g = parse_required(self._manual_entries["wb_g"].get(), "WB G")
            wb_b = parse_required(self._manual_entries["wb_b"].get(), "WB B")

            self._controller.disable_auto_white_balance()
            self._controller.set_manual_white_balance(wb_r, wb_g, wb_b)
            self._messagebox.showinfo("成功", "手动白平衡已写入并保存。")
        except Exception as exc:
            self._messagebox.showerror("失败", str(exc))

    def _persist_all(self) -> None:
        try:
            status = self._controller.persist_current_state()
            self._messagebox.showinfo(
                "成功",
                "当前相机参数已保存。\n"
                f"曝光={status.get('exposure_ms'):.2f} ms\n"
                f"增益={status.get('gain_db'):.2f} dB\n"
                f"帧率={status.get('framerate_hz'):.2f} Hz"
            )
        except Exception as exc:
            self._messagebox.showerror("失败", str(exc))


if __name__ == "__main__":
    controller = XimeaCameraController(exposure_us=20000)
    try:
        controller.open()
        controller.apply_saved_or_default_profile()
        controller.start_rgb_mode(bit_depth=24)
        controller.show_parameter_panel()

        while True:
            frame = controller.grab_frame()
            controller.process_panel_events()
            cv2.imshow("XIMEA Preview", cv2.cvtColor(frame[..., :3], cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        controller.close()
        cv2.destroyAllWindows()
