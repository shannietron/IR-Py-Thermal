#!/usr/bin/env python3
"""macOS capture backend for Xtherm / HT-301 / InfiRay thermal cameras.

On Linux the library drives the camera through ``cv2.VideoCapture`` on V4L2 and relies
on two things that OpenCV's AVFoundation backend does not implement:

* ``CAP_PROP_CONVERT_RGB = 0`` to receive the untouched YUYV bytes, which the camera
  actually fills with little-endian uint16 sensor data (``height`` image rows plus 4
  metadata rows).
* ``CAP_PROP_ZOOM = value`` as the camera's command register (UVC Camera-Terminal
  "Zoom Absolute" control).

:class:`MacVideoCapture` is a duck-typed stand-in for ``cv2.VideoCapture`` that provides
exactly the surface :class:`irpythermal.Camera` uses. Frames come from AVFoundation via
PyObjC in the camera's native ``yuvs`` (YUYV) pixel format; commands go through the small
IOKit helper in ``macos/libuvcctl.dylib`` (built with ``make -C macos``).

Run ``python macos_capture.py`` to list devices, ``python macos_capture.py --test [dev]``
to grab a few frames and print statistics.
"""

from __future__ import annotations

import ctypes
import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

if sys.platform != "darwin":  # pragma: no cover
    raise ImportError("macos_capture only works on macOS")

import objc  # noqa: E402
import AVFoundation as AVF  # noqa: E402
import CoreMedia  # noqa: E402
import libdispatch  # noqa: E402
import Quartz  # noqa: E402  (CoreVideo lives here)
from Foundation import NSObject  # noqa: E402

FOURCC_YUVS = 0x79757673  # 'yuvs' kCVPixelFormatType_422YpCbCr8_yuvs == YUYV (what the camera sends)
FOURCC_2VUY = 0x32767579  # '2vuy' kCVPixelFormatType_422YpCbCr8 == UYVY (byte-swapped pairs)
ROWS_SPECIAL_DATA = 4
_HERE = os.path.dirname(os.path.abspath(__file__))
_DYLIB = os.path.join(_HERE, "macos", "libuvcctl.dylib")


def fourcc_str(code: int) -> str:
    return "".join(chr((code >> s) & 0xFF) for s in (24, 16, 8, 0))


# --------------------------------------------------------------------------- UVC control


class UvcControl:
    """ctypes wrapper around macos/libuvcctl.dylib (UVC zoom-absolute as command register)."""

    def __init__(self, vid: int = 0, pid: int = 0, lib_path: str = _DYLIB, debug: bool = False):
        if not os.path.exists(lib_path):
            raise FileNotFoundError(
                f"{lib_path} not found. Build it with: make -C {os.path.dirname(lib_path)}"
            )
        lib = ctypes.CDLL(lib_path)
        lib.uvcctl_open.argtypes = [ctypes.c_uint16, ctypes.c_uint16, ctypes.POINTER(ctypes.c_void_p)]
        lib.uvcctl_open.restype = ctypes.c_int
        lib.uvcctl_close.argtypes = [ctypes.c_void_p]
        lib.uvcctl_last_error.argtypes = [ctypes.c_void_p]
        lib.uvcctl_last_error.restype = ctypes.c_char_p
        lib.uvcctl_set_debug.argtypes = [ctypes.c_int]
        lib.uvcctl_set_zoom.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
        lib.uvcctl_set_zoom.restype = ctypes.c_int
        lib.uvcctl_get_zoom.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint16)]
        lib.uvcctl_get_zoom.restype = ctypes.c_int
        for name in ("uvcctl_interface_number", "uvcctl_camera_terminal_id", "uvcctl_processing_unit_id"):
            getattr(lib, name).argtypes = [ctypes.c_void_p]
            getattr(lib, name).restype = ctypes.c_uint8
        lib.uvcctl_set_camera_terminal_id.argtypes = [ctypes.c_void_p, ctypes.c_uint8]
        lib.uvcctl_request.argtypes = [
            ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_uint16,
        ]
        lib.uvcctl_request.restype = ctypes.c_int
        self._lib = lib
        lib.uvcctl_set_debug(1 if debug else 0)
        self._h = ctypes.c_void_p()
        rc = lib.uvcctl_open(vid, pid, ctypes.byref(self._h))
        if rc != 0:
            msg = lib.uvcctl_last_error(self._h) or b""
            lib.uvcctl_close(self._h)
            self._h = None
            raise RuntimeError(f"uvcctl_open(vid=0x{vid:04x}, pid=0x{pid:04x}) failed rc={rc}: {msg.decode()}")
        self._lock = threading.Lock()

    @property
    def interface_number(self) -> int:
        return self._lib.uvcctl_interface_number(self._h)

    @property
    def camera_terminal_id(self) -> int:
        return self._lib.uvcctl_camera_terminal_id(self._h)

    @camera_terminal_id.setter
    def camera_terminal_id(self, value: int) -> None:
        self._lib.uvcctl_set_camera_terminal_id(self._h, value)

    def last_error(self) -> str:
        return (self._lib.uvcctl_last_error(self._h) or b"").decode()

    def set_zoom(self, value: int) -> bool:
        with self._lock:
            return self._lib.uvcctl_set_zoom(self._h, int(value) & 0xFFFF) == 0

    def get_zoom(self) -> Optional[int]:
        out = ctypes.c_uint16(0)
        with self._lock:
            rc = self._lib.uvcctl_get_zoom(self._h, ctypes.byref(out))
        return out.value if rc == 0 else None

    def request(self, request: int, unit_id: int, selector: int, data: bytes | int) -> Optional[bytes]:
        """Raw class request. ``data`` is the payload for SET_* or the length for GET_*."""
        if isinstance(data, int):
            buf = ctypes.create_string_buffer(data)
            length = data
        else:
            buf = ctypes.create_string_buffer(bytes(data), len(data))
            length = len(data)
        with self._lock:
            rc = self._lib.uvcctl_request(self._h, request, unit_id, selector, buf, length)
        return buf.raw[:length] if rc == 0 else None

    def close(self) -> None:
        if self._h:
            self._lib.uvcctl_close(self._h)
            self._h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- device listing


@dataclass
class FormatInfo:
    width: int
    height: int
    fourcc: int
    max_fps: float
    _fmt: object

    def __repr__(self) -> str:
        return f"{self.width}x{self.height} {fourcc_str(self.fourcc)} @{self.max_fps:g}fps"


@dataclass
class DeviceInfo:
    index: int
    name: str
    unique_id: str
    model_id: str
    vid: int
    pid: int
    formats: list[FormatInfo]
    _dev: object

    def __repr__(self) -> str:
        return (
            f"[{self.index}] {self.name!r} vid=0x{self.vid:04x} pid=0x{self.pid:04x} "
            f"formats={self.formats}"
        )


def _device_formats(dev) -> list[FormatInfo]:
    out = []
    for f in dev.formats():
        desc = f.formatDescription()
        dims = CoreMedia.CMVideoFormatDescriptionGetDimensions(desc)
        sub = CoreMedia.CMFormatDescriptionGetMediaSubType(desc)
        fps = 0.0
        for r in f.videoSupportedFrameRateRanges():
            fps = max(fps, float(r.maxFrameRate()))
        out.append(FormatInfo(int(dims.width), int(dims.height), int(sub), fps, f))
    return out


def _parse_vid_pid(model_id: str) -> tuple[int, int]:
    m = re.search(r"VendorID_(\d+)\s+ProductID_(\d+)", model_id or "")
    return (int(m.group(1)), int(m.group(2))) if m else (0, 0)


def list_devices() -> list[DeviceInfo]:
    """All AVFoundation video devices, external USB cameras first."""
    types = []
    for name in ("AVCaptureDeviceTypeExternal", "AVCaptureDeviceTypeExternalUnknown",
                 "AVCaptureDeviceTypeBuiltInWideAngleCamera"):
        if hasattr(AVF, name):
            types.append(getattr(AVF, name))
        if name == "AVCaptureDeviceTypeExternal" and types:
            continue
    devices = []
    try:
        sess = AVF.AVCaptureDeviceDiscoverySession.discoverySessionWithDeviceTypes_mediaType_position_(
            types, AVF.AVMediaTypeVideo, AVF.AVCaptureDevicePositionUnspecified
        )
        devices = list(sess.devices())
    except Exception:
        devices = []
    if not devices:
        devices = list(AVF.AVCaptureDevice.devicesWithMediaType_(AVF.AVMediaTypeVideo))
    out = []
    for i, d in enumerate(devices):
        model = str(d.modelID() or "")
        vid, pid = _parse_vid_pid(model)
        out.append(DeviceInfo(i, str(d.localizedName()), str(d.uniqueID()), model, vid, pid, _device_formats(d), d))
    return out


def request_camera_permission(timeout: float = 60.0) -> bool:
    status = AVF.AVCaptureDevice.authorizationStatusForMediaType_(AVF.AVMediaTypeVideo)
    if status == AVF.AVAuthorizationStatusAuthorized:
        return True
    if status == AVF.AVAuthorizationStatusNotDetermined:
        done = threading.Event()
        result = {"ok": False}

        def _cb(granted):
            result["ok"] = bool(granted)
            done.set()

        AVF.AVCaptureDevice.requestAccessForMediaType_completionHandler_(AVF.AVMediaTypeVideo, _cb)
        print("Waiting for macOS camera permission prompt...")
        done.wait(timeout)
        return result["ok"]
    return False


# --------------------------------------------------------------------------- capture


class _SampleDelegate(NSObject, protocols=[objc.protocolNamed("AVCaptureVideoDataOutputSampleBufferDelegate")]):
    owner = objc.ivar()

    def initWithOwner_(self, owner):
        self = objc.super(_SampleDelegate, self).init()
        if self is None:
            return None
        self.owner = owner
        return self

    def captureOutput_didOutputSampleBuffer_fromConnection_(self, output, sample_buffer, connection):
        pb = CoreMedia.CMSampleBufferGetImageBuffer(sample_buffer)
        if pb is None:
            return
        try:
            self.owner._on_pixel_buffer(pb)
        except Exception as e:  # never let an exception escape into the ObjC runtime
            self.owner._last_exc = e

    def captureOutput_didDropSampleBuffer_fromConnection_(self, output, sample_buffer, connection):
        self.owner._dropped += 1


class MacVideoCapture:
    """cv2.VideoCapture look-alike backed by AVFoundation + IOKit UVC control.

    ``read()`` returns ``(True, frame)`` where ``frame`` is a ``(rows, width*2)`` uint8 array
    of the raw YUYV bytes, so ``frame.view(np.uint16).ravel()`` yields the camera's
    little-endian 16-bit words exactly like the V4L2 path.
    """

    supported_resolutions = {(240, 180), (256, 192), (384, 288), (640, 512)}

    def __init__(
        self,
        device: int | str | DeviceInfo | None = None,
        fmt: FormatInfo | None = None,
        control: bool = True,
        read_timeout: float = 2.0,
        debug: bool = False,
        warmup_frames: int = 2,
    ):
        self._lock = threading.Condition()
        self._frame: Optional[np.ndarray] = None
        self._seq = 0
        self._read_seq = 0
        self._dropped = 0
        self._received = 0
        self._last_exc: Optional[Exception] = None
        self._swap_bytes = False
        self._read_timeout = read_timeout
        self._debug = debug
        self._warmup = warmup_frames  # first frames after startRunning are garbage
        self.ctl: Optional[UvcControl] = None

        if not request_camera_permission():
            raise PermissionError("Camera access denied. Allow it in System Settings > Privacy & Security > Camera.")

        self.dev_info = self._resolve_device(device)
        self.fmt = fmt or self._pick_format(self.dev_info)
        dev = self.dev_info._dev

        if control:
            try:
                self.ctl = UvcControl(self.dev_info.vid, self.dev_info.pid, debug=debug)
            except Exception as e:
                print(f"[macos_capture] UVC control unavailable: {e}")
                self.ctl = None

        # pixel format: prefer native yuvs; accept 2vuy with a byte swap.
        self._session = AVF.AVCaptureSession.alloc().init()
        preset = getattr(AVF, "AVCaptureSessionPresetInputPriority", None)
        if preset is not None and self._session.canSetSessionPreset_(preset):
            self._session.setSessionPreset_(preset)
        inp, err = AVF.AVCaptureDeviceInput.deviceInputWithDevice_error_(dev, None)
        if inp is None:
            raise RuntimeError(f"AVCaptureDeviceInput failed: {err}")
        if not self._session.canAddInput_(inp):
            raise RuntimeError("cannot add camera input to session")
        self._session.addInput_(inp)

        out = AVF.AVCaptureVideoDataOutput.alloc().init()
        out.setAlwaysDiscardsLateVideoFrames_(True)
        available = [int(x) for x in out.availableVideoCVPixelFormatTypes()]
        if self._debug:
            print("[macos_capture] available output formats:", [fourcc_str(x) for x in available])
        if self.fmt.fourcc == FOURCC_YUVS and FOURCC_YUVS in available:
            want = FOURCC_YUVS
        elif FOURCC_2VUY in available:
            want = FOURCC_2VUY
            self._swap_bytes = True
        elif FOURCC_YUVS in available:
            want = FOURCC_YUVS
        else:
            raise RuntimeError(f"no 16-bit-per-pixel passthrough format available; got {[fourcc_str(x) for x in available]}")
        self._out_fourcc = want
        out.setVideoSettings_({Quartz.kCVPixelBufferPixelFormatTypeKey: want})
        self._delegate = _SampleDelegate.alloc().initWithOwner_(self)
        self._queue = libdispatch.dispatch_queue_create(b"irpythermal.capture", None)
        out.setSampleBufferDelegate_queue_(self._delegate, self._queue)
        if not self._session.canAddOutput_(out):
            raise RuntimeError("cannot add video data output to session")
        self._session.addOutput_(out)
        self._output = out

        ok, err = dev.lockForConfiguration_(None)
        if ok:
            try:
                dev.setActiveFormat_(self.fmt._fmt)
                ranges = self.fmt._fmt.videoSupportedFrameRateRanges()
                if ranges:
                    r = ranges[0]
                    dev.setActiveVideoMinFrameDuration_(r.minFrameDuration())
                    dev.setActiveVideoMaxFrameDuration_(r.minFrameDuration())
            finally:
                dev.unlockForConfiguration()
        else:
            print(f"[macos_capture] lockForConfiguration failed: {err}")

        self._width = self.fmt.width
        self._rows = self.fmt.height
        self._fps = self.fmt.max_fps
        self._session.startRunning()
        self._opened = True

    # -- device selection -------------------------------------------------------------

    @classmethod
    def _resolve_device(cls, device) -> DeviceInfo:
        devices = list_devices()
        if not devices:
            raise RuntimeError("no AVFoundation video devices found")
        if isinstance(device, DeviceInfo):
            return device
        if device is None:
            for d in devices:
                if cls._pick_format(d, strict=False) is not None:
                    return d
            raise RuntimeError(
                "no thermal camera found (looking for a YUYV format of width x (height+4) in "
                f"{sorted(cls.supported_resolutions)}). Devices: {devices}"
            )
        if isinstance(device, int) or (isinstance(device, str) and device.isdigit()):
            idx = int(device)
            if 0 <= idx < len(devices):
                return devices[idx]
            raise RuntimeError(f"device index {idx} out of range; devices: {devices}")
        needle = str(device).lower()
        for d in devices:
            if needle in d.name.lower() or needle in d.unique_id.lower() or needle in d.model_id.lower():
                return d
        raise RuntimeError(f"no device matching {device!r}; devices: {devices}")

    @classmethod
    def _pick_format(cls, d: DeviceInfo, strict: bool = True) -> Optional[FormatInfo]:
        candidates = [
            f for f in d.formats
            if f.fourcc in (FOURCC_YUVS, FOURCC_2VUY)
            and (f.width, f.height - ROWS_SPECIAL_DATA) in cls.supported_resolutions
        ]
        if candidates:
            candidates.sort(key=lambda f: (f.fourcc != FOURCC_YUVS, -f.max_fps))
            return candidates[0]
        if strict:
            raise RuntimeError(f"device {d.name!r} has no supported thermal format: {d.formats}")
        return None

    # -- frame path ---------------------------------------------------------------------

    def _on_pixel_buffer(self, pb) -> None:
        Quartz.CVPixelBufferLockBaseAddress(pb, 1)  # kCVPixelBufferLock_ReadOnly
        try:
            h = int(Quartz.CVPixelBufferGetHeight(pb))
            w = int(Quartz.CVPixelBufferGetWidth(pb))
            bpr = int(Quartz.CVPixelBufferGetBytesPerRow(pb))
            size = int(Quartz.CVPixelBufferGetDataSize(pb))
            base = Quartz.CVPixelBufferGetBaseAddress(pb)
            if hasattr(base, "as_buffer"):
                raw = np.frombuffer(base.as_buffer(size), dtype=np.uint8, count=size)
            elif isinstance(base, int):
                raw = np.frombuffer(ctypes.string_at(base, size), dtype=np.uint8)
            else:
                raw = np.frombuffer(bytes(base)[:size], dtype=np.uint8)
            rowbytes = w * 2
            frame = raw[: h * bpr].reshape(h, bpr)[:, :rowbytes].copy()
        finally:
            Quartz.CVPixelBufferUnlockBaseAddress(pb, 1)
        if self._swap_bytes:
            frame = frame.reshape(h, w, 2)[:, :, ::-1].reshape(h, rowbytes).copy()
        with self._lock:
            self._received += 1
            if self._warmup > 0:
                self._warmup -= 1
                return
            self._frame = frame
            self._seq += 1
            self._lock.notify_all()

    def read(self, timeout: Optional[float] = None):
        timeout = self._read_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._seq <= self._read_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, None
                self._lock.wait(remaining)
            self._read_seq = self._seq
            return True, self._frame

    def grab(self) -> bool:
        return self.read()[0]

    def retrieve(self):
        with self._lock:
            return self._frame is not None, self._frame

    # -- cv2.VideoCapture surface -------------------------------------------------------

    def isOpened(self) -> bool:  # noqa: N802 (cv2 naming)
        return getattr(self, "_opened", False)

    def get(self, prop: int) -> float:
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self._width)
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self._rows)
        if prop == cv2.CAP_PROP_FPS:
            return float(self._fps)
        if prop == cv2.CAP_PROP_CONVERT_RGB:
            return 0.0
        if prop == cv2.CAP_PROP_ZOOM:
            v = self.ctl.get_zoom() if self.ctl else None
            return float(v) if v is not None else -1.0
        if prop == cv2.CAP_PROP_FOURCC:
            return float(self._out_fourcc)
        return 0.0

    def set(self, prop: int, value) -> bool:
        if prop == cv2.CAP_PROP_CONVERT_RGB:
            return True  # always raw
        if prop == cv2.CAP_PROP_ZOOM:
            if self.ctl is None:
                print(f"[macos_capture] cannot send command 0x{int(value):04x}: UVC control not available")
                return False
            ok = self.ctl.set_zoom(int(value))
            if self._debug:
                print(f"[macos_capture] zoom<-0x{int(value):04x} {'ok' if ok else 'FAIL ' + self.ctl.last_error()}")
            return ok
        return False

    def release(self) -> None:
        if getattr(self, "_opened", False):
            self._session.stopRunning()
            self._opened = False
        if self.ctl:
            self.ctl.close()
            self.ctl = None

    def stats(self) -> dict:
        return {"received": self._received, "dropped": self._dropped, "swap_bytes": self._swap_bytes,
                "format": repr(self.fmt), "output": fourcc_str(self._out_fourcc), "last_exc": repr(self._last_exc)}

    @classmethod
    def find(cls, **kw) -> "MacVideoCapture":
        return cls(device=None, **kw)


# --------------------------------------------------------------------------- CLI


def _main(argv: list[str]) -> int:
    import argparse

    ap = argparse.ArgumentParser(description="List AVFoundation devices / test thermal capture on macOS")
    ap.add_argument("--test", nargs="?", const="auto", metavar="DEV",
                    help="grab frames from DEV (index, name substring, or 'auto' for the thermal camera)")
    ap.add_argument("--any-format", action="store_true", help="with --test: use the device's first format, not a thermal one")
    ap.add_argument("-n", type=int, default=30, help="frames to grab")
    ap.add_argument("--zoom", type=lambda s: int(s, 0), help="send this zoom-absolute value before grabbing (e.g. 0x8004)")
    ap.add_argument("--save", help="save the last frame's raw bytes to this .npy (uint16, rows x width)")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args(argv)

    request_camera_permission()
    devices = list_devices()
    print("AVFoundation video devices:")
    for d in devices:
        print(" ", d)
    if args.test is None:
        return 0

    device = None if args.test == "auto" else args.test
    fmt = None
    if args.any_format:
        info = MacVideoCapture._resolve_device(device) if device is not None else devices[0]
        device, fmt = info, info.formats[0]
    cap = MacVideoCapture(device=device, fmt=fmt, debug=args.debug)
    print("Using:", cap.dev_info.name, cap.fmt, "output", fourcc_str(cap._out_fourcc))
    if cap.ctl:
        print(f"UVC control: iface={cap.ctl.interface_number} camera_terminal={cap.ctl.camera_terminal_id} "
              f"zoom GET_CUR={cap.ctl.get_zoom()}")
    if args.zoom is not None:
        print(f"SET zoom 0x{args.zoom:04x} ->", cap.set(cv2.CAP_PROP_ZOOM, args.zoom))
        time.sleep(0.5)
    t0 = time.time()
    last = None
    for i in range(args.n):
        ok, frame = cap.read()
        if not ok:
            print(f"frame {i}: timeout")
            continue
        u16 = frame.view(np.uint16)
        last = u16
        if i < 3 or i == args.n - 1:
            img = u16[: u16.shape[0] - ROWS_SPECIAL_DATA] if u16.shape[0] > ROWS_SPECIAL_DATA else u16
            print(f"frame {i}: shape={u16.shape} img min={img.min()} max={img.max()} mean={img.mean():.1f} "
                  f"std={img.std():.1f} meta_row0[:8]={u16[-ROWS_SPECIAL_DATA][:8].tolist()}")
    dt = time.time() - t0
    print(f"{args.n} frames in {dt:.2f}s = {args.n / dt:.1f} fps; stats={cap.stats()}")
    if last is not None and args.save:
        np.save(args.save, last)
        print("saved", args.save)
    cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
