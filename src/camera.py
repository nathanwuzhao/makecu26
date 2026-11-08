"""Webcam in a background thread; always hands out the newest frame."""

import threading
import time

import cv2 as cv


class Camera:
    def __init__(self, index: int = 0, width: int = 1280, height: int = 720):
        self.cap = cv.VideoCapture(index, cv.CAP_DSHOW)  # DirectShow: fast open on Windows
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"camera {index} did not open")
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> "Camera":
        self._thread.start()
        t0 = time.time()
        while self.frame() is None and time.time() - t0 < 3:
            time.sleep(0.05)
        return self

    def _loop(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self._frame = f

    def frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def jpeg(self, max_width: int = 640, quality: int = 80) -> bytes | None:
        """Newest frame, downscaled (fewer tokens, faster upload) and JPEG-encoded."""
        f = self.frame()
        if f is None:
            return None
        h, w = f.shape[:2]
        if w > max_width:
            f = cv.resize(f, (max_width, int(h * max_width / w)), interpolation=cv.INTER_AREA)
        ok, buf = cv.imencode(".jpg", f, [cv.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else None

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self.cap.release()
