# -*- coding: utf-8 -*-
"""HTTP MJPEG 관전 경로 — UDP 없이 TCP만으로 Isaac Sim 화면을 본다.

view_scene.py 가 `from http_stream import HttpViewer` 로 부르는데 팀 저장소에
파일이 빠져 있어서(README에는 기재) 구현한 것. 팀 저장소에 반영 필요.

사용 (kit 앱 안에서):
    from http_stream import HttpViewer
    viewer = HttpViewer(port=8211)          # 데몬 스레드로 서버 기동
    while app.is_running():
        app.update()
        viewer.tick()                        # 주기 캡처 (in-flight 1건)

접속: http://<서버IP>:8211/     브라우저 MJPEG (조작 불가, 관전 전용)
      http://<서버IP>:8211/frame 단장 JPEG
      http://<서버IP>:8211/stats JSON 상태

왜 필요한가
  WebRTC는 시그널링 49100/TCP + 미디어 47998/**UDP** 를 쓴다. UDP가 막힌 망
  (교육장 내부망, tailscale이 DERP/TCP 릴레이로 떨어지는 환경)에서는 화면이 안 온다.
  MJPEG은 순수 HTTP라 TCP 한 포트만 열면 된다. NVENC를 안 쓰므로
  NVST_R_BUSY(인코더 중복 점유) 문제도 구조적으로 없고, 여러 명이 동시에 볼 수 있다.

한계
  H.264 대비 대역폭이 크다(프레임마다 완전한 JPEG). 720p 6~15fps 수준.
  시점 조작 불가 — 카메라는 앱 쪽에서 정한 것을 그대로 본다.
"""
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_BOUNDARY = "frameboundary"


class _State:
    """최신 JPEG 한 장과 통계. 캡처 스레드와 HTTP 스레드가 공유."""

    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.seq = 0
        self.captured = 0
        self.dropped = 0
        self.encode_ms = 0.0
        self.started = time.time()

    def put(self, data, encode_ms):
        with self.lock:
            self.jpeg = data
            self.seq += 1
            self.captured += 1
            self.encode_ms = encode_ms

    def get(self):
        with self.lock:
            return self.jpeg, self.seq

    def stats(self):
        with self.lock:
            up = max(time.time() - self.started, 1e-6)
            return dict(frames=self.captured, dropped=self.dropped,
                        fps=round(self.captured / up, 2),
                        encode_ms=round(self.encode_ms, 1),
                        bytes=len(self.jpeg) if self.jpeg else 0,
                        uptime_s=round(up, 1))


def _make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):        # 요청마다 stdout을 더럽히지 않는다
            pass

        def _no_cache(self):
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self._page()
            if path == "/stream":
                return self._stream()
            if path == "/frame":
                return self._frame()
            if path == "/stats":
                return self._stats()
            self.send_error(404)

        def _page(self):
            html = (
                "<!doctype html><meta charset='utf-8'>"
                "<title>Isaac Sim — MJPEG</title>"
                "<style>html,body{margin:0;background:#111;height:100%}"
                "img{display:block;width:100%;height:100%;object-fit:contain}</style>"
                "<img src='/stream' alt='stream'>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self._no_cache()
            self.end_headers()
            self.wfile.write(html)

        def _frame(self):
            data, _ = state.get()
            if data is None:
                return self.send_error(503, "no frame yet")
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self._no_cache()
            self.end_headers()
            self.wfile.write(data)

        def _stats(self):
            body = json.dumps(state.stats()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self._no_cache()
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
            self._no_cache()
            self.end_headers()
            last = -1
            try:
                while True:
                    data, seq = state.get()
                    if data is None or seq == last:
                        time.sleep(0.02)          # 새 프레임이 없으면 잠깐 양보
                        continue
                    last = seq
                    self.wfile.write(
                        f"--{_BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                        f"Content-Length: {len(data)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                              # 브라우저가 닫음 — 정상
    return Handler


class HttpViewer:
    """활성 뷰포트를 주기적으로 캡처해 MJPEG으로 내보낸다.

    캡처는 **in-flight 1건**만 유지한다. 이전 캡처가 안 끝났는데 새로 걸면
    지연이 누적되고 앱이 느려진다.
    """

    def __init__(self, port=8211, fps=8, quality=70, host="0.0.0.0", viewport=None):
        self.port = port
        self.interval = 1.0 / max(fps, 1)
        self.quality = quality
        self.state = _State()
        self._inflight = False
        self._next_t = 0.0
        self._vp = viewport
        self._warned = False

        self.server = ThreadingHTTPServer((host, port), _make_handler(self.state))
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        print(f"[http_stream] http://0.0.0.0:{port}/  (MJPEG, 목표 {fps}fps, q{quality})",
              flush=True)

    # ---- 내부 ----
    def _viewport(self):
        if self._vp is None:
            from omni.kit.viewport.utility import get_active_viewport
            self._vp = get_active_viewport()
        return self._vp

    @staticmethod
    def _as_array(buf, buf_size):
        """ByteCapture 콜백의 버퍼를 numpy uint8 배열로.

        Omniverse는 여기에 **PyCapsule**을 넘긴다. memoryview/bytes로 바로 못 받고
        ctypes로 포인터를 꺼내야 한다 (실측: "a bytes-like object is required,
        not 'PyCapsule'"). bytes-like를 주는 빌드도 있어 양쪽을 다 받는다.
        """
        import ctypes
        import numpy as np
        try:
            return np.frombuffer(memoryview(buf), dtype=np.uint8)
        except TypeError:
            pass
        ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.POINTER(
            ctypes.c_byte * buf_size)
        ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object,
                                                          ctypes.c_char_p]
        ptr = ctypes.pythonapi.PyCapsule_GetPointer(buf, None)
        return np.frombuffer(ptr.contents, dtype=np.uint8)

    def _encode(self, buf, buf_size, width, height, fmt):
        """캡처 버퍼 -> JPEG bytes."""
        import numpy as np
        from PIL import Image
        arr = self._as_array(buf, buf_size)
        need = width * height * 4
        if arr.size < need:                       # 예상과 다른 포맷이면 포기
            raise ValueError(f"buffer {arr.size} < {need} (fmt={fmt})")
        img = arr[:need].reshape(height, width, 4)[:, :, :3]
        out = io.BytesIO()
        Image.fromarray(img).save(out, format="JPEG", quality=self.quality)
        return out.getvalue()

    def _on_bytes(self, buf, buf_size, width, height, fmt):
        t0 = time.time()
        try:
            self.state.put(self._encode(buf, buf_size, width, height, fmt),
                           (time.time() - t0) * 1000.0)
        except Exception as e:
            if not self._warned:
                print(f"[http_stream] 인코딩 실패: {e}", flush=True)
                self._warned = True
        finally:
            self._inflight = False

    # ---- 앱 루프에서 매 프레임 호출 ----
    def tick(self):
        now = time.time()
        if self._inflight:
            return
        if now < self._next_t:
            return
        self._next_t = now + self.interval
        try:
            from omni.kit.widget.viewport.capture import ByteCapture
            self._inflight = True
            self._viewport().schedule_capture(ByteCapture(self._on_bytes))
        except Exception as e:
            self._inflight = False
            self.state.dropped += 1
            if not self._warned:
                print(f"[http_stream] 캡처 실패: {e}", flush=True)
                self._warned = True

    def close(self):
        try:
            self.server.shutdown()
        except Exception:
            pass
