#!/usr/bin/env python3
"""
KPatrol MJPEG Camera Server v2
==============================
Ultra-low latency MJPEG streaming optimized for web browsers.
Uses picamera2 for OV5647 camera on Raspberry Pi.

Endpoints:
- GET /           → Status page
- GET /stream     → MJPEG stream (low latency)
- GET /stream/hd  → MJPEG stream (high quality)
- GET /snapshot   → Single JPEG frame
- GET /status     → JSON status
"""

from flask import Flask, Response, jsonify, render_template_string, request
from flask_cors import CORS
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder, Quality
from picamera2.outputs import FileOutput
import functools
import logging
import os
import threading
import io
import time
import socket

try:
    import jwt as pyjwt  # PyJWT
except ImportError:
    pyjwt = None

_ROBOT_SERIAL = os.environ.get("ROBOT_SERIAL", "KPATROL-001")

# Stream-token verification — backend signs short-TTL JWTs (scope=mjpeg) that
# the operator's browser passes via ?token= or `Authorization: Bearer …`. We
# fail-secure: if STREAM_TOKEN_SECRET is set we REQUIRE a valid token; if it's
# unset we disable auth and log loudly (dev-only path, never deploy like that).
_STREAM_TOKEN_SECRET = os.environ.get("STREAM_TOKEN_SECRET", "").strip()
_STREAM_TOKEN_SCOPE = "mjpeg"
_STREAM_TOKEN_LEEWAY_SEC = 5  # tolerate small Pi/server clock drift


def _extract_token(req):
    qs = req.args.get("token")
    if qs:
        return qs
    auth = req.headers.get("Authorization", "")
    if auth.lower().startswith("bearer "):
        return auth[7:].strip()
    return None


def _verify_token(token):
    """Returns (ok, reason). Reason is for logging — never leak to clients."""
    if pyjwt is None:
        return False, "PyJWT not installed"
    if not token:
        return False, "missing token"
    try:
        claims = pyjwt.decode(
            token,
            _STREAM_TOKEN_SECRET,
            algorithms=["HS256"],
            leeway=_STREAM_TOKEN_LEEWAY_SEC,
            options={"require": ["exp", "scope"]},
        )
    except pyjwt.ExpiredSignatureError:
        return False, "expired"
    except pyjwt.InvalidTokenError as e:
        return False, f"invalid: {e}"
    if claims.get("scope") != _STREAM_TOKEN_SCOPE:
        return False, "wrong scope"
    return True, None


def require_stream_token(view):
    """Decorator: enforce stream-token auth when STREAM_TOKEN_SECRET is set."""
    @functools.wraps(view)
    def wrapped(*args, **kwargs):
        if not _STREAM_TOKEN_SECRET:
            return view(*args, **kwargs)
        ok, reason = _verify_token(_extract_token(request))
        if not ok:
            # Log without echoing the token itself — referer/proxy logs already
            # carry enough risk.
            logging.getLogger("mjpeg").warning(
                "stream auth rejected from %s: %s", request.remote_addr, reason,
            )
            return jsonify({"error": "unauthorized"}), 401
        return view(*args, **kwargs)
    return wrapped

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")

app = Flask(__name__)
CORS(app)

# Configuration - Balanced: Good quality + Low latency
RESOLUTION_BALANCED = (960, 540)   # 540p - best balance
RESOLUTION_HD = (1280, 720)        # HD stream
FRAMERATE = 30
QUALITY_BALANCED = 65              # Better quality, still fast

# Global camera instance
camera = None
output = None
frame_lock = threading.Lock()
latest_frame = None
frame_count = 0
start_time = time.time()
current_resolution = RESOLUTION_BALANCED


class StreamingOutput(io.BufferedIOBase):
    """Zero-copy buffered output for MJPEG streaming"""
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()
        self.timestamp = 0

    def write(self, buf):
        global latest_frame, frame_count
        with self.condition:
            self.frame = buf
            self.timestamp = time.time()
            frame_count += 1
            self.condition.notify_all()
        with frame_lock:
            latest_frame = buf
        return len(buf)


def init_camera():
    """Initialize picamera2 with balanced settings (quality + speed)"""
    global camera, output, current_resolution
    
    camera = Picamera2()
    
    # 960x540 for good quality with reasonable bandwidth
    config = camera.create_video_configuration(
        main={"size": RESOLUTION_BALANCED, "format": "RGB888"},
        controls={
            "FrameRate": FRAMERATE,
            "AwbEnable": True,          # Auto white balance
            "AeEnable": True,           # Auto exposure
            "NoiseReductionMode": 1,    # Fast noise reduction
            "Sharpness": 1.2,           # Slightly sharpen
        },
        buffer_count=2,                 # Minimal buffers for low latency
        queue=False                     # Don't queue frames
    )
    camera.configure(config)
    
    # Create optimized MJPEG encoder
    output = StreamingOutput()
    encoder = MJPEGEncoder()
    encoder.output = FileOutput(output)
    
    # MEDIUM quality = better sharpness, still fast
    camera.start_encoder(encoder, quality=Quality.MEDIUM)
    camera.start()
    
    current_resolution = RESOLUTION_BALANCED
    print(f"📷 Camera initialized: {RESOLUTION_BALANCED[0]}x{RESOLUTION_BALANCED[1]} @ {FRAMERATE}fps (BALANCED)")


def generate_frames():
    """Generate MJPEG frames with timeout"""
    while True:
        with output.condition:
            # Short timeout to prevent blocking
            if not output.condition.wait(timeout=0.1):
                continue
            frame = output.frame
        
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n'
                   b'Content-Length: ' + str(len(frame)).encode() + b'\r\n\r\n' 
                   + frame + b'\r\n')


@app.route('/')
def index():
    """Status page with embedded stream"""
    html = '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>KPatrol Camera</title>
        <style>
            body { 
                font-family: Arial, sans-serif; 
                background: #1a1a2e; 
                color: #fff;
                margin: 0;
                padding: 20px;
            }
            h1 { color: #00d4ff; }
            .stream-container {
                max-width: 1280px;
                margin: 20px auto;
            }
            img { 
                width: 100%; 
                border-radius: 10px;
                box-shadow: 0 4px 20px rgba(0,0,0,0.5);
            }
            .info { 
                background: #16213e; 
                padding: 15px; 
                border-radius: 10px;
                margin-top: 20px;
            }
            code { 
                background: #0f3460; 
                padding: 2px 8px; 
                border-radius: 4px;
            }
        </style>
    </head>
    <body>
        <h1>🤖 KPatrol Camera - MJPEG Stream</h1>
        <div class="stream-container">
            <img src="/stream" alt="Camera Stream">
        </div>
        <div class="info">
            <h3>📡 Stream Endpoints:</h3>
            <p><code>GET /stream</code> - MJPEG stream (for img src or video element)</p>
            <p><code>GET /snapshot</code> - Single JPEG frame</p>
            <p><code>GET /status</code> - JSON status</p>
            <h3>🔧 Usage in Web App:</h3>
            <pre>&lt;img src="https://stream.khoavd.online/stream" /&gt;</pre>
        </div>
    </body>
    </html>
    '''
    return render_template_string(html)


@app.route('/stream')
@require_stream_token
def stream():
    """MJPEG video stream - optimized for low latency"""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate, max-age=0',
            'Pragma': 'no-cache',
            'Expires': '0',
            'Access-Control-Allow-Origin': '*',
            'X-Accel-Buffering': 'no',          # Disable nginx buffering
            'Connection': 'keep-alive',
        }
    )


@app.route('/stream/hd')
@require_stream_token
def stream_hd():
    """HD MJPEG stream (higher quality, higher latency)"""
    # Note: This uses the same stream but could be configured differently
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
            'Expires': '0',
            'Access-Control-Allow-Origin': '*',
            'X-Accel-Buffering': 'no',
        }
    )


@app.route('/snapshot')
@require_stream_token
def snapshot():
    """Single JPEG frame"""
    with frame_lock:
        if latest_frame:
            return Response(
                latest_frame,
                mimetype='image/jpeg',
                headers={'Cache-Control': 'no-cache'}
            )
    return jsonify({"error": "No frame available"}), 503


@app.route('/status')
def status():
    """JSON status"""
    global frame_count, start_time, current_resolution
    
    uptime = time.time() - start_time
    fps = frame_count / uptime if uptime > 0 else 0
    
    # Estimate frame size for bandwidth calculation
    avg_frame_size = len(latest_frame) if latest_frame else 0
    bandwidth_kbps = (avg_frame_size * fps * 8) / 1024 if fps > 0 else 0
    
    return jsonify({
        "status": "online",
        "camera": "OV5647",
        "resolution": f"{current_resolution[0]}x{current_resolution[1]}",
        "mode": "balanced",
        "target_fps": FRAMERATE,
        "actual_fps": round(fps, 1),
        "frames_sent": frame_count,
        "uptime_seconds": round(uptime, 1),
        "avg_frame_size_kb": round(avg_frame_size / 1024, 1),
        "bandwidth_kbps": round(bandwidth_kbps, 0),
        "endpoints": {
            "stream": "/stream (balanced: sharp + fast)",
            "stream_hd": "/stream/hd (high quality)",
            "snapshot": "/snapshot",
            "status": "/status"
        }
    })


def get_local_ip():
    """Get local IP"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"


if __name__ == '__main__':
    import sys
    
    print("=" * 50)
    print("🎥 KPatrol MJPEG Camera Server v2 (BALANCED)")
    print("=" * 50)
    
    init_camera()

    if not _STREAM_TOKEN_SECRET:
        print("⚠️  STREAM_TOKEN_SECRET not set — /stream is OPEN. DO NOT deploy like this.")
    elif pyjwt is None:
        print("❌ STREAM_TOKEN_SECRET is set but PyJWT is missing. Install with: pip install PyJWT")
        sys.exit(1)
    else:
        print("🔒 Stream-token auth enabled (HS256, scope=mjpeg)")

    local_ip = get_local_ip()
    print(f"Mode:   BALANCED (960x540, Quality: MEDIUM)")
    print(f"Local:  http://{local_ip}:8080/stream")
    print(f"Public: https://stream.khoavd.online/stream")
    print("=" * 50)
    print("TIP: For best latency, connect to Local IP directly")
    print("Cloudflare adds ~500ms-1s latency")
    print("=" * 50)
    
    # Use threaded mode for better concurrency
    # Set socket options for lower latency
    from werkzeug.serving import WSGIRequestHandler
    WSGIRequestHandler.protocol_version = "HTTP/1.1"
    
    app.run(
        host='0.0.0.0', 
        port=8080, 
        threaded=True, 
        debug=False,
        use_reloader=False
    )
