"""
이기어검 손목 수신기 (외부 라이브러리 필요 없음).

갤럭시 워치에서 오는 UDP 패킷을 받아
  1) 손목이 가리키는 방향을 계산하고
  2) 찌르기/베기 동작을 감지해서
  3) 검(나중엔 드론)의 목표 위치(setpoint)를 만든다.

브라우저로 http://localhost:8000 을 열면 검이 움직이는 모습을 볼 수 있다.

사용:
    python receiver.py                 # 왼손에 워치
    python receiver.py --hand right    # 오른손에 워치
    python receiver.py --no-web        # 콘솔 출력만
"""

import argparse
import json
import math
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# ------------------------------------------------------------------ 벡터 수학


def quat_rotate(q, v):
    """쿼터니언 q=(w,x,y,z)로 벡터 v를 회전 (워치 좌표 -> 세계 좌표)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    # v' = v + w*t + cross(q.xyz, t)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def rotate_z(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def lerp(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


# ------------------------------------------------------------------ 검 상태


class SwordState:
    """
    좌표계 (보정 후): +X = 사용자 정면, +Y = 왼쪽, +Z = 위.  단위 m.
    원점 = 사용자 발밑.
    """

    SHOULDER = (0.0, 0.0, 1.4)       # 검이 뻗어 나가는 기준점
    HOME = (0.3, -0.5, 1.0)          # 회수 시 검이 머무는 곳 (오른쪽 허리 옆)
    REACH = 1.5                      # 평소 손끝에서 검까지 거리
    THRUST_REACH = 3.0               # 찌르기 때 거리
    THRUST_TIME = 0.5                # 찌르기 지속 시간 (s)
    THRUST_ACCEL = 12.0              # 찌르기 판정: 정면 방향 가속도 (m/s^2)
    SLASH_GYRO = 7.0                 # 베기 판정: 각속도 크기 (rad/s)
    SMOOTH = 0.25                    # 목표 위치 저역통과 계수 (0~1, 작을수록 부드러움)

    def __init__(self, hand="left"):
        # 워치 좌표계에서 "손끝 방향" 축.
        # 왼손에 차면 3시 방향(+X)이 손끝, 오른손에 차면 9시 방향(-X)이 손끝.
        self.forearm_axis = (1.0, 0.0, 0.0) if hand == "left" else (-1.0, 0.0, 0.0)
        self.lock = threading.Lock()
        self.yaw0 = None
        self.calibrate_next = True
        self.last_packet = None
        self.last_seq = None
        self.lost = 0
        self.received = 0
        self.rate_hz = 0.0
        self._rate_count = 0
        self._rate_t0 = time.monotonic()
        self.direction = (1.0, 0.0, 0.0)
        self.target = self.HOME
        self.drawn = False
        self.thrust_until = 0.0
        self.events = []             # (id, time, name)
        self.event_count = 0
        self.last_rx = 0.0

    def _event(self, now, name):
        self.event_count += 1
        self.events.append((self.event_count, now, name))
        self.events = self.events[-20:]

    def request_calibration(self):
        with self.lock:
            self.calibrate_next = True

    def update(self, pkt):
        now = time.monotonic()
        with self.lock:
            self.received += 1
            self.last_rx = now
            self._rate_count += 1
            if now - self._rate_t0 >= 1.0:
                self.rate_hz = self._rate_count / (now - self._rate_t0)
                self._rate_count = 0
                self._rate_t0 = now

            if self.last_seq is not None and pkt["seq"] > self.last_seq + 1:
                self.lost += pkt["seq"] - self.last_seq - 1
            if self.last_seq is not None and pkt["seq"] < self.last_seq:
                # 워치 앱을 다시 시작함
                self.lost = 0
            self.last_seq = pkt["seq"]
            self.last_packet = pkt

            q = pkt["q"]
            d_world = quat_rotate(q, self.forearm_axis)

            if self.calibrate_next:
                # 지금 손이 가리키는 수평 방향을 '정면(+X)'으로 정한다.
                self.yaw0 = math.atan2(d_world[1], d_world[0])
                self.calibrate_next = False
                self._event(now, "정면 보정")

            d = rotate_z(d_world, -self.yaw0)
            n = norm(d) or 1.0
            d = tuple(c / n for c in d)
            self.direction = d

            # 발검/회수 상태 변화
            drawn = bool(pkt["drawn"])
            if drawn != self.drawn:
                self._event(now, "발검" if drawn else "회수")
                self.drawn = drawn

            # 동작 감지 (세계 좌표로 바꿔서 정면 성분을 본다)
            lin_world = rotate_z(quat_rotate(q, pkt["lin"]), -self.yaw0)
            forward_accel = dot(lin_world, d)
            gyro_mag = norm(pkt["gyro"])

            if drawn and forward_accel > self.THRUST_ACCEL and now > self.thrust_until:
                self.thrust_until = now + self.THRUST_TIME
                self._event(now, "찌르기")
            elif drawn and gyro_mag > self.SLASH_GYRO and now > self.thrust_until:
                if not self.events or self.events[-1][2] != "베기" or now - self.events[-1][1] > 0.4:
                    self._event(now, "베기")

            # 목표 위치 계산
            if drawn:
                reach = self.THRUST_REACH if now < self.thrust_until else self.REACH
                raw = tuple(s + c * reach for s, c in zip(self.SHOULDER, d))
                # 바닥 아래로는 못 가게
                raw = (raw[0], raw[1], max(raw[2], 0.3))
            else:
                raw = self.HOME
            self.target = lerp(self.target, raw, self.SMOOTH)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, self.direction[2]))))
            yaw = math.degrees(math.atan2(self.direction[1], self.direction[0]))
            return {
                "connected": now - self.last_rx < 1.0,
                "rate_hz": round(self.rate_hz, 1),
                "received": self.received,
                "lost": self.lost,
                "drawn": self.drawn,
                "thrusting": now < self.thrust_until,
                "direction": [round(c, 4) for c in self.direction],
                "yaw_deg": round(yaw, 1),
                "pitch_deg": round(pitch, 1),
                # 드론에 보낼 목표점: x, y, z (m) 와 검끝 방향(yaw)
                "setpoint": {
                    "x": round(self.target[0], 3),
                    "y": round(self.target[1], 3),
                    "z": round(self.target[2], 3),
                    "yaw_deg": round(yaw, 1),
                },
                "events": [
                    {"id": i, "ago": round(now - t, 2), "name": n} for i, t, n in self.events[-6:]
                ],
            }


# ------------------------------------------------------------------ UDP 수신


def parse_packet(text):
    parts = text.strip().split(",")
    if len(parts) != 14 or parts[0] != "IGEO1":
        return None
    try:
        f = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    return {
        "seq": int(f[0]),
        "t_ms": int(f[1]),
        "q": tuple(f[2:6]),
        "gyro": tuple(f[6:9]),
        "lin": tuple(f[9:12]),
        "drawn": int(f[12]),
    }


def udp_loop(state, port, stop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    while not stop.is_set():
        try:
            data, _addr = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            break
        pkt = parse_packet(data.decode("ascii", errors="ignore"))
        if pkt:
            state.update(pkt)
    sock.close()


# ------------------------------------------------------------------ 웹 화면

PAGE = (Path(__file__).parent / "viewer.html").read_text(encoding="utf-8")


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/state":
                body = json.dumps(state.snapshot(), ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path == "/calibrate":
                state.request_calibration()
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

    return Handler


# ------------------------------------------------------------------ main


def main():
    ap = argparse.ArgumentParser(description="이기어검 손목 수신기")
    ap.add_argument("--port", type=int, default=5005, help="UDP 포트 (워치 앱과 같아야 함)")
    ap.add_argument("--web-port", type=int, default=8000)
    ap.add_argument("--hand", choices=["left", "right"], default="left", help="워치를 찬 손")
    ap.add_argument("--no-web", action="store_true", help="브라우저 화면 없이 콘솔만")
    ap.add_argument("--duration", type=float, default=0, help="N초 후 자동 종료 (테스트용)")
    args = ap.parse_args()

    # 콘솔 인코딩(cp949 등)에 없는 문자가 있어도 죽지 않게
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    state = SwordState(hand=args.hand)
    stop = threading.Event()
    threading.Thread(target=udp_loop, args=(state, args.port, stop), daemon=True).start()
    print(f"UDP {args.port} 수신 대기 중 (워치: {args.hand}손)")

    if not args.no_web:
        server = ThreadingHTTPServer(("127.0.0.1", args.web_port), make_handler(state))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"화면: http://localhost:{args.web_port}")

    if sys.stdin and sys.stdin.isatty():
        def stdin_loop():
            for line in sys.stdin:
                if line.strip().lower() in ("", "c"):
                    state.request_calibration()
                    print(">> 다음 패킷에서 정면 보정")
        threading.Thread(target=stdin_loop, daemon=True).start()
        print("Enter = 지금 손 방향을 정면으로 보정, Ctrl+C = 종료")

    t_end = time.monotonic() + args.duration if args.duration else None
    last_event_id = 0
    try:
        while not (t_end and time.monotonic() > t_end):
            time.sleep(0.2)
            s = state.snapshot()
            sp = s["setpoint"]
            conn = "연결" if s["connected"] else "끊김"
            print(
                f"[{conn} {s['rate_hz']:5.1f}Hz 손실 {s['lost']}] "
                f"{'발검' if s['drawn'] else '회수'}  "
                f"방향 yaw {s['yaw_deg']:6.1f}° pitch {s['pitch_deg']:5.1f}°  "
                f"목표 ({sp['x']:5.2f}, {sp['y']:5.2f}, {sp['z']:5.2f})"
                + ("  << 찌르기" if s["thrusting"] else "")
            )
            with state.lock:
                new = [(i, n) for i, _t, n in state.events if i > last_event_id]
            for i, name in new:
                print(f"   >> {name}")
                last_event_id = i
    except KeyboardInterrupt:
        pass
    stop.set()


if __name__ == "__main__":
    main()
