"""
이기어검 손목 수신기 (외부 라이브러리 필요 없음).

갤럭시 워치에서 오는 UDP 패킷을 받아
  1) 손목이 가리키는 방향을 계산하고
  2) 찌르기/베기 동작을 감지해서
  3) 검(나중엔 드론)의 목표 위치(setpoint)를 만든다.

브라우저로 http://127.0.0.1:8000 을 열면 검이 움직이는 모습을 볼 수 있다.

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
    # 손동작 판정 기준: 손끝 방향 가속도 (m/s^2). 실행할 때 --thrust / --pull 로 바꿀 수 있다.
    # - 새 워치 앱(IGEO2): 원본 가속도계 값을 PC에서 직접 걸러 쓴다. 손목을 툭 치면 20 이상 나온다.
    # - 옛 워치 앱(IGEO1): 기기가 걸러준 선형가속도라 세게 내질러도 4 안팎만 나온다.
    # 새 앱은 가속도가 아니라 '손목 속도'로 판정한다.
    # 팔을 내지르면 출발할 때보다 끝에서 멈출 때 가속(반대 방향)이 더 세게 나와서,
    # 가속도 최대치로 판정하면 내지르기가 '당기기'로 잡힌다. 속도는 내지르는 동안
    # 계속 손끝 방향(+)이라 방향이 헷갈리지 않는다.
    THRUST_VEL = 0.8                 # 찌르기/발검: 손끝 방향 속도 (m/s)
    PULL_VEL = 0.8                   # 회수: 몸 쪽 속도 (m/s)
    VEL_TAU = 0.3                    # 속도 적분이 0으로 서서히 돌아오는 시간상수 (s, 누적 오차 방지)
    THRUST_ACCEL_LIN = 3.0           # 찌르기/발검 (옛 앱)
    PULL_ACCEL_LIN = 3.0             # 회수 (옛 앱)
    GRAVITY_TAU = 1.0                # 중력 추정을 원본 가속도 쪽으로 천천히 되돌리는 시간상수 (s)
    SLASH_GYRO = 7.0                 # 베기 판정: 각속도 크기 (rad/s)
    GESTURE_COOLDOWN = 0.7           # 동작 하나 뒤 다음 동작을 무시하는 시간 (s)
                                     # 팔을 내지르면 멈출 때 반대 방향 가속이 생기는데,
                                     # 그걸 '회수'로 착각하지 않게 막는다.
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
        self.button_prev = None      # 워치 발검/회수 버튼 값 (바뀔 때마다 상태 토글)
        self.thrust_until = 0.0
        self.gesture_block_until = 0.0
        self.events = []             # (id, time, name)
        self.event_count = 0
        self.last_rx = 0.0
        self.fwd_hist = []           # (time, 손끝 방향 가속도) 최근 1초 — 감도 조정용
        self.thrust_override = None  # --thrust 로 준 값
        self.pull_override = None
        self.thrust_accel = self.THRUST_ACCEL_LIN   # 지금 쓰는 기준 (패킷 종류에 따라)
        self.pull_accel = self.PULL_ACCEL_LIN
        self.raw_mode = False
        self.grav_dev = None         # 워치 좌표계에서 본 중력 방향 추정 (자이로로 추적)
        self.last_t_ms = None
        self.vel = 0.0               # 손끝 방향 속도 추정 (m/s)

    def _event(self, now, name):
        self.event_count += 1
        self.events.append((self.event_count, now, name))
        self.events = self.events[-20:]

    def _set_drawn(self, now, drawn, how):
        if drawn == self.drawn:
            return
        self.drawn = drawn
        self.thrust_until = 0.0
        self.gesture_block_until = now + self.GESTURE_COOLDOWN
        self._event(now, f"{'발검' if drawn else '회수'} ({how})")

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
            if sum(c * c for c in q) < 0.5:
                return  # 자세 값이 아직 없음
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

            # 워치 버튼: 누를 때마다(값이 바뀔 때마다) 발검 <-> 회수 토글.
            # 손동작으로도 상태가 바뀌므로 버튼 글자와 실제 상태가 어긋날 수 있지만,
            # 버튼을 누르면 항상 '반대 상태'로 바뀐다.
            button = bool(pkt["drawn"])
            if self.button_prev is None:
                self.drawn = button
            elif button != self.button_prev:
                self._set_drawn(now, not self.drawn, "버튼")
            self.button_prev = button

            # 동작 감지 (세계 좌표로 바꿔서 손끝 방향 성분을 본다)
            raw = pkt.get("raw")
            self.raw_mode = raw is not None
            if raw is not None:
                # 워치의 자세 값(q)은 빠르게 내지를 때 가속을 '기울어짐'으로 착각해서
                # 그걸로 중력을 빼면 손 움직임까지 지워진다. 그래서 q를 쓰지 않고,
                # 워치 좌표계에서 중력 방향을 자이로로 직접 돌려가며 추적한다.
                #   중력(워치 좌표) 변화: dg/dt = -w x g   (w = 자이로)
                # 그리고 아주 천천히(GRAVITY_TAU) 원본 가속도 쪽으로 보정해 오차가 쌓이지 않게 한다.
                dt = 0.01
                if self.last_t_ms is not None:
                    dt = min(max((pkt["t_ms"] - self.last_t_ms) / 1000.0, 0.001), 0.1)
                self.last_t_ms = pkt["t_ms"]
                g = self.grav_dev if self.grav_dev is not None else raw
                w = pkt["gyro"]
                wxg = (w[1] * g[2] - w[2] * g[1], w[2] * g[0] - w[0] * g[2], w[0] * g[1] - w[1] * g[0])
                g = tuple(gc - dt * c for gc, c in zip(g, wxg))
                g = lerp(g, raw, dt / (self.GRAVITY_TAU + dt))
                self.grav_dev = g
                lin_dev = tuple(a - b for a, b in zip(raw, g))
                # 손끝 방향 가속 = 워치 좌표의 팔뚝 축 성분 (자세 값과 무관)
                forward_accel = dot(lin_dev, self.forearm_axis)
                # 가속도를 적분해 속도로 (오래된 값은 VEL_TAU로 서서히 잊음)
                self.vel = self.vel * math.exp(-dt / self.VEL_TAU) + forward_accel * dt
                metric = self.vel
                base_t, base_p = self.THRUST_VEL, self.PULL_VEL
            else:
                lin_world = rotate_z(quat_rotate(q, pkt["lin"]), -self.yaw0)
                forward_accel = dot(lin_world, d)
                metric = forward_accel
                base_t, base_p = self.THRUST_ACCEL_LIN, self.PULL_ACCEL_LIN
            self.thrust_accel = self.thrust_override if self.thrust_override is not None else base_t
            self.pull_accel = self.pull_override if self.pull_override is not None else base_p
            # metric  +: 손끝 쪽으로 내지름, -: 몸 쪽으로 당김
            #         (새 앱: 속도 m/s, 옛 앱: 가속도 m/s^2)
            self.fwd_hist.append((now, metric))
            while self.fwd_hist and now - self.fwd_hist[0][0] > 1.0:
                self.fwd_hist.pop(0)
            gyro_mag = norm(pkt["gyro"])

            if now > self.gesture_block_until:
                if not self.drawn and metric > self.thrust_accel:
                    # 회수 상태에서 내지르기 -> 발검
                    self._set_drawn(now, True, "내지르기")
                elif self.drawn and metric > self.thrust_accel:
                    self.thrust_until = now + self.THRUST_TIME
                    self.gesture_block_until = self.thrust_until + 0.2
                    self._event(now, "찌르기")
                elif self.drawn and metric < -self.pull_accel:
                    # 발검 상태에서 손목을 몸 쪽으로 당기기 -> 회수
                    self._set_drawn(now, False, "당기기")
                elif self.drawn and gyro_mag > self.SLASH_GYRO:
                    self.gesture_block_until = now + 0.4
                    self._event(now, "베기")

            # 목표 위치 계산
            drawn = self.drawn
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
                # 최근 1초 동안 손끝 방향 가속도의 최대(+, 내지르기)와 최소(-, 당기기)
                "fwd_max": round(max((a for _, a in self.fwd_hist), default=0.0), 2),
                "fwd_min": round(min((a for _, a in self.fwd_hist), default=0.0), 2),
                "metric": "속도" if self.raw_mode else "가속",
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
    """IGEO1(옛 워치 앱, 14칸)과 IGEO2(원본 가속도 추가, 17칸) 둘 다 받는다."""
    parts = text.strip().split(",")
    if not ((parts[0] == "IGEO1" and len(parts) == 14) or (parts[0] == "IGEO2" and len(parts) == 17)):
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
        "raw": tuple(f[13:16]) if len(f) >= 16 else None,
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
    ap.add_argument("--thrust", type=float, default=None,
                    help="내지르기(발검·찌르기) 판정 기준 (낮을수록 잘 걸림). "
                         "새 워치 앱: 손목 속도 m/s, 기본 0.8 / 옛 앱: 가속도 m/s^2, 기본 3")
    ap.add_argument("--pull", type=float, default=None,
                    help="당기기(회수) 판정 기준 (낮을수록 잘 걸림). 새 앱: 속도 m/s 기본 0.8 / 옛 앱: 가속도 기본 3")
    ap.add_argument("--no-web", action="store_true", help="브라우저 화면 없이 콘솔만")
    ap.add_argument("--duration", type=float, default=0, help="N초 후 자동 종료 (테스트용)")
    args = ap.parse_args()

    # 콘솔 인코딩(cp949 등)에 없는 문자가 있어도 죽지 않게
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    state = SwordState(hand=args.hand)
    state.thrust_override = args.thrust
    state.pull_override = args.pull
    stop = threading.Event()
    threading.Thread(target=udp_loop, args=(state, args.port, stop), daemon=True).start()
    print(f"UDP {args.port} 수신 대기 중 (워치: {args.hand}손)")

    if not args.no_web:
        server = ThreadingHTTPServer(("127.0.0.1", args.web_port), make_handler(state))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"화면: http://127.0.0.1:{args.web_port}")

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
                f"목표 ({sp['x']:5.2f}, {sp['y']:5.2f}, {sp['z']:5.2f})  "
                f"{s['metric']} +{s['fwd_max']:4.2f}/{s['fwd_min']:5.2f} (기준 +{state.thrust_accel:.2f}/-{state.pull_accel:.2f}, {'원본' if state.raw_mode else '옛 앱'})"
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
