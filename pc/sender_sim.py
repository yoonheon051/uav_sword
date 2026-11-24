"""
워치 없이 수신기를 시험하는 가짜 워치.
워치 앱과 같은 형식의 패킷을 100Hz로 보낸다.

  - 손목이 좌우로 천천히 흔들리고 위아래로 조금 움직임
  - 8초 주기: 1초 내지르기(발검) -> 2.5초 찌르기 -> 4초 베기 -> 5.5초 당기기(회수)
  - 10초에 워치 버튼 누름(발검), 12초에 다시 누름(회수)
"""

import argparse
import math
import socket
import time


def quat_from_yaw_pitch(yaw, pitch):
    """세계 Z축 yaw 회전 후 (회전된) Y축 기준 pitch. 왼손 워치 +X축이 그 방향을 가리키게 된다."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(-pitch / 2), math.sin(-pitch / 2)
    # q = q_yaw(z) * q_pitch(y)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


def rotate_inv(q, v):
    """세계 좌표 벡터 v를 워치 좌표로 (q의 역회전)."""
    w, x, y, z = q
    x, y, z = -x, -y, -z
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--duration", type=float, default=0, help="N초 후 종료 (0 = 계속)")
    ap.add_argument("--format", choices=["igeo1", "igeo2"], default="igeo2",
                    help="igeo2 = 새 워치 앱(원본 가속도 포함), igeo1 = 옛 워치 앱")
    ap.add_argument("--yaw-offset", type=float, default=40, help="워치의 임의 기준 방향 (도) — 보정 테스트용")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.monotonic()
    seq = 0
    q_prev, t_prev = None, None
    print(f"가짜 워치 → {args.host}:{args.port}")
    while True:
        t = time.monotonic() - t0
        if args.duration and t > args.duration:
            break
        yaw = math.radians(args.yaw_offset) + 0.6 * math.sin(t * 0.8)
        pitch = 0.25 * math.sin(t * 0.5)
        q = quat_from_yaw_pitch(yaw, pitch)

        button = 1 if (t % 16) >= 10 and (t % 16) < 12 else 0
        c = t % 8 if (t % 16) < 8 else -1   # 동작은 16초 중 앞 8초에만

        def burst(start, sign):
            # 손끝(+X) 방향 가속 후 멈추면서 반대 방향 감속 (실제 팔 동작처럼)
            if start <= c < start + 0.08:
                return (25.0 * sign, 0.0, 0.0)
            if start + 0.08 <= c < start + 0.16:
                return (-25.0 * sign, 0.0, 0.0)
            return None

        motion = burst(1.0, +1) or burst(2.5, +1) or burst(5.5, -1) or (0.0, 0.0, 0.0)
        # 자세 변화에서 계산한 실제 각속도(워치 좌표): w = 2 * (q_prev^-1 * q).xyz / dt
        body_w = (0.0, 0.0, 0.0)
        if q_prev is not None and t > t_prev:
            pw, px, py, pz = q_prev[0], -q_prev[1], -q_prev[2], -q_prev[3]
            w, x, y, z = q
            rx = pw * x + px * w + py * z - pz * y
            ry = pw * y - px * z + py * w + pz * x
            rz = pw * z + px * y - py * x + pz * w
            dt = t - t_prev
            body_w = (2 * rx / dt, 2 * ry / dt, 2 * rz / dt)
        q_prev, t_prev = q, t
        slash = (0.0, 0.0, 9.0) if 4.0 <= c < 4.15 else (0.0, 0.0, 0.0)
        gyro = tuple(a + b for a, b in zip(body_w, slash))
        drawn = button

        if args.format == "igeo1":
            # 옛 앱: 기기가 걸러준 선형가속도 (실제 워치4처럼 최대 4 정도)
            lin = tuple(c * 0.16 for c in motion)
            head = "IGEO1"
            tail = ""
        else:
            # 새 앱: 원본 가속도 = 중력(워치 좌표) + 손 움직임
            lin = tuple(c * 0.16 for c in motion)
            g = rotate_inv(q, (0.0, 0.0, 9.81))
            raw = tuple(a + b for a, b in zip(g, motion))
            head = "IGEO2"
            tail = f",{raw[0]:.3f},{raw[1]:.3f},{raw[2]:.3f}"
        msg = (
            f"{head},{seq},{int(t * 1000)},{q[0]:.5f},{q[1]:.5f},{q[2]:.5f},{q[3]:.5f},"
            f"{gyro[0]:.4f},{gyro[1]:.4f},{gyro[2]:.4f},{lin[0]:.3f},{lin[1]:.3f},{lin[2]:.3f},{drawn}"
            + tail
        )
        sock.sendto(msg.encode("ascii"), (args.host, args.port))
        seq += 1
        time.sleep(0.01)


if __name__ == "__main__":
    main()
