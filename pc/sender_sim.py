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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--duration", type=float, default=0, help="N초 후 종료 (0 = 계속)")
    ap.add_argument("--yaw-offset", type=float, default=40, help="워치의 임의 기준 방향 (도) — 보정 테스트용")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.monotonic()
    seq = 0
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

        lin = burst(1.0, +1) or burst(2.5, +1) or burst(5.5, -1) or (0.0, 0.0, 0.0)
        gyro = (0.0, 0.0, 9.0) if 4.0 <= c < 4.15 else (0.0, 0.0, 0.0)
        drawn = button

        msg = (
            f"IGEO1,{seq},{int(t * 1000)},{q[0]:.5f},{q[1]:.5f},{q[2]:.5f},{q[3]:.5f},"
            f"{gyro[0]:.4f},{gyro[1]:.4f},{gyro[2]:.4f},{lin[0]:.3f},{lin[1]:.3f},{lin[2]:.3f},{drawn}"
        )
        sock.sendto(msg.encode("ascii"), (args.host, args.port))
        seq += 1
        time.sleep(0.01)


if __name__ == "__main__":
    main()
