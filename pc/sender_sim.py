"""
워치 없이 수신기를 시험하는 가짜 워치.
워치 앱과 같은 형식의 패킷을 100Hz로 보낸다.

  - 손목이 좌우로 천천히 흔들리고 위아래로 조금 움직임
  - 2초 후 발검, 4초마다 찌르기, 6초마다 베기
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

        drawn = 1 if t > 2 else 0
        # 찌르기: 4초마다 0.1초 동안 손끝 방향(+X, 워치 좌표)으로 큰 가속
        lin = (25.0, 0.0, 0.0) if drawn and (t % 4) < 0.1 else (0.0, 0.0, 0.0)
        # 베기: 6초마다 0.15초 동안 빠른 회전 (찌르기와 겹치지 않는 시점)
        gyro = (0.0, 0.0, 9.0) if drawn and 3.0 < (t % 6) < 3.15 else (0.0, 0.0, 0.0)

        msg = (
            f"IGEO1,{seq},{int(t * 1000)},{q[0]:.5f},{q[1]:.5f},{q[2]:.5f},{q[3]:.5f},"
            f"{gyro[0]:.4f},{gyro[1]:.4f},{gyro[2]:.4f},{lin[0]:.3f},{lin[1]:.3f},{lin[2]:.3f},{drawn}"
        )
        sock.sendto(msg.encode("ascii"), (args.host, args.port))
        seq += 1
        time.sleep(0.01)


if __name__ == "__main__":
    main()
