#!/usr/bin/env python3
"""无真实设备时, 向 PAUT UDP 端口发送合成帧(冒烟验证用)。

wire 格式与 PAUT_ros2 一致: int32 height + int32 width + height*width 个 int32(行优先, little-endian)。
用法:
    python3 scripts/send_paut_frames.py [port] [rate_hz] [height] [width]
示例:
    python3 scripts/send_paut_frames.py 12345 100
"""

import socket
import struct
import sys
import time


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 12345
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
    height = int(sys.argv[3]) if len(sys.argv) > 3 else 61
    width = int(sys.argv[4]) if len(sys.argv) > 4 else 167
    count = height * width

    base = struct.pack("<ii", height, width)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = ("127.0.0.1", port)
    interval = 1.0 / rate if rate > 0.0 else 0.0
    frame = 0

    print(f"发送合成 PAUT 帧到 {addr[0]}:{addr[1]}  "
          f"尺寸={height}x{width}  rate={rate}Hz  (Ctrl-C 停止)", flush=True)
    try:
        while True:
            # 行优先 int32; 用递增梯度图案便于在接收侧核对数值。
            values = [(i * 7) % 170 for i in range(count)]
            payload = base + struct.pack(f"<{count}i", *values)
            sock.sendto(payload, addr)
            frame += 1
            if frame % 50 == 0:
                print(f"  已发送 {frame} 帧", flush=True)
            if interval > 0.0:
                time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print(f"共发送 {frame} 帧")


if __name__ == "__main__":
    main()
