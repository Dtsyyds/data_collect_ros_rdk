#!/usr/bin/env python3
"""合成 PAUT :12346 报文，用于在没有真实设备时验证接收端。

配对工具: scripts/recv_paut_12346.py
线格式:   设备端 Demo/Src/Common/PautRawBroadcaster.cs (C#)

用途:
    在联调前先把接收链路验证掉。这样等真机接入时若收不到数据，可以确定问题在设备侧，
    而不是接收脚本。

用法:
    # 本机自测（两个终端）
    python3 scripts/recv_paut_12346.py --duration 10 --save t.npz
    python3 scripts/send_paut_12346_synth.py

    # 发到另一台机器
    python3 scripts/send_paut_12346_synth.py --host 192.168.1.50

    # 改规模 / 帧率
    python3 scripts/send_paut_12346_synth.py --beams 61 --samples 896 --fps 40 --frames 200

    # 故意造错，验证接收端的检出能力
    python3 scripts/send_paut_12346_synth.py --break-crc
    python3 scripts/send_paut_12346_synth.py --bad-sat-count
"""

import argparse
import socket
import struct
import sys
import time
import zlib

MAGIC = 0x54554150
TYPE_CONFIG, TYPE_FRAME, TYPE_HEARTBEAT = 1, 2, 3
ENVELOPE_LEN = 24
FRAME_HEADER_LEN = 72
CONFIG_HEADER_LEN = 140
HEARTBEAT_LEN = 32


def make_envelope(ptype, header_len, packet_len):
    b = bytearray(packet_len)
    struct.pack_into("<IHHBBHII", b, 0, MAGIC, 1, 0, ptype, 0, header_len, packet_len, 0)
    return b


def seal(b, packet_len, break_crc=False):
    """按设计约定算 CRC：覆盖 [24, packet_len)。"""
    crc = zlib.crc32(bytes(b[ENVELOPE_LEN:packet_len])) & 0xFFFFFFFF
    if break_crc:
        crc ^= 0xDEADBEEF
    struct.pack_into("<I", b, 16, crc)
    return bytes(b)


def build_config(seq, beams, samples, vel_mismatch=False):
    names = ["5L64-0.6x10", "SD3-N0L-IHC",
             "SYNTHETIC SCRIPTS/send_paut_12346_synth.py, 非真实设备"]
    packet_len = CONFIG_HEADER_LEN + sum(2 + len(n.encode()) for n in names)
    b = make_envelope(TYPE_CONFIG, CONFIG_HEADER_LEN, packet_len)

    range_ns = 100000
    fs_hz = samples / (range_ns * 1e-9)
    fields = [
        (24, "I", 0x8201A8C0),      # board_ip（.NET IPAddress.Address 形式）-> 192.168.1.130
        (28, "I", samples),         # sample_point_num
        (32, "I", range_ns),        # range_ns
        (36, "H", 400),             # ut_voltage_v
        (38, "H", 80),              # pulse_width_ns
        (40, "H", 5246),            # prf_hz
        (42, "H", 0),               # start_ns_raw（累积漂移，不可信）
        (44, "B", 3),               # board_type = Robust3264Tofd
        (45, "B", 4),               # filter_level
        (48, "f", 0.6),             # element_pitch_mm
        (52, "f", 5.0),             # frequency_mhz
        (56, "H", 64),              # element_total
        (58, "B", 0),               # probe_type
        (60, "f", 0.0),             # wedge_angle_deg
        (64, "f", 2337.0),          # wedge_velocity_m_s
        (68, "f", 20.0),            # first_ele_height_mm
        (72, "f", 49.9),            # primary_offset_mm
        (76, "f", 20.0),            # img_height_mm
        (80, "f", 2.5),             # img_height_bias_mm
        (84, "f", 0.0),             # focus_angle_deg
        (88, "f", 15.0),            # focus_depth_mm
        (92, "f", 5900.0),          # specimen_vel_m_s（成像实际用的）
        (96, "f", 5918.0 if vel_mismatch else 5900.0),  # board_vel_m_s
        (100, "H", 4),              # little_aperture
        (102, "H", 1),              # beam_step
        (104, "H", beams),          # beam_count
        (106, "B", 0),              # scan_mode = 线扫
        (108, "f", fs_hz),          # sampling_rate_hz
        (112, "B", 1),              # sampling_rate_src = derived_unverified
        (113, "B", 0),              # sample_dtype = u8
        (114, "B", 0),              # adc_bits = 未知
        (116, "f", 67.0),           # enc1_resolution
        (120, "f", 67.0),           # enc2_resolution
        (124, "B", 0),              # enc1_unit = unknown
        (125, "b", 1),              # enc1_polarity
        (126, "B", 0),              # enc1_mode
        (127, "B", 0),              # enc2_unit
        (128, "b", 1),              # enc2_polarity
        (129, "B", 0),              # enc2_mode
        (132, "I", seq),            # config_seq
    ]
    for rel, fmt, v in fields:
        struct.pack_into("<" + fmt, b, rel, v)

    off = CONFIG_HEADER_LEN
    for n in names:
        e = n.encode()
        struct.pack_into("<H", b, off, len(e))
        b[off + 2:off + 2 + len(e)] = e
        off += 2 + len(e)
    assert off == packet_len, (off, packet_len)
    return seal(b, packet_len)


def build_frame(seq, beams, samples, enc_scan, enc_index,
                break_crc=False, bad_sat_count=False):
    """合成一帧。波形仿 A 扫：前端回波饱和 / 安静区 / 底波饱和。"""
    packet_len = FRAME_HEADER_LEN + beams * samples
    b = make_envelope(TYPE_FRAME, FRAME_HEADER_LEN, packet_len)

    front_end = samples // 12
    back_start = int(samples * 0.88)
    wave = bytearray(samples)
    for j in range(samples):
        if j < front_end or j >= back_start:
            wave[j] = 255                    # 界面/底波强回波 -> 饱和
        elif front_end <= j < front_end + 8:
            wave[j] = (j * 13) % 200 + 20
        else:
            wave[j] = (j * 7) % 12           # 噪声底

    payload = bytes(wave) * beams            # 61 条波束近似相同（静置平板）

    sat = payload.count(255)
    if bad_sat_count:
        sat = 99999                          # 故意不符，验证接收端的交叉校验

    flags = 1 | (1 << 1) | (1 << 7)          # has_data | saturated | sample_dtype_u8
    if enc_scan or enc_index:
        flags |= (1 << 3)                    # encoder_valid

    struct.pack_into("<QQiiHHHHIHHII", b, ENVELOPE_LEN,
                     seq,
                     0,                      # device_timestamp_ns（设备无硬件时戳，恒 0）
                     enc_scan, enc_index,
                     beams, samples, 64,
                     flags, sat,
                     0, 1,                   # frag_index, frag_count
                     1,                      # config_seq
                     0)
    b[FRAME_HEADER_LEN:] = payload
    return seal(b, packet_len, break_crc)


def build_heartbeat(counter):
    b = make_envelope(TYPE_HEARTBEAT, HEARTBEAT_LEN, HEARTBEAT_LEN)
    struct.pack_into("<Q", b, ENVELOPE_LEN, counter)
    return seal(b, HEARTBEAT_LEN)


def main():
    ap = argparse.ArgumentParser(description="合成 PAUT :12346 报文（验证接收端用）")
    ap.add_argument("--host", default="127.0.0.1", help="目标地址，默认本机")
    ap.add_argument("--port", type=int, default=12346)
    ap.add_argument("--beams", type=int, default=61, help="波束数，默认 61")
    ap.add_argument("--samples", type=int, default=896, help="每波束采样点数，默认 896")
    ap.add_argument("--fps", type=float, default=40.0, help="帧率，默认 40")
    ap.add_argument("--frames", type=int, default=0, help="总帧数，0 = 一直发到 Ctrl-C")
    ap.add_argument("--break-crc", action="store_true",
                    help="故意写错 CRC（验证接收端能检出）")
    ap.add_argument("--bad-sat-count", action="store_true",
                    help="故意写错 saturated_count（验证交叉校验）")
    ap.add_argument("--vel-mismatch", action="store_true",
                    help="让 board_vel 与 specimen_vel 不一致（验证提示）")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dst = (args.host, args.port)

    pkt_bytes = FRAME_HEADER_LEN + args.beams * args.samples
    print("目标 %s:%d   %d 波束 x %d 采样点   %d B/帧   %.0f fps  ≈ %.2f Mbit/s"
          % (args.host, args.port, args.beams, args.samples, pkt_bytes, args.fps,
             pkt_bytes * 8 * args.fps / 1e6))
    if pkt_bytes > 60000:
        print("⚠ 单帧 %d B 超过设备端的 60000 B 上限；真实设备在该配置下不会发包"
              % pkt_bytes)

    cfg = build_config(1, args.beams, args.samples, args.vel_mismatch)
    sock.sendto(cfg, dst)
    print("已发 CONFIG (%d B, config_seq=1)" % len(cfg))

    period = 1.0 / args.fps if args.fps > 0 else 0.05
    t0 = time.time()
    sent = 0
    hb_next = time.time() + 1.0
    try:
        while args.frames == 0 or sent < args.frames:
            now = time.time()
            n = sent + 1
            pkt = build_frame(n, args.beams, args.samples,
                              enc_scan=(n // 10) * 67, enc_index=0,
                              break_crc=args.break_crc,
                              bad_sat_count=args.bad_sat_count)
            sock.sendto(pkt, dst)
            sent += 1

            if now >= hb_next:
                sock.sendto(build_heartbeat(sent), dst)
                hb_next = now + 1.0

            # 每 2s 重发 CONFIG（与设备端行为一致）
            if args.fps > 0 and n % max(1, int(args.fps * 2)) == 0:
                sock.sendto(cfg, dst)

            if sent % 40 == 0:
                print("  已发 %d 帧  (%.1f s, 实际 %.1f fps)"
                      % (sent, now - t0, sent / max(1e-6, now - t0)))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n中断。")
    finally:
        sock.close()
    print("共发 %d 帧" % sent)
    return 0


if __name__ == "__main__":
    sys.exit(main())
