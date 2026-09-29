#!/usr/bin/env python3
"""接收并解析 PAUT :12346 原始数据外发报文。

对端实现: 设备端 Demo/Src/Common/PautRawBroadcaster.cs (C#)
线格式文档: docs/PAUT_12346_WIRE_FORMAT_ZH.md
全包小端, 紧凑排列(无填充字节), 消费端必须按显式偏移读取。

三类包 (由封装段 packet_type 判别):
    CONFIG(1)     启动 + 配置变更时 + 每 2s 重发   低频元数据
    FRAME(2)      每整帧                           帧块 + 原始数据段
    HEARTBEAT(3)  每 1s                            存活心跳

与 :12345 的区别:
    12345 发的是成像算法裁剪过的 61x167 int32 图像 (UI 线程, 有损)
    12346 发的是完整 61x896 uint8 原始 A 扫 (接收线程, 保真)

用法:
    python3 scripts/recv_paut_12346.py                    # 持续接收, Ctrl-C 结束
    python3 scripts/recv_paut_12346.py --duration 20      # 采 20 秒后自动退出
    python3 scripts/recv_paut_12346.py --save out.npz     # 另存帧数据
    python3 scripts/recv_paut_12346.py -v                 # 每帧都打印一行
    python3 scripts/recv_paut_12346.py --bind 0.0.0.0 --port 12346

前置条件:
    设备端与采集主机同一网段。设备默认向 255.255.255.255:12346 广播,
    故本脚本绑 0.0.0.0 即可收到。若设备改成单播, 需绑对应网卡地址。

退出码:
    0 = 全部报文魔数与 CRC 均通过; 1 = 有失败(便于脚本化判断)
"""

import argparse
import socket
import struct
import sys
import time
import zlib

try:
    import numpy as np
except ImportError:
    np = None

# ================================================================ 线格式常量

MAGIC = 0x54554150  # "PAUT" 小端

TYPE_CONFIG = 1
TYPE_FRAME = 2
TYPE_HEARTBEAT = 3

TYPE_NAME = {TYPE_CONFIG: "CONFIG", TYPE_FRAME: "FRAME", TYPE_HEARTBEAT: "HEARTBEAT"}

ENVELOPE_LEN = 24
FRAME_HEADER_LEN = 72     # 封装 24 + 帧块 48；FRAME 数据段起点
CONFIG_HEADER_LEN = 140   # CONFIG 变长区起点
HEARTBEAT_LEN = 32

# 封装段 (自偏移 0): magic u32, verMajor u16, verMinor u16, type u8, flags u8,
#                  headerLen u16, packetLen u32, crc32 u32, reserved u32
ENV_FMT = "<IHHBBHII"
ENV_SIZE = struct.calcsize(ENV_FMT)      # 20（不含末尾 4 字节 reserved）
ENV_OFF_CRC = 16
ENV_OFF_PACKET_LEN = 12

# 帧块 (自偏移 24)
FRAME_FMT = "<QQiiHHHHIHHII"
FRAME_FIELDS = (
    "frame_seq", "device_timestamp_ns", "scan_axis_encoder", "index_axis_encoder",
    "beam_count", "sample_count", "element_total", "status_flags",
    "saturated_count", "frag_index", "frag_count", "config_seq", "reserved",
)
FRAME_SIZE = struct.calcsize(FRAME_FMT)  # 48
FRAME_OFF_BEAM_COUNT = ENVELOPE_LEN + 24  # = 48

# status_flags 位定义
STATUS_BITS = {
    0: "has_data", 1: "saturated", 2: "no_sync", 3: "encoder_valid",
    4: "is_fragmented", 5: "calibrated", 6: "start_ns_trusted", 7: "sample_dtype_u8",
}

# CONFIG 固定字段区。(名字, 绝对偏移自包起始, struct 格式)
# 注意：偏移是绝对的（与 C# 侧 PautWire.OffCfg* 一致），不是相对封装段的。
# 例如 OffCfgBoardIp=24 紧接在 24 字节封装之后。
CONFIG_FIELDS = [
    # 板卡段 [24,48)
    ("board_ip",            24, "I"),
    ("sample_point_num",    28, "I"),
    ("range_ns",            32, "I"),
    ("ut_voltage_v",        36, "H"),
    ("pulse_width_ns",      38, "H"),
    ("prf_hz",              40, "H"),
    ("start_ns_raw",        42, "H"),
    ("board_type",          44, "B"),
    ("filter_level",        45, "B"),
    # 探头段 [48,60)
    ("element_pitch_mm",    48, "f"),
    ("frequency_mhz",       52, "f"),
    ("element_total",       56, "H"),
    ("probe_type",          58, "B"),
    # 楔块段 [60,76)
    ("wedge_angle_deg",     60, "f"),
    ("wedge_velocity_m_s",  64, "f"),
    ("first_ele_height_mm", 68, "f"),
    ("primary_offset_mm",   72, "f"),
    # 扫查段 [76,108)
    ("img_height_mm",       76, "f"),
    ("img_height_bias_mm",  80, "f"),
    ("focus_angle_deg",     84, "f"),
    ("focus_depth_mm",      88, "f"),
    ("specimen_vel_m_s",    92, "f"),
    ("board_vel_m_s",       96, "f"),
    ("little_aperture",    100, "H"),
    ("beam_step",          102, "H"),
    ("beam_count",         104, "H"),
    ("scan_mode",          106, "B"),
    # 时间基准 [108,116)
    ("sampling_rate_hz",   108, "f"),
    ("sampling_rate_src",  112, "B"),
    ("sample_dtype",       113, "B"),
    ("adc_bits",           114, "B"),
    # 位置基准 [116,132)
    ("enc1_resolution",    116, "f"),
    ("enc2_resolution",    120, "f"),
    ("enc1_unit",          124, "B"),
    ("enc1_polarity",      125, "b"),
    ("enc1_mode",          126, "B"),
    ("enc2_unit",          127, "B"),
    ("enc2_polarity",      128, "b"),
    ("enc2_mode",          129, "B"),
    # 尾部 [132,140)
    ("config_seq",         132, "I"),
]

ENCODER_UNIT = {0: "unknown", 1: "count/mm", 2: "mm/count"}
RATE_SRC = {0: "unknown", 1: "derived_unverified"}
SAMPLE_DTYPE = {0: "u8", 1: "i16", 2: "u16", 3: "i32", 4: "f32"}
SCAN_MODE = {0: "线扫", 1: "扇扫", 2: "TOFM", 3: "TOFD", 4: "PWI"}
BOARD_TYPE = {0: "Robust3264", 1: "Robust32128", 2: "Robust32256", 3: "Robust3264Tofd",
              4: "Robust32128Tofd", 5: "Multiscan", 6: "Robust64128",
              7: "Robust64128Tofd", 8: "MultiscanPar"}


# ================================================================ 解析

def parse_envelope(buf):
    """解析封装段。魔数不符或长度不足返回 None。"""
    if len(buf) < ENVELOPE_LEN:
        return None
    magic, ver_m, ver_n, ptype, flags, hlen, plen, crc = struct.unpack_from(ENV_FMT, buf, 0)
    if magic != MAGIC:
        return None
    return dict(ver_major=ver_m, ver_minor=ver_n, packet_type=ptype, flags=flags,
                header_len=hlen, packet_len=plen, crc32=crc)


def check_crc(buf, packet_len):
    """CRC 覆盖 [24, packet_len)。返回 True/False；长度不足时返回 None。"""
    if packet_len < ENVELOPE_LEN or len(buf) < packet_len:
        return None
    return struct.unpack_from("<I", buf, ENV_OFF_CRC)[0] == \
        (zlib.crc32(buf[ENVELOPE_LEN:packet_len]) & 0xFFFFFFFF)


def parse_config(buf):
    """解析 CONFIG 包。越界的字段置 None 而不是抛异常。"""
    out = {}
    for name, rel, fmt in CONFIG_FIELDS:
        try:
            out[name] = struct.unpack_from("<" + fmt, buf, rel)[0]
        except struct.error:
            out[name] = None

    # 变长区: probe_name, wedge_name, notes —— 各自 u16 长度前缀 + UTF-8
    off = CONFIG_HEADER_LEN
    for key in ("probe_name", "wedge_name", "notes"):
        out[key] = None
        if len(buf) < off + 2:
            continue
        n = struct.unpack_from("<H", buf, off)[0]
        if len(buf) < off + 2 + n:
            continue
        out[key] = buf[off + 2:off + 2 + n].decode("utf-8", "replace")
        off += 2 + n
    return out


def parse_frame(buf):
    """解析 FRAME 包。返回 (帧块 dict, payload bytes)；不足则 (None, None)。"""
    if len(buf) < FRAME_HEADER_LEN:
        return None, None
    vals = struct.unpack_from(FRAME_FMT, buf, ENVELOPE_LEN)
    fr = dict(zip(FRAME_FIELDS, vals))
    n = fr["beam_count"] * fr["sample_count"]
    payload = buf[FRAME_HEADER_LEN:FRAME_HEADER_LEN + n]
    return fr, payload


def flag_names(v):
    return [n for b, n in sorted(STATUS_BITS.items()) if v & (1 << b)] or ["(none)"]


# ================================================================ 格式化

def fmt_ip(v):
    """board_ip 是 .NET IPAddress.Address 形式：0x8201A8C0 表示 192.168.1.130。

    即 'd.c.b.a' 打包成 a.b.c.d 的整数——四个八位要倒序还原，不是常见的网络序。
    验证方式：设备 config.json 里 `"ip": 2181146816`（=0x8201A8C0）应为 192.168.1.130。
    """
    if v is None:
        return "?"
    return "%d.%d.%d.%d" % (v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF)


def fmt_mhz(hz):
    if hz is None:
        return "?"
    if hz != hz:          # NaN
        return "NaN(未知)"
    return "%.4f" % (hz / 1e6)


def print_config(cfg, crc_state):
    print("-" * 74)
    print("CONFIG  config_seq=%s  packet_len=C段  crc=%s"
          % (cfg.get("config_seq"), crc_state))
    print("  [板卡] type=%s(%s) ip=%s utVoltage=%sV pulseWidth=%sns prf=%sHz"
          % (cfg.get("board_type"), BOARD_TYPE.get(cfg.get("board_type"), "?"),
             fmt_ip(cfg.get("board_ip")), cfg.get("ut_voltage_v"),
             cfg.get("pulse_width_ns"), cfg.get("prf_hz")))
    print("         filter_level=%s  sample_point_num=%s  range_ns=%s  start_ns_raw=%s(!不可信)"
          % (cfg.get("filter_level"), cfg.get("sample_point_num"),
             cfg.get("range_ns"), cfg.get("start_ns_raw")))
    print("  [探头] %s  element_total=%s  pitch=%smm  freq=%sMHz"
          % (cfg.get("probe_name"), cfg.get("element_total"),
             cfg.get("element_pitch_mm"), cfg.get("frequency_mhz")))
    print("  [楔块] %s  angle=%s°  velocity=%sm/s  firstEleHeight=%smm  primaryOffset=%smm"
          % (cfg.get("wedge_name"), cfg.get("wedge_angle_deg"), cfg.get("wedge_velocity_m_s"),
             cfg.get("first_ele_height_mm"), cfg.get("primary_offset_mm")))
    print("  [扫查] mode=%s(%s)  aperture=%s  step=%s  beam_count=%s"
          % (cfg.get("scan_mode"), SCAN_MODE.get(cfg.get("scan_mode"), "?"),
             cfg.get("little_aperture"), cfg.get("beam_step"), cfg.get("beam_count")))
    print("         img_height=%smm  img_height_bias=%smm  focus=%s°/%smm"
          % (cfg.get("img_height_mm"), cfg.get("img_height_bias_mm"),
             cfg.get("focus_angle_deg"), cfg.get("focus_depth_mm")))
    same_vel = cfg.get("specimen_vel_m_s") == cfg.get("board_vel_m_s")
    print("         specimen_vel=%sm/s  board_vel=%sm/s  %s"
          % (cfg.get("specimen_vel_m_s"), cfg.get("board_vel_m_s"),
             "" if same_vel else "← 两者不一致，需确认取哪个"))
    print("  [时基] fs=%sMHz  src=%s  dtype=%s  adc_bits=%s"
          % (fmt_mhz(cfg.get("sampling_rate_hz")),
             RATE_SRC.get(cfg.get("sampling_rate_src"), "?"),
             SAMPLE_DTYPE.get(cfg.get("sample_dtype"), "?"), cfg.get("adc_bits")))
    print("  [编码] enc1=%s(%s) pol=%s mode=%s | enc2=%s(%s) pol=%s mode=%s"
          % (cfg.get("enc1_resolution"), ENCODER_UNIT.get(cfg.get("enc1_unit"), "?"),
             cfg.get("enc1_polarity"), cfg.get("enc1_mode"),
             cfg.get("enc2_resolution"), ENCODER_UNIT.get(cfg.get("enc2_unit"), "?"),
             cfg.get("enc2_polarity"), cfg.get("enc2_mode")))
    if cfg.get("notes"):
        print("  [说明] %s" % cfg["notes"])
    print("-" * 74)


# ================================================================ 统计

class Stats(object):
    def __init__(self):
        self.total = 0
        self.by_type = {}
        self.crc_fail = 0
        self.magic_fail = 0
        self.len_mismatch = 0
        self.truncated = 0
        self.frame_seq = []
        self.sat_counts = []
        self.sat_mismatch = 0      # 设备上报的 saturated_count 与实际 255 字节数不符的帧数
        self.len_mismatch_frames = 0   # FRAME.sample_count 与 CONFIG.sample_point_num 不符的帧数
        self.value_min = 256
        self.value_max = -1
        self.zero_bytes = 0
        self.sat_bytes = 0
        self.byte_total = 0

    def feed_payload(self, payload):
        if not payload:
            return
        self.value_min = min(self.value_min, min(payload))
        self.value_max = max(self.value_max, max(payload))
        self.zero_bytes += payload.count(0)
        self.sat_bytes += payload.count(255)
        self.byte_total += len(payload)

    def summary(self):
        L = ["", "=" * 74, "统计", "=" * 74]
        L.append("  报文总数        : %d" % self.total)
        for t, c in sorted(self.by_type.items()):
            L.append("    %-10s    : %d" % (TYPE_NAME.get(t, "UNKNOWN(%d)" % t), c))
        L.append("  魔数不符        : %d" % self.magic_fail)
        L.append("  CRC 失败        : %d" % self.crc_fail)
        L.append("  截断(长度不足)  : %d" % self.truncated)
        L.append("  packet_len 不符 : %d" % self.len_mismatch)

        if self.frame_seq:
            gaps = sum(1 for a, b in zip(self.frame_seq, self.frame_seq[1:]) if b != a + 1)
            L.append("")
            L.append("  帧数            : %d" % len(self.frame_seq))
            L.append("  frame_seq 范围  : %d .. %d" % (self.frame_seq[0], self.frame_seq[-1]))
            L.append("  frame_seq 跳变  : %d 处%s"
                     % (gaps, "" if gaps == 0 else "  ← 连续应为 0；非 0 说明丢帧或端序有误"))

        if self.byte_total:
            L.append("")
            L.append("  字节值域        : %d .. %d" % (self.value_min, self.value_max))
            L.append("  恒 0 占比       : %.1f%%" % (100.0 * self.zero_bytes / self.byte_total))
            L.append("  饱和(255)占比   : %.2f%%" % (100.0 * self.sat_bytes / self.byte_total))
            if self.value_max > 255:
                L.append("  ⚠ 值域超过 255，但 sample_dtype 声明为 u8 —— 字节流不是 8 位？")
        if self.sat_counts:
            L.append("  saturated_count : min=%d max=%d"
                     % (min(self.sat_counts), max(self.sat_counts)))
            if self.sat_mismatch:
                L.append("  ⚠ 与实际 255 计数不符的帧：%d 帧 —— 设备端饱和统计或帧完整性有问题"
                         % self.sat_mismatch)
            else:
                L.append("  ✓ saturated_count 与实际 255 字节数逐帧一致")
        if self.len_mismatch_frames:
            L.append("  ⚠ %d 帧的 sample_count 与 CONFIG.sample_point_num 不一致"
                     " —— 深度换算请用 FRAME 的值" % self.len_mismatch_frames)
        if not self.by_type.get(TYPE_CONFIG):
            L.append("")
            L.append("  ⚠ 未收到 CONFIG —— 没有它无法解释维度与单位；检查设备端 SetConfig 是否被调用")
        if not self.by_type.get(TYPE_FRAME):
            L.append("")
            L.append("  ⚠ 未收到 FRAME —— 设备端可能未开采集，或 scan_mode 非线扫，或攒帧未凑齐")
        L.append("=" * 74)
        return "\n".join(L)


# ================================================================ 主流程

def main():
    ap = argparse.ArgumentParser(
        description="接收并解析 PAUT :12346 原始数据外发报文",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0", help="绑定地址，默认 0.0.0.0（收广播）")
    ap.add_argument("--port", type=int, default=12346, help="端口，默认 12346")
    ap.add_argument("--duration", type=float, default=0, help="接收秒数，0 = 持续到 Ctrl-C")
    ap.add_argument("--save", default=None, help="把收到的帧存成 .npz（需 numpy）")
    ap.add_argument("-v", "--verbose", action="store_true", help="每帧打印一行")
    ap.add_argument("--no-crc", action="store_true", help="跳过 CRC 校验（排障用）")
    ap.add_argument("--bufsize", type=int, default=65535, help="接收缓冲，默认 65535")
    args = ap.parse_args()

    if args.save and np is None:
        print("⚠ --save 需要 numpy，未安装，忽略该选项")
        args.save = None

    st = Stats()
    frames = []
    frame_meta = []
    cfg_dump = None
    crc_state = "skipped" if args.no_crc else "OK"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    except OSError:
        pass
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)

    print("监听 %s:%d ... (Ctrl-C 结束%s)"
          % (args.bind, args.port, "" if not args.duration else "，%.0fs 后自动停止" % args.duration))

    t0 = time.time()
    last_hb = 0.0
    try:
        while True:
            if args.duration and (time.time() - t0) >= args.duration:
                break
            try:
                data, addr = sock.recvfrom(args.bufsize)
            except socket.timeout:
                continue

            st.total += 1
            env = parse_envelope(data)
            if env is None:
                st.magic_fail += 1
                continue

            ptype = env["packet_type"]
            st.by_type[ptype] = st.by_type.get(ptype, 0) + 1

            # 长度自洽性检查（与内容推算的长度比对）
            expect = None
            if ptype == TYPE_FRAME and len(data) >= FRAME_OFF_BEAM_COUNT + 4:
                bc, sc = struct.unpack_from("<HH", data, FRAME_OFF_BEAM_COUNT)
                expect = FRAME_HEADER_LEN + bc * sc
            elif ptype == TYPE_HEARTBEAT:
                expect = HEARTBEAT_LEN
            if expect is not None and env["packet_len"] != expect:
                st.len_mismatch += 1
                print("⚠ packet_len=%d，但按内容应为 %d（来自 %s）"
                      % (env["packet_len"], expect, addr[0]))

            if not args.no_crc:
                ok = check_crc(data, env["packet_len"])
                if ok is None:
                    st.truncated += 1
                    print("⚠ 报文被截断：实际 %d 字节 < packet_len %d（来自 %s）"
                          % (len(data), env["packet_len"], addr[0]))
                    continue
                if ok is False:
                    st.crc_fail += 1
                    print("⚠ CRC 失败（来自 %s, type=%s, len=%d）"
                          % (addr[0], TYPE_NAME.get(ptype, ptype), len(data)))
                    continue

            if ptype == TYPE_CONFIG:
                cfg = parse_config(data)
                cfg_dump = cfg
                print_config(cfg, crc_state)

            elif ptype == TYPE_FRAME:
                fr, payload = parse_frame(data)
                if fr is None:
                    continue
                st.frame_seq.append(fr["frame_seq"])
                st.sat_counts.append(fr["saturated_count"])
                st.feed_payload(payload)

                # 交叉校验：设备上报的 saturated_count 应等于本帧 255 的字节数。
                # 不符说明设备端统计逻辑有问题，或帧被截断（payload 长度不足）。
                if payload:
                    actual_sat = payload.count(255)
                    if actual_sat != fr["saturated_count"]:
                        st.sat_mismatch += 1
                        if st.sat_mismatch == 1:
                            print("⚠ saturated_count 不符：设备上报 %d，实际 255 字节数 %d（seq=%d）"
                                  % (fr["saturated_count"], actual_sat, fr["frame_seq"]))

                # 交叉校验：帧的 sample_count 应与 CONFIG 声明的 sample_point_num 一致。
                # 实测设备端两者不等（1000 vs 896）——sample_point_num 是板卡配置的采样点数，
                # 而每波束实际传来的字节数是另一个值。深度换算必须用 FRAME 的值。
                if cfg_dump and cfg_dump.get("sample_point_num"):
                    if fr["sample_count"] != cfg_dump["sample_point_num"]:
                        st.len_mismatch_frames += 1
                        if st.len_mismatch_frames == 1:
                            print("⚠ FRAME.sample_count=%d 与 CONFIG.sample_point_num=%d 不一致"
                                  "（深度换算以 FRAME 为准）"
                                  % (fr["sample_count"], cfg_dump["sample_point_num"]))

                if args.verbose:
                    print("FRAME seq=%-8d %dx%-4d enc=(%d,%d) sat=%-6d flags=%s"
                          % (fr["frame_seq"], fr["beam_count"], fr["sample_count"],
                             fr["scan_axis_encoder"], fr["index_axis_encoder"],
                             fr["saturated_count"], ",".join(flag_names(fr["status_flags"]))))
                elif len(st.frame_seq) == 1:
                    print("FRAME 首帧 seq=%d  %d 波束 x %d 采样点  enc=(%d,%d)  flags=%s"
                          % (fr["frame_seq"], fr["beam_count"], fr["sample_count"],
                             fr["scan_axis_encoder"], fr["index_axis_encoder"],
                             ",".join(flag_names(fr["status_flags"]))))
                    if cfg_dump is not None and fr["config_seq"] != cfg_dump.get("config_seq"):
                        print("  ⚠ 帧的 config_seq=%s 与最近 CONFIG 的 %s 不一致"
                              % (fr["config_seq"], cfg_dump.get("config_seq")))

                if args.save:
                    frames.append(
                        np.frombuffer(payload, dtype=np.uint8)
                          .reshape(fr["beam_count"], fr["sample_count"]).copy())
                    frame_meta.append(fr)

            elif ptype == TYPE_HEARTBEAT:
                now = time.time()
                if now - last_hb > 5.0:
                    last_hb = now
                    cnt = struct.unpack_from("<Q", data, ENVELOPE_LEN)[0] \
                        if len(data) >= HEARTBEAT_LEN else 0
                    print("HEARTBEAT 计数=%d  (设备在线；无 FRAME 说明未开采集或非线扫模式)" % cnt)

            else:
                print("未知 packet_type=%d（来自 %s, len=%d）" % (ptype, addr[0], len(data)))

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，停止。")
    finally:
        sock.close()

    print(st.summary())

    if args.save and frames:
        cfg_dump = cfg_dump or {}
        scalar = {}
        for k, v in cfg_dump.items():
            if isinstance(v, bool):
                continue
            if isinstance(v, (int, float)):
                scalar["cfg_" + k] = np.array(v)
        np.savez_compressed(
            args.save,
            frames=np.stack(frames),
            frame_seq=np.array([m["frame_seq"] for m in frame_meta], dtype=np.uint64),
            scan_axis_encoder=np.array([m["scan_axis_encoder"] for m in frame_meta], dtype=np.int32),
            index_axis_encoder=np.array([m["index_axis_encoder"] for m in frame_meta], dtype=np.int32),
            saturated_count=np.array([m["saturated_count"] for m in frame_meta], dtype=np.uint32),
            status_flags=np.array([m["status_flags"] for m in frame_meta], dtype=np.uint16),
            **scalar)
        print("已保存 %d 帧到 %s" % (len(frames), args.save))
        print("  读取: d = np.load('%s'); d['frames'].shape   # (帧, 波束, 采样点)"
              % args.save)

    bad = st.magic_fail + st.crc_fail + st.truncated + st.len_mismatch
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
