#!/usr/bin/env python3
"""从 PAUT v1 的 mcap bag 里抽出波形并画图。

与 scripts/recv_paut_12346.py 的分工:
    recv_paut_12346.py —— 实时看（设备正在推流时）
    view_paut_bag.py   —— 事后看（读已录制的 bag）

用法:
    python3 scripts/view_paut_bag.py bags/paut_run_20260929_130825_073769
    python3 scripts/view_paut_bag.py <bag> --frames 300 --beam 30
    python3 scripts/view_paut_bag.py <bag> -o /tmp/paut.png --no-show

产出:
    一张 PNG，三个子图:
      ① A 扫        —— 指定波束的波形 + 全体波束的中位数剖面
      ② B 扫热图    —— 按 NDT 惯例 **x = 扫查位置(波束)、y = 深度**（深度向下递增）
      ③ 回波随时间  —— 每帧峰值，判断"探头有没有动 / 耦合有没有变化"

⚠ 深度轴依赖 PautConfig 的 sampling_rate_hz 与 specimen_vel_m_s，而**采样率是设备
  反推值、未经验证**（sampling_rate_src=derived_unverified）。故深度轴仅供参考，图上
  会标注 UNVERIFIED。横轴波束位置只依赖 element_pitch_mm 与 beam_step，那两个可靠。

需要 source ROS 环境（用到 rosbag2_py 与消息类型）:
    export PATH=/usr/bin:$PATH
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
"""

import argparse
import math
import os
from pathlib import Path
import sys

# ── 解释器自愈 ───────────────────────────────────────────────────────────
# 本工作区的 ROS 2 (Jazzy) 是为 /usr/bin/python3 (3.12) 构建的。
# 系统的 `python` 很可能是 anaconda 的 3.7 —— 它 import rosbag2_py 会报
#   ModuleNotFoundError: No module named 'rosbag2_py._compression_options'
# 与其让用户每次记得敲 /usr/bin/python3，这里自动换过去再跑一次。
# （第二次执行时 sys.executable 已相同，不会再触发，无死循环。）
_EXPECTED_PY = "/usr/bin/python3"
if (os.path.exists(_EXPECTED_PY)
        and os.path.realpath(sys.executable) != os.path.realpath(_EXPECTED_PY)):
    sys.stderr.write(f"[view_paut_bag] 当前解释器是 {sys.executable}，"
                     f"自动切换到 {_EXPECTED_PY}（ROS 2 是为它构建的）\n")
    os.execv(_EXPECTED_PY, [_EXPECTED_PY] + sys.argv)

import numpy as np

FRAME_TOPIC = "/inspection/paut/raw_v2"
CONFIG_TOPIC = "/inspection/paut/config"


def _read_bag(bag_dir, max_frames, verbose):
    """读 bag，返回 (frames[N,beam,sample] uint8, config 或 None, 统计 dict)。"""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit(
            f"导入 ROS 2 Python 模块失败: {exc}\n\n"
            f"本脚本已自动切到 /usr/bin/python3；剩下的失败通常是**没 source ROS**。\n"
            f"请这样运行:\n"
            f"    export PATH=/usr/bin:$PATH\n"
            f"    source /opt/ros/jazzy/setup.bash && source install/setup.bash\n"
            f"    python3 scripts/view_paut_bag.py <bag 目录>")

    bag_dir = Path(bag_dir).resolve()
    if not bag_dir.is_dir():
        raise SystemExit(f"不是目录: {bag_dir}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (FRAME_TOPIC, CONFIG_TOPIC):
        if topic not in type_map:
            raise SystemExit(
                f"bag 里没有 {topic}；可用话题: {sorted(type_map)}\n"
                f"（旧 bag 用的是 /inspection/paut/raw + PautFrame，本脚本不支持）")

    try:
        frame_cls = get_message(type_map[FRAME_TOPIC])
        config_cls = get_message(type_map[CONFIG_TOPIC])
    except Exception as exc:
        raise SystemExit(
            f"加载消息类型失败: {type_map[FRAME_TOPIC]} / {type_map[CONFIG_TOPIC]}\n"
            f"  底层错误: {exc}\n\n"
            f"通常是因为**没 source 本工作区的 install**（ROS 主体 source 了、但\n"
            f"inspection_interfaces 找不到）。请补齐:\n"
            f"    export PATH=/usr/bin:$PATH\n"
            f"    source /opt/ros/jazzy/setup.bash\n"
            f"    source install/setup.bash          # ← 这一句才是关键\n"
            f"    python3 scripts/view_paut_bag.py <bag 目录>")

    frames = []
    frame_seqs = []
    config = None
    total_frames = 0
    skipped = 0
    first_stamp = last_stamp = None

    while reader.has_next():
        topic, serialized, _bag_ns = reader.read_next()
        if topic == CONFIG_TOPIC:
            if config is None:
                config = deserialize_message(serialized, config_cls)
            continue
        if topic != FRAME_TOPIC:
            continue

        total_frames += 1
        if len(frames) >= max_frames:
            continue

        msg = deserialize_message(serialized, frame_cls)
        beam = int(msg.beam_count)
        samp = int(msg.sample_count)
        arr = np.frombuffer(bytes(msg.samples), dtype=np.uint8)
        if arr.size != beam * samp:
            skipped += 1
            continue
        frames.append(arr.reshape(beam, samp))
        frame_seqs.append(int(msg.frame_seq))
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if first_stamp is None:
            first_stamp = stamp_ns
        last_stamp = stamp_ns

    if not frames:
        raise SystemExit(
            "没有读到任何帧。可能原因: bag 为空 / 本次采集 raw_v2 的 Count 为 0"
            "（QoS 不匹配，见 docs/PAUT_12346_ROS2_MAPPING_ZH.md §6.2）")

    stats = {
        "total_frames": total_frames,
        "loaded_frames": len(frames),
        "frame_seqs": frame_seqs,
        "skipped": skipped,
        "duration_s": ((last_stamp - first_stamp) / 1e9) if first_stamp else None,
    }
    if verbose:
        print(f"  bag 内帧总数 {total_frames}，已载入 {len(frames)}"
              + (f"，跳过 {skipped}" if skipped else ""))
    return np.stack(frames), config, stats


def main():
    ap = argparse.ArgumentParser(
        description="从 PAUT v1 bag 抽波形并画图",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_directory", type=Path)
    ap.add_argument("--frames", type=int, default=500,
                    help="最多载入多少帧用于画图（默认 500；bag 很大时会慢）")
    ap.add_argument("--beam", type=int, default=-1,
                    help="A 扫画哪一条波束（默认取中间那条）")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="PNG 输出路径（默认写到 bag 目录下 paut_waveform.png）")
    ap.add_argument("--no-show", action="store_true",
                    help="只存图不弹窗（无显示环境时用）")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args()

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # 中文字体：matplotlib 的默认字体不含 CJK，不设的话图上所有中文会渲染成方块。
    # 按可用性依次回退；正号/负号用 ASCII 以免 unicode_minus 缺字形。
    matplotlib.rcParams["font.sans-serif"] = [
        "Noto Sans CJK JP", "Noto Sans CJK SC", "Droid Sans Fallback",
        "WenQuanYi Zen Hei", "SimHei", "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False

    verbose = not args.quiet
    if verbose:
        print(f"读取 {args.bag_directory} ...")
    frames, config, stats = _read_bag(args.bag_directory, args.frames, verbose)

    n_frames, beam_count, sample_count = frames.shape
    seqs = stats["frame_seqs"]

    def cfg(name, default=None):
        return getattr(config, name, default) if config is not None else default

    fs = float(cfg("sampling_rate_hz", float("nan")))
    vel = float(cfg("specimen_vel_m_s", float("nan")))
    pitch = float(cfg("element_pitch_mm", float("nan")))
    step = int(cfg("beam_step", 0) or 0)
    notes = str(cfg("notes", "") or "")

    # ---- 深度轴（依赖反推的采样率，故标注 UNVERIFIED）----
    depth_ok = math.isfinite(fs) and fs > 0 and math.isfinite(vel) and vel > 0
    if depth_ok:
        # depth = v*t/2；initial_time 未知（设备 start_ns_raw 不可信），按 0 处理
        depth = vel * (np.arange(sample_count, dtype=float) / fs) / 2.0 * 1000.0
        depth_label = "depth [mm]  ⚠ fs 为设备反推值, 初时按 0"
    else:
        depth = np.arange(sample_count, dtype=float)
        depth_label = "sample index (fs/velocity 未知, 无法换算深度)"

    # ---- 波束位置轴（只依赖 pitch 与 step，这两个可靠）----
    if math.isfinite(pitch) and pitch > 0 and step > 0:
        beam_pos = np.arange(beam_count, dtype=float) * pitch * step
        pos_label = "beam position [mm]"
    else:
        beam_pos = np.arange(beam_count, dtype=float)
        pos_label = "beam index"

    # ---------------- 统计 ----------------
    total = frames.size
    n_zero = int(np.count_nonzero(frames == 0))
    n_sat = int(np.count_nonzero(frames == 255))

    print()
    print("=" * 68)
    print("PAUT 采集数据概览")
    print("=" * 68)
    print(f"  帧数            : {n_frames}"
          + (f"  (bag 内共 {stats['total_frames']})"
             if stats["total_frames"] != n_frames else ""))
    if stats["duration_s"] and stats["duration_s"] > 0:
        print(f"  时长            : {stats['duration_s']:.2f} s"
              f"   →  ≈{n_frames / stats['duration_s']:.1f} fps")
    print(f"  每帧维度        : {beam_count} 波束 × {sample_count} 采样点")
    print(f"  值域            : {frames.min()} .. {frames.max()}")
    print(f"  恒 0 占比       : {100.0 * n_zero / total:.1f}%")
    print(f"  饱和(255) 占比  : {100.0 * n_sat / total:.2f}%")
    if seqs:
        gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)
        print(f"  frame_seq       : {seqs[0]}..{seqs[-1]}   跳变 {gaps} 处")
    if config is not None:
        print()
        print("  采集配置:")
        print(f"    探头          : {cfg('probe_name')}   {cfg('element_total')} 阵元"
              f"   pitch={cfg('element_pitch_mm')}mm   {cfg('frequency_mhz')}MHz")
        print(f"    楔块          : {cfg('wedge_name')}   角度={cfg('wedge_angle_deg')}°")
        print(f"    波束          : {cfg('beam_count')} 条   步进={cfg('beam_step')}"
              f"   孔径={cfg('little_aperture')}")
        print(f"    时基          : fs={fs / 1e6:.3f} MHz   range={cfg('range_ns')} ns"
              f"   试件声速={vel} m/s")
        print(f"    配置序号      : config_seq={cfg('config_seq')}")
        if notes:
            print(f"    设备说明      : {notes[:96]}" + ("..." if len(notes) > 96 else ""))
    else:
        print("  ⚠ 未读到 /inspection/paut/config —— 深度轴无法换算")
    if depth_ok:
        print()
        print(f"  ⚠ 深度轴按 fs={fs / 1e6:.2f}MHz、初时=0 换算，"
              f"而 fs 是设备反推值（UNVERIFIED），仅作参考")
    print("=" * 68)

    # ---------------- 画图 ----------------
    beam_idx = args.beam if 0 <= args.beam < beam_count else beam_count // 2

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2))

    # ① A 扫：指定波束 + 全体波束中位数
    ax = axes[0]
    ax.plot(depth, frames[0, beam_idx, :], lw=0.8, color="#1f77b4",
            label=f"beam {beam_idx}")
    ax.plot(depth, np.median(frames[0], axis=0), lw=1.0, color="#d62728",
            alpha=0.75, label="median over all beams")
    ax.set_xlabel(depth_label)
    ax.set_ylabel("amplitude (uint8)")
    ax.set_title(f"A-scan  (fs={fs / 1e6:.2f} MHz)" if depth_ok else "A-scan")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # ② B 扫热图 —— 按 NDT 惯例：**x = 扫查位置(波束)、y = 深度**（横截面视图）。
    # frames[0] 是 [beam, sample]，转置成 [sample, beam] 后 imshow 的
    # 行→y(深度)、列→x(波束)；origin="upper" 让深度 0 在顶部、向下递增。
    # 配色用 jet —— 与团队既有 B 扫视图(udp_Bscan)保持一致，便于对照。
    ax = axes[1]
    im = ax.imshow(
        frames[0].T.astype(np.float32),
        aspect="auto", origin="upper", cmap="jet",
        extent=[beam_pos[0], beam_pos[-1], depth[-1], depth[0]])
    ax.set_xlabel(pos_label)
    ax.set_ylabel(depth_label)
    ax.set_title("B-scan (frame 0)")
    fig.colorbar(im, ax=ax, label="amplitude (uint8)")

    # ③ 回波随时间：每帧峰值
    ax = axes[2]
    peak = frames.reshape(n_frames, -1).max(axis=1).astype(np.float64)
    ax.plot(np.arange(n_frames), peak, lw=1.0, color="#2ca02c")
    ax.set_xlabel("frame index")
    ax.set_ylabel("max amplitude (uint8)")
    ax.set_title("echo peak vs time")
    ax.grid(alpha=0.3)
    ax.set_ylim(0, 260)
    if n_frames > 1 and float(np.ptp(peak)) <= 1.0:
        ax.text(0.5, 0.06, "峰值无变化 -> 探头静止或未耦合",
                transform=ax.transAxes, ha="center", color="#d62728", fontsize=9)

    if "UNVERIFIED" in notes:
        fig.suptitle("⚠ 采样率为设备反推值、未经验证；深度轴仅作参考", color="#d62728")

    fig.tight_layout()

    out = args.out or (Path(args.bag_directory) / "paut_waveform.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"\n图已保存: {out}")
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
