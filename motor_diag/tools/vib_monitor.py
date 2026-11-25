#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32F407 Motor Vibration Online Diagnosis & Blackbox Host Tool (W5 v1.1)
Usage:
  1. Export blackbox fault summary table to CSV + stats:
       python vib_monitor.py dump --port COM11 --count 32 --out blackbox_dump.csv
  2. Fetch a 502-bin q15 fault spectrum snapshot, export CSV & plot PNG:
       python vib_monitor.py snap --port COM11 --slot 32 --out snap_32.png
  3. Active 1h stability / fault-injection monitor with periodic stat+resetinfo polling
     and start/end blackbox next_seq audit:
       python vib_monitor.py watch --port COM11 --duration 3600 --poll-interval 30
  4. Offline self-test (no serial port required):
       python vib_monitor.py selftest
"""

import argparse
import collections
import csv
import datetime
import re
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


DUMP_HDR_RE = re.compile(
    r"\[BB DUMP\]\s+valid=(\d+)\s+corrupt=(\d+)\s+next_seq=(\d+)\s+\(showing\s+(\d+)\)"
)
DUMP_ROW_RE = re.compile(
    r"^\s*(\d+)\s+(\d+)\s+([A-Z0-9_]+)\s+([-+]?\d+(?:\.\d+)?)\s+"
    r"([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)\s+(\d+)\s*$"
)
SNAP_HDR_RE = re.compile(
    r"\[SNAP\s+(\d+)\]\s+seq_low=(\d+)\s+ts=(\d+)\s+fs=([-+]?\d+(?:\.\d+)?)\s+scale_max=([-+]?\d+(?:\.\d+)?)"
)
SNAP_BIN_RE = re.compile(r"^B(\d{3}):([-\d,]+)\s*$")
BB_SAVED_RE = re.compile(
    r"\[BB\]\s+Saved\s+seq=(\d+)\s+type=([A-Z0-9_]+)\s+peak=([-+]?\d+(?:\.\d+)?)Hz\s+val=([-+]?\d+(?:\.\d+)?)\s+snap=(\d+)"
)
HEAP_RE = re.compile(r"Free Heap:\s*(\d+)\s*B\s*\|\s*Min Ever Free:\s*(\d+)\s*B")
SUP_RE = re.compile(r"\[SUP\]\s+ResetReason=([A-Z0-9_]+).*IWDG_Feeds=(\d+)\s*\|\s*Uptime=(\d+)ms")


def parse_dump_lines(lines):
    """Parse CLI 'dump' output into metadata dict and list of row dicts."""
    meta = {}
    rows = []
    for raw in lines:
        line = raw.strip()
        m_hdr = DUMP_HDR_RE.search(line)
        if m_hdr:
            meta = {
                "valid": int(m_hdr.group(1)),
                "corrupt": int(m_hdr.group(2)),
                "next_seq": int(m_hdr.group(3)),
                "showing": int(m_hdr.group(4)),
            }
            continue
        m_row = DUMP_ROW_RE.match(line)
        if m_row:
            rows.append({
                "seq": int(m_row.group(1)),
                "tick_ms": int(m_row.group(2)),
                "type": m_row.group(3),
                "peak_hz": float(m_row.group(4)),
                "score_or_800": float(m_row.group(5)),
                "rms_or_amp": float(m_row.group(6)),
                "snap_slot": int(m_row.group(7)),
            })
    return meta, rows


def parse_snap_lines(lines):
    """Parse CLI 'snap <slot>' output into metadata dict and 502-bin q15 list."""
    meta = {}
    bins = {}
    for raw in lines:
        line = raw.strip()
        m_hdr = SNAP_HDR_RE.search(line)
        if m_hdr:
            meta = {
                "slot": int(m_hdr.group(1)),
                "seq_low": int(m_hdr.group(2)),
                "ts_ms": int(m_hdr.group(3)),
                "fs_hz": float(m_hdr.group(4)),
                "scale_max": float(m_hdr.group(5)),
            }
            continue
        m_bin = SNAP_BIN_RE.match(line)
        if m_bin:
            base = int(m_bin.group(1))
            vals = [int(x) for x in m_bin.group(2).split(",") if x.strip() != ""]
            for idx, v in enumerate(vals):
                bins[base + idx] = v

    if not meta or not bins:
        return meta, []

    n_bins = max(bins.keys()) + 1
    q15 = [bins.get(i, 0) for i in range(n_bins)]
    return meta, q15


def save_dump_csv(rows, out_path):
    fieldnames = ["seq", "tick_ms", "type", "peak_hz", "score_or_800", "rms_or_amp", "snap_slot"]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_snapshot(meta, q15, out_png, out_csv=None):
    fs_hz = meta.get("fs_hz", 3200.0)
    scale_max = meta.get("scale_max", 1.0)
    df = fs_hz / 1024.0

    freqs = [k * df for k in range(len(q15))]
    amps = [(v / 32767.0) * scale_max for v in q15]

    if out_csv:
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["bin", "freq_hz", "q15", "amplitude"])
            for k, (fr, v, a) in enumerate(zip(freqs, q15, amps)):
                w.writerow([k, f"{fr:.3f}", v, f"{a:.5f}"])

    peaks = sorted(
        [(amps[k], freqs[k], k) for k in range(3, len(amps))],
        reverse=True,
    )
    top3 = []
    for a, fr, k in peaks:
        if all(abs(fr - p[1]) > (4.0 * df) for p in top3):
            top3.append((a, fr, k))
        if len(top3) == 3:
            break

    print(f"[SNAP {meta['slot']}] seq_low={meta['seq_low']} ts={meta['ts_ms']}ms "
          f"fs={fs_hz:.1f}Hz bins={len(q15)} scale_max={scale_max:.4f}")
    for rank, (a, fr, k) in enumerate(top3, 1):
        print(f"  Peak #{rank}: {fr:7.2f} Hz (bin {k:3d}) | amp={a:.4f}")

    if plt is None:
        print("[WARN] matplotlib not installed; skipped PNG plot. Install via: pip install matplotlib")
        return

    fig, ax = plt.subplots(figsize=(10, 4.6), dpi=150)
    fig.patch.set_facecolor("#0f172a")
    ax.set_facecolor("#111827")

    ax.plot(freqs[2:], amps[2:], color="#38bdf8", linewidth=1.2, label="FFT Magnitude")
    ax.fill_between(freqs[2:], 0, amps[2:], color="#38bdf8", alpha=0.18)

    for rank, (a, fr, _k) in enumerate(top3, 1):
        ax.scatter([fr], [a], color="#f43f5e", s=36, zorder=5)
        ax.annotate(
            f"P{rank}: {fr:.1f} Hz",
            xy=(fr, a),
            xytext=(6, 6),
            textcoords="offset points",
            color="#fda4af",
            fontsize=8.5,
            fontweight="bold",
        )

    ax.set_title(
        f"Blackbox Fault Spectrum Snapshot | Slot #{meta['slot']} "
        f"(fs={fs_hz:.0f} Hz, ts={meta['ts_ms']} ms)",
        color="#f8fafc",
        fontsize=11,
        pad=10,
    )
    ax.set_xlabel("Frequency (Hz)", color="#cbd5e1")
    ax.set_ylabel("Magnitude (scaled)", color="#cbd5e1")
    ax.tick_params(colors="#94a3b8")
    ax.grid(True, linestyle="--", alpha=0.25, color="#64748b")
    for spine in ax.spines.values():
        spine.set_color("#334155")

    plt.tight_layout()
    plt.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"[OK] Saved spectrum plot -> {out_png}")


def open_serial(port, baud):
    if serial is None:
        print("[ERR] pyserial not installed. Run: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    return serial.Serial(port=port, baudrate=baud, timeout=0.25)


def send_and_collect(ser, cmd, idle_timeout=0.9, max_time=6.0):
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\r\n").encode("ascii"))
    ser.flush()

    lines = []
    t0 = time.monotonic()
    last_rx = t0
    while (time.monotonic() - t0) < max_time:
        raw = ser.readline()
        if raw:
            last_rx = time.monotonic()
            lines.append(raw.decode("utf-8", errors="replace"))
        elif lines and ((time.monotonic() - last_rx) >= idle_timeout):
            break
    return lines


def cmd_dump(args):
    with open_serial(args.port, args.baud) as ser:
        lines = send_and_collect(ser, f"dump {args.count}")
    meta, rows = parse_dump_lines(lines)
    if not rows:
        print("[WARN] No blackbox records parsed. Raw output:")
        print("".join(lines))
        return
    save_dump_csv(rows, args.out)
    counts = collections.Counter(r["type"] for r in rows)
    print(f"[OK] Parsed {len(rows)} records (valid={meta.get('valid')} corrupt={meta.get('corrupt')} "
          f"next_seq={meta.get('next_seq')}) -> {args.out}")
    print("  Breakdown:", ", ".join(f"{k}={v}" for k, v in counts.most_common()))


def cmd_snap(args):
    with open_serial(args.port, args.baud) as ser:
        lines = send_and_collect(ser, f"snap {args.slot}", idle_timeout=1.0, max_time=8.0)
    meta, q15 = parse_snap_lines(lines)
    if not q15:
        print("[WARN] Snapshot invalid or empty. Raw output:")
        print("".join(lines))
        return
    csv_path = str(Path(args.out).with_suffix(".csv"))
    plot_snapshot(meta, q15, args.out, out_csv=csv_path)


def cmd_watch(args):
    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log or f"motor_diag_session_{ts_str}.log"
    fault_counts = collections.Counter()
    heap_samples = []
    feed_samples = []
    boot_restarts = 0
    poll_cnt = 0

    print(f"[WATCH] Port={args.port} @ {args.baud} | duration={args.duration}s | "
          f"poll={args.poll_interval}s -> {log_path} (Ctrl+C to stop)")

    with open_serial(args.port, args.baud) as ser, open(log_path, "w", encoding="utf-8") as lf:
        # Optional startup commands (e.g. "motor 2 400;mode phy")
        if args.init_cmd:
            for c in [x.strip() for x in args.init_cmd.split(";") if x.strip()]:
                ser.write((c + "\r\n").encode("ascii"))
                ser.flush()
                time.sleep(0.35)

        # Baseline blackbox cursor before test
        start_meta, _ = parse_dump_lines(send_and_collect(ser, "dump 5", idle_timeout=0.6, max_time=3.0))
        lf.write(f"# START_BB_META: {start_meta}\n")
        lf.flush()
        print(f"[AUDIT] Start Blackbox: {start_meta}")

        t0 = time.monotonic()
        next_poll = t0 if args.poll_interval > 0 else float("inf")

        try:
            while True:
                now = time.monotonic()
                if (args.duration > 0) and ((now - t0) >= args.duration):
                    break

                # Periodic health poll (stat + resetinfo) so stability logs are never empty
                if now >= next_poll:
                    poll_cnt += 1
                    ser.write(b"stat\r\n")
                    ser.flush()
                    time.sleep(0.15)
                    ser.write(b"resetinfo\r\n")
                    ser.flush()
                    next_poll = now + args.poll_interval

                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if not text:
                    continue

                now_s = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                lf.write(f"[{now_s}] {text}\n")
                lf.flush()
                print(f"[{now_s}] {text}")

                m_bb = BB_SAVED_RE.search(text)
                if m_bb:
                    fault_counts[m_bb.group(2)] += 1

                m_hp = HEAP_RE.search(text)
                if m_hp:
                    heap_samples.append((int(m_hp.group(1)), int(m_hp.group(2))))

                m_sup = SUP_RE.search(text)
                if m_sup:
                    feed_samples.append((int(m_sup.group(2)), int(m_sup.group(3))))

                if "Motor Diag CLI Ready" in text:
                    boot_restarts += 1
        except KeyboardInterrupt:
            pass

        end_meta, _ = parse_dump_lines(send_and_collect(ser, "dump 5", idle_timeout=0.6, max_time=3.0))
        lf.write(f"# END_BB_META: {end_meta}\n")
        lf.flush()

    elapsed = time.monotonic() - t0
    delta_seq = None
    if ("next_seq" in start_meta) and ("next_seq" in end_meta):
        delta_seq = end_meta["next_seq"] - start_meta["next_seq"]

    print(f"\n=== Session Audit Summary ({elapsed:.1f} s) ===")
    print(f"Log file        : {log_path}")
    print(f"Health polls    : {poll_cnt}")
    print(f"Unexpected boot : {boot_restarts}")
    print(f"Blackbox seq    : {start_meta.get('next_seq')} -> {end_meta.get('next_seq')} "
          f"(delta={delta_seq}, corrupt={end_meta.get('corrupt')})")
    if heap_samples:
        print(f"Free Heap       : start={heap_samples[0][0]} B -> end={heap_samples[-1][0]} B "
              f"(Min Ever={heap_samples[-1][1]} B)")
    if feed_samples:
        print(f"IWDG Feeds      : start={feed_samples[0][0]} -> end={feed_samples[-1][0]} "
              f"(+{feed_samples[-1][0] - feed_samples[0][0]})")
    print(f"Live fault logs : {sum(fault_counts.values())}")
    for k, v in fault_counts.most_common():
        print(f"  {k:<12}: {v}")


def cmd_selftest(_args):
    sample_dump = [
        "[BB DUMP] valid=1018 corrupt=0 next_seq=22009 (showing 3)\r\n",
        "22008  1567   IWDG_RST  0.0    0.00   0.00   65535\r\n",
        "21998  147660 OVERTEMP  400.0  28.10  16.79  65535\r\n",
        "21993  138768 OVERLOAD  400.0  14.10  186.31 32\r\n",
    ]
    meta, rows = parse_dump_lines(sample_dump)
    assert meta["corrupt"] == 0 and len(rows) == 3

    assert HEAP_RE.search("Free Heap: 18768 B | Min Ever Free: 18432 B") is not None
    assert SUP_RE.search("[SUP] ResetReason=SOFT_RESET (RCC_CSR=0x14000003) | IWDG_Feeds=173 | Uptime=43862ms") is not None

    sample_snap = ["[SNAP 32] seq_low=21993 ts=138768 fs=3200.00 scale_max=250.0000\r\n"]
    q15_mock = [120] * 502
    q15_mock[64] = 18000
    q15_mock[128] = 32767
    q15_mock[256] = 24000
    for base in range(0, 502, 16):
        chunk = ",".join(str(x) for x in q15_mock[base:base + 16])
        sample_snap.append(f"B{base:03d}:{chunk}\r\n")

    s_meta, q15 = parse_snap_lines(sample_snap)
    assert len(q15) == 502 and q15[128] == 32767
    plot_snapshot(s_meta, q15, "selftest_snap.png", out_csv="selftest_snap.csv")
    print("[PASS] vib_monitor.py v1.1 offline self-test passed!")


def main():
    parser = argparse.ArgumentParser(description="STM32F407 Motor Diag Host Monitor & Blackbox Parser")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_dump = sub.add_parser("dump", help="Dump blackbox fault records to CSV")
    p_dump.add_argument("--port", required=True, help="Serial port (e.g. COM11)")
    p_dump.add_argument("--baud", type=int, default=115200)
    p_dump.add_argument("--count", type=int, default=32)
    p_dump.add_argument("--out", default="blackbox_dump.csv")
    p_dump.set_defaults(func=cmd_dump)

    p_snap = sub.add_parser("snap", help="Fetch and plot a 502-bin FFT snapshot")
    p_snap.add_argument("--port", required=True, help="Serial port (e.g. COM11)")
    p_snap.add_argument("--baud", type=int, default=115200)
    p_snap.add_argument("--slot", type=int, required=True, help="Snapshot slot index (0..63)")
    p_snap.add_argument("--out", default="snap_spectrum.png")
    p_snap.set_defaults(func=cmd_snap)

    p_watch = sub.add_parser("watch", help="Active serial monitor & timed stability auditor")
    p_watch.add_argument("--port", required=True, help="Serial port (e.g. COM11)")
    p_watch.add_argument("--baud", type=int, default=115200)
    p_watch.add_argument("--duration", type=int, default=0, help="Auto-stop seconds (0 = unlimited)")
    p_watch.add_argument("--poll-interval", type=int, default=30,
                         help="Seconds between automatic 'stat' + 'resetinfo' health polls (0 = passive)")
    p_watch.add_argument("--init-cmd", default="",
                         help="Optional semicolon-separated startup CLI commands (e.g. 'motor 2 400;mode phy')")
    p_watch.add_argument("--log", default="", help="Optional output log path")
    p_watch.set_defaults(func=cmd_watch)

    p_st = sub.add_parser("selftest", help="Run offline parser & plotter self-test")
    p_st.set_defaults(func=cmd_selftest)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()