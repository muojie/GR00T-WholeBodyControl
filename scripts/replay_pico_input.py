#!/usr/bin/env python3
"""Replay recorded headset input into pico_manager_thread_server.py (no headset).

Runs pico_manager in-process with a replay shim standing in for xrobotoolkit_sdk that
serves the values captured by record_pico_input.py, time-aligned to wall-clock. Two
poller threads (body reader + controller getters) each get the value that was live at
the corresponding moment, so pico_manager reproduces the exact motion -- calibration at
the recorded A+B+X+Y instant included -- and publishes to deploy as if the headset were
attached.

IMPORTANT: the launcher's live pico_manager must be STOPPED first (both bind the same
ZMQ :5556). deploy / C++ proxy / IsaacLab stay up.

    pkill -INT -f pico_manager_thread_server.py
    .venv_teleop/bin/python scripts/replay_pico_input.py \
        -i ~/.sonic_recordings/walk1.picoinput \
        -- --manager --port 5556 --target_fps 50 --buffer_size 15
    # then in IsaacLab press U to unlock

Everything after `--` is passed verbatim to pico_manager (defaults to `--manager`).
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # for _pico_input_shim
from _pico_input_shim import ReplayDelegate, run_pico_manager_with, split_passthrough  # noqa: E402


def main() -> None:
    before, passthrough = split_passthrough(sys.argv[1:])
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        usage="replay_pico_input.py -i IN [--rate R] [--loop] [-- <pico_manager args>]",
    )
    p.add_argument("-i", "--in", dest="infile", type=Path, required=True, help="recording (.picoinput)")
    p.add_argument("--rate", type=float, default=1.0, help="playback speed multiplier (keep ~1.0)")
    p.add_argument("--loop", action="store_true", help="loop the timeline forever until Ctrl-C")
    p.add_argument("--grace", type=float, default=2.0, help="seconds to hold after timeline end before exit")
    args = p.parse_args(before)

    delegate = ReplayDelegate(args.infile, rate=args.rate, loop=args.loop)
    print(f"[replay] 载入 {args.infile}  时长={delegate.duration:.1f}s  变化事件: {delegate.stats()}")
    print(f"[replay] rate={args.rate}  loop={args.loop}  （确保 pico_manager 已停、deploy 在跑）")
    if not delegate.duration:
        print("[replay] ❌ 录制为空", file=sys.stderr)
        sys.exit(1)

    # Auto-exit once the timeline is played out (unless looping): send SIGINT so
    # pico_manager shuts down its threads cleanly.
    if not args.loop:
        def _auto_exit():
            time.sleep(delegate.duration / max(args.rate, 1e-6) + args.grace)
            print("\n[replay] 时间线播放完毕，结束")
            os.kill(os.getpid(), signal.SIGINT)
        threading.Thread(target=_auto_exit, daemon=True).start()

    try:
        run_pico_manager_with(delegate, passthrough)
    except KeyboardInterrupt:
        print("\n[replay] 停止")


if __name__ == "__main__":
    main()
