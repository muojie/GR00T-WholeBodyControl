#!/usr/bin/env python3
"""Record the headset input pico_manager_thread_server.py reads from XRoboToolkit.

Runs pico_manager in-process with a recording shim standing in for xrobotoolkit_sdk,
so every headset read (body joint poses, device timestamp, A/B/X/Y buttons, triggers,
grips, joysticks) is captured while the live closed loop runs normally. Replay it
later with replay_pico_input.py to drive pico_manager with no headset attached.

Needs the real SDK + a connected headset, so run it with the teleop venv IN PLACE OF
the launcher's pico_manager pane:

    pkill -INT -f pico_manager_thread_server.py        # stop the launcher's copy
    .venv_teleop/bin/python scripts/record_pico_input.py \
        -o ~/.sonic_recordings/walk1.picoinput \
        -- --manager --port 5556 --target_fps 50 --buffer_size 15
    # do A+B+X+Y + the motion, then Ctrl-C here

Everything after `--` is passed verbatim to pico_manager (defaults to `--manager`).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # for _pico_input_shim
from _pico_input_shim import RecordingDelegate, run_pico_manager_with, split_passthrough  # noqa: E402


def main() -> None:
    before, passthrough = split_passthrough(sys.argv[1:])
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        usage="record_pico_input.py -o OUT [-- <pico_manager args>]",
    )
    p.add_argument("-o", "--out", type=Path, required=True, help="output recording file (.picoinput)")
    args = p.parse_args(before)
    args.out.parent.mkdir(parents=True, exist_ok=True)

    try:
        import xrobotoolkit_sdk as real_xrt  # the REAL SDK (needs headset + teleop venv)
    except ImportError as e:
        print(f"[record] ❌ 无法 import xrobotoolkit_sdk: {e}\n"
              f"        用 .venv_teleop/bin/python 跑，且已 install_pico.sh。", file=sys.stderr)
        sys.exit(2)

    delegate = RecordingDelegate(real_xrt, args.out)
    print(f"[record] 录制 pico_manager 头显输入 -> {args.out}")
    print("[record] 现在做动作（A+B+X+Y 校准+CONTROL），完成后 Ctrl-C 停止")
    try:
        run_pico_manager_with(delegate, passthrough)
    except KeyboardInterrupt:
        print("\n[record] 收到 Ctrl-C，停止录制")
    finally:
        delegate.close()
        print(f"[record] 完成 -> {args.out}")


if __name__ == "__main__":
    main()
