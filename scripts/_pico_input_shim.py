"""Shared record/replay shim for pico_manager_thread_server.py's headset input.

pico_manager reads the PICO headset only through ``import xrobotoolkit_sdk as xrt``
(a module-level import with an ``xrt = None`` fallback). By injecting a stand-in
module into ``sys.modules['xrobotoolkit_sdk']`` *before* pico_manager imports it, we
can capture every headset read it makes (body joint poses, device timestamp, A/B/X/Y
buttons, triggers, grips, joysticks) and later feed those exact reads back with no
headset attached -- pico_manager runs its full pipeline (calibration at the recorded
A+B+X+Y instant, SMPL conversion) and publishes to deploy exactly as it did live.

Two poller threads read xrt at different, unsynchronized rates (PicoReader for body
data; the consumer loop for controller getters), so a sequential call log would
desync on replay. Instead we store, per function, the value *timeline* (logging only
when the return value changes) and on replay return the latest value at-or-before the
current wall-clock time. That matches the "read latest sensor value" semantics and is
robust to poll-rate differences between record and replay.
"""

from __future__ import annotations

import bisect
import runpy
import sys
import threading
import time
import types
from pathlib import Path

import msgpack

FORMAT = "pico_input_v1"
_MISSING = object()


def to_serializable(v):
    """Convert numpy arrays/tuples to plain lists so msgpack can pack them.

    pico_manager consumes these via np.array(...)/indexing/bool(...), so plain
    Python lists/floats/bools are interchangeable with the SDK's native returns.
    """
    if isinstance(v, (list, tuple)):
        return [to_serializable(x) for x in v]
    if hasattr(v, "tolist"):  # numpy array / scalar
        return v.tolist()
    return v


def _default_for(name: str):
    n = name.lower()
    if "available" in n or "button" in n or "click" in n:
        return False
    if n.endswith("_ns") or "stamp" in n or "timestamp" in n:
        return 0
    if "pose" in n or "joint" in n or "axis" in n:
        return []
    if "trigger" in n or "grip" in n:
        return 0.0
    return 0.0


class _ShimModule(types.ModuleType):
    """A real ModuleType so ``import xrobotoolkit_sdk`` binds cleanly; delegates
    every attribute access to the record/replay delegate."""

    def __init__(self, name: str, delegate):
        super().__init__(name)
        self.__delegate = delegate

    def __getattr__(self, item):  # only called for attrs not set on the module
        return getattr(self.__delegate, item)


# --------------------------------------------------------------------------- record


class RecordingDelegate:
    """Wraps the real xrt: passes calls through and logs value-changes with timestamps."""

    def __init__(self, real, out_path: Path):
        self._real = real
        self._f = open(out_path, "wb")
        self._packer = msgpack.Packer(use_bin_type=True)
        self._f.write(self._packer.pack({"format": FORMAT}))
        self._last: dict[str, object] = {}
        self._t0 = None
        self._lock = threading.Lock()
        self._n = 0

    def __getattr__(self, name):
        real_attr = getattr(self._real, name)
        if not callable(real_attr):
            return real_attr

        def wrapper(*args, **kwargs):
            result = real_attr(*args, **kwargs)
            try:
                self._log(name, result)
            except Exception as e:  # never let logging break the live loop
                print(f"[record-shim] log error for {name}: {e}", file=sys.stderr)
            return result

        return wrapper

    def _log(self, name, result):
        ser = to_serializable(result)
        with self._lock:
            if self._last.get(name, _MISSING) == ser:
                return  # unchanged -> hold, don't log
            self._last[name] = ser
            now = time.monotonic()
            if self._t0 is None:
                self._t0 = now
            self._f.write(self._packer.pack([round(now - self._t0, 6), name, ser]))
            self._n += 1
            if self._n % 200 == 0:
                self._f.flush()

    def close(self):
        with self._lock:
            try:
                self._f.flush()
                self._f.close()
            except Exception:
                pass
        print(f"[record-shim] wrote {self._n} change-events")


# --------------------------------------------------------------------------- replay


class ReplayDelegate:
    """Serves recorded xrt values by wall-clock time (step-hold)."""

    TS_FUNC = "get_time_stamp_ns"

    def __init__(self, in_path: Path, rate: float = 1.0, loop: bool = False):
        self._rate = rate
        self._loop = loop
        self._timelines: dict[str, tuple[list[float], list[object]]] = {}
        self._warned: set[str] = set()
        self._load(in_path)
        self._t0 = None
        self._lock = threading.Lock()
        self.duration = max((ts[-1] for ts, _ in self._timelines.values()), default=0.0)

    def _load(self, path: Path):
        raw: dict[str, list[tuple[float, object]]] = {}
        header = None
        with open(path, "rb") as f:
            for obj in msgpack.Unpacker(f, raw=False):
                if header is None:
                    header = obj
                    if not (isinstance(obj, dict) and obj.get("format") == FORMAT):
                        raise ValueError(f"{path} 不是 pico_input 录制文件（format 不符）")
                    continue
                t_rel, name, value = obj
                raw.setdefault(name, []).append((t_rel, value))
        for name, evs in raw.items():
            evs.sort(key=lambda e: e[0])
            self._timelines[name] = ([e[0] for e in evs], [e[1] for e in evs])

    def stats(self) -> str:
        return " ".join(f"{k}={len(v[0])}" for k, v in sorted(self._timelines.items()))

    def _elapsed(self):
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        e = (now - self._t0) * self._rate
        loops = 0
        if self._loop and self.duration > 0:
            loops = int(e // self.duration)
            e = e % self.duration
        return e, loops

    def __getattr__(self, name):
        def wrapper(*args, **kwargs):
            return self._value_at(name)

        return wrapper

    def _value_at(self, name):
        e, loops = self._elapsed()
        tl = self._timelines.get(name)
        if tl is None:
            if name not in self._warned:
                self._warned.add(name)
                print(f"[replay-shim] {name} 未在录制中出现，返回默认值", file=sys.stderr)
            return _default_for(name)
        times, values = tl
        idx = bisect.bisect_right(times, e) - 1
        if idx < 0:
            idx = 0
        v = values[idx]
        # keep the device timestamp strictly advancing across loop seams
        if name == self.TS_FUNC and self._loop and loops and self.duration > 0:
            try:
                v = int(v) + int(loops * self.duration * 1e9)
            except (TypeError, ValueError):
                pass
        return v


# --------------------------------------------------------------------------- runner


def split_passthrough(argv: list[str]) -> tuple[list[str], list[str]]:
    """Split argv on the first standalone '--': (before, after)."""
    if "--" in argv:
        i = argv.index("--")
        return argv[:i], argv[i + 1:]
    return argv, []


def pico_manager_path() -> Path:
    return Path(__file__).resolve().parents[1] / "gear_sonic" / "scripts" / "pico_manager_thread_server.py"


def run_pico_manager_with(delegate, passthrough: list[str]) -> None:
    """Inject the shim and run pico_manager's __main__ in-process via runpy."""
    pm = pico_manager_path()
    if not pm.exists():
        raise FileNotFoundError(f"找不到 pico_manager: {pm}")
    sys.modules["xrobotoolkit_sdk"] = _ShimModule("xrobotoolkit_sdk", delegate)
    sys.argv = [str(pm)] + (passthrough or ["--manager"])
    print(f"[shim] 运行 pico_manager: argv={sys.argv[1:]}")
    runpy.run_path(str(pm), run_name="__main__")
