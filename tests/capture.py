"""Run a tool against a fake bus and render its behavior as a golden record.

    python tests/capture.py                 # rewrite the records from the legacy tools
    python tests/capture.py --check         # compare
    python tests/capture.py --impl new --check   # does the refactor still match?

See docs/testing.md.
"""

from __future__ import annotations

import argparse
import builtins
import contextlib
import importlib
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for p in (HERE, ROOT / "src", ROOT, HERE / "legacy"):
    sys.path.insert(0, str(p))

import scenarios  # noqa: E402
import serial  # noqa: E402
from clock import VirtualClock  # noqa: E402
from fakebus import FakeBus, FakeSerial  # noqa: E402
from serial.tools import list_ports  # noqa: E402

FAKE_PORT = "/dev/ttyFAKE0"
INST = {0x01: "PING", 0x02: "READ", 0x03: "WRITE", 0x08: "REBOOT"}


class FakePortInfo:
    device = FAKE_PORT
    vid = 0x1A86
    pid = 0x55D3
    description = "fake adapter"


class FakeDatetime:
    """Deterministic clock for tools that stamp their output."""

    _n = 0

    @classmethod
    def now(cls, tz=None):
        cls._n += 1
        return cls

    @classmethod
    def astimezone(cls, tz=None):
        return cls

    @classmethod
    def isoformat(cls, timespec=None):
        return f"2024-01-01T00:00:{cls._n:02d}+00:00"


def _fresh_import(module_name: str):
    for name in (module_name, "chain_autonumber", "_common"):
        sys.modules.pop(name, None)
    return importlib.import_module(module_name)


def legacy_module(module_name: str, argv: list[str]):
    """Call a pre-refactor script's main() with the given argv."""
    def go():
        mod = _fresh_import(module_name)
        if hasattr(mod, "datetime"):
            mod.datetime = FakeDatetime
        sys.argv = [module_name + ".py", *argv]
        mod.main()
    go.argv = argv
    return go


def new_cli(argv: list[str]):
    """Invoke the `servo` CLI in-process."""
    def go():
        import servotools.commands.watch as watch_mod
        from servotools import cli
        watch_mod.datetime = FakeDatetime
        raise SystemExit(cli.main(argv))
    go.argv = argv
    return go


def run(entry, scenario: str, *, max_writes: int | None = None, setup=None,
        seed: int = 7, no_adapter: bool = False) -> dict:
    """Execute one tool end to end and collect everything observable."""
    clock = VirtualClock()
    bus = FakeBus(scenarios.ALL[scenario](), clock, seed=seed)
    bus.max_writes = max_writes
    if setup:
        setup(bus, clock)

    real_serial, real_comports, real_input = serial.Serial, list_ports.comports, builtins.input
    FakeSerial.bus = bus
    serial.Serial = FakeSerial
    list_ports.comports = (lambda: []) if no_adapter else (lambda: [FakePortInfo()])
    builtins.input = lambda prompt="": print(prompt, end="") or ""

    out = io.StringIO()
    code = 0
    try:
        with clock, contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            try:
                entry()
            except SystemExit as e:
                code = e.code if isinstance(e.code, int) else 0
            except KeyboardInterrupt:
                out.write("\n[capture] stopped at the write budget\n")
    finally:
        serial.Serial, list_ports.comports, builtins.input = real_serial, real_comports, real_input
        FakeSerial.bus = None

    return {"argv": getattr(entry, "argv", []), "exit": code, "stdout": out.getvalue(),
            "ops": bus.oplog, "state": bus.state()}


def render(name: str, rec: dict, invocation: str) -> str:
    lines = [f"# {name}", f"$ {invocation}", "",
             "--- exit ---", str(rec["exit"]), "", "--- stdout ---",
             rec["stdout"].rstrip("\n"), "", "--- ops ---"]
    for baud, sid, inst, params in rec["ops"]:
        label = INST.get(inst, f"0x{inst:02X}")
        detail = ""
        if label == "READ" and len(params) >= 2:
            detail = f" addr={params[0]} len={params[1]}"
        elif label == "WRITE" and params:
            detail = f" addr={params[0]} data={list(params[1:])}"
        lines.append(f"{baud} id={sid} {label}{detail}")
    lines += ["", "--- servos ---"]
    for st in rec["state"]:
        lines.append(" ".join(f"{k}={v}" for k, v in st.items()))
    return "\n".join(lines) + "\n"


def legacy_split():
    ca = _fresh_import("chain_autonumber")
    ser = serial.Serial(FAKE_PORT, ca.BAUD, timeout=0.006)
    ids, leftover = ca.split_all(ser)
    print(f"\nresult: ids={ids} leftover={leftover}")
    ser.close()


def new_split():
    from servotools.bus import Bus
    from servotools.split import split_all
    bus = Bus.open(FAKE_PORT, 1_000_000, 0.006)
    ids, leftover = split_all(bus)
    print(f"\nresult: ids={ids} leftover={leftover}")
    bus.close()


MAPPING = {88: 1, 61: 2, 45: 3, 23: 4, 7: 5, 1: 6}


def legacy_renumber():
    ca = _fresh_import("chain_autonumber")
    ser = serial.Serial(FAKE_PORT, ca.BAUD, timeout=0.006)
    print(f"renumber ok={ca.renumber(ser, dict(MAPPING))}")
    ser.close()


def new_renumber():
    from servotools.bus import Bus
    from servotools.renumber import renumber
    bus = Bus.open(FAKE_PORT, 1_000_000, 0.006)
    print(f"renumber ok={renumber(bus, dict(MAPPING))}")
    bus.close()


def _drops(*ids):
    def setup(b, c):
        for sid in ids:
            b.schedule_silence(c.now() + 0.20, sid)
            b.schedule_silence(c.now() + 0.60, sid, False)
    return setup


# name -> (legacy entry, new entry, scenario, options)
CASES = {
    "health.healthy": (legacy_module("health_check", []), new_cli(["health"]), "healthy", {}),
    "health.missing": (legacy_module("health_check", []), new_cli(["health"]), "missing", {}),
    "health.hot": (legacy_module("health_check", []), new_cli(["health"]), "hot", {}),
    "health.misconfigured": (legacy_module("health_check", []), new_cli(["health"]),
                             "misconfigured", {}),
    "health.empty": (legacy_module("health_check", []), new_cli(["health"]), "empty", {}),
    "health.no_adapter": (legacy_module("health_check", []), new_cli(["health"]), "empty",
                          {"no_adapter": True}),

    "scan.quick": (legacy_module("scan_bus", ["--quick"]), new_cli(["scan", "--quick"]),
                   "healthy", {}),
    "scan.one_baud": (legacy_module("scan_bus", ["--baud", "1000000"]),
                      new_cli(["scan", "--baud", "1000000"]), "scattered", {}),
    "scan.all_bauds": (legacy_module("scan_bus", []), new_cli(["scan"]), "other_baud", {}),
    "scan.empty": (legacy_module("scan_bus", ["--quick"]), new_cli(["scan", "--quick"]),
                   "empty", {}),

    "defaults.healthy": (legacy_module("configure_defaults", []), new_cli(["defaults"]),
                         "healthy", {}),
    "defaults.misconfigured": (legacy_module("configure_defaults", []), new_cli(["defaults"]),
                               "misconfigured", {}),
    "defaults.missing": (legacy_module("configure_defaults", []), new_cli(["defaults"]),
                         "missing", {}),

    "torque.off": (legacy_module("torque_off", []), new_cli(["torque", "off"]), "healthy", {}),
    "torque.on": (legacy_module("torque_off", ["--on"]), new_cli(["torque", "on"]),
                  "healthy", {}),

    "setid.fresh": (legacy_module("set_servo_id", ["4"]), new_cli(["set-id", "4"]), "single", {}),
    "setid.absent": (legacy_module("set_servo_id", ["4"]), new_cli(["set-id", "4"]), "empty", {}),

    "watch.stable": (legacy_module("watch_chain", []), new_cli(["watch"]), "healthy",
                     {"max_writes": 40}),
    "watch.dropout": (legacy_module("watch_chain", []), new_cli(["watch"]), "healthy",
                      {"max_writes": 120, "setup": _drops(5, 6)}),

    "flatten.scan_only": (legacy_module("flatten_arm", ["--scan-only"]),
                          new_cli(["flatten", "--scan-only"]), "scattered", {}),
    "flatten.apply": (legacy_module("flatten_arm", ["--yes"]), new_cli(["flatten", "--yes"]),
                      "scattered", {}),
    "flatten.apply_wide": (legacy_module("flatten_arm", ["--yes", "--max-id", "90"]),
                           new_cli(["flatten", "--yes", "--max-id", "90"]), "scattered", {}),

    "split.collided": (legacy_split, new_split, "collided", {}),
    "split.degenerate": (legacy_split, new_split, "collided_degenerate", {}),
    "split.nothing_to_do": (legacy_split, new_split, "scattered", {}),
    "renumber.scattered": (legacy_renumber, new_renumber, "scattered", {}),

    "autonumber.scan_only": (legacy_module("arm_autonumber", ["--scan-only"]),
                             new_cli(["autonumber", "--scan-only"]), "scattered", {}),
    "autonumber.scan_collided": (legacy_module("arm_autonumber", ["--scan-only"]),
                                 new_cli(["autonumber", "--scan-only"]), "collided", {}),
    "autonumber.apply": (legacy_module("arm_autonumber", ["--apply", "88,61,45,23,7,1", "--yes"]),
                         new_cli(["autonumber", "--apply", "88,61,45,23,7,1", "--yes"]),
                         "scattered", {}),
    "autonumber.single": (legacy_module("arm_autonumber", ["--yes"]),
                          new_cli(["autonumber", "--yes"]), "single", {}),
    "autonumber.split": (legacy_module("arm_autonumber", ["--scan-only"]),
                         new_cli(["autonumber", "--scan-only"]), "collided_degenerate", {}),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(HERE / "golden"))
    ap.add_argument("--only", default=None, help="substring filter on the case name")
    ap.add_argument("--impl", choices=["legacy", "new"], default="legacy")
    ap.add_argument("--check", action="store_true", help="compare instead of writing")
    ap.add_argument("--diff", action="store_true", help="print a unified diff on mismatch")
    args = ap.parse_args()

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    failures = []
    for name, (legacy, new, scenario, opts) in CASES.items():
        if args.only and args.only not in name:
            continue
        entry = legacy if args.impl == "legacy" else new
        cli_argv = getattr(new, "argv", None)
        invocation = f"servo {' '.join(cli_argv)}" if cli_argv else f"({name} library call)"
        text = render(name, run(entry, scenario, **opts), invocation)
        path = outdir / f"{name}.txt"
        if args.check:
            old = path.read_text(encoding="utf-8") if path.exists() else ""
            if old != text:
                failures.append(name)
                print(f"CHANGED {name}")
                if args.diff:
                    import difflib
                    sys.stdout.writelines(difflib.unified_diff(
                        old.splitlines(True), text.splitlines(True),
                        fromfile=f"golden/{name}", tofile=f"{args.impl}/{name}", n=2))
            else:
                print(f"ok      {name}")
        else:
            path.write_text(text, encoding="utf-8")
            print(f"wrote   {name}  ({len(text)} bytes)")
    if args.check and failures:
        print(f"\n{len(failures)} record(s) changed: {', '.join(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
