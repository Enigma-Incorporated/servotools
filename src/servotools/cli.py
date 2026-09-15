"""The `servo` command. See README.md."""

from __future__ import annotations

import argparse
import sys

from .bus import Bus
from .config import BAUD, PORT_TIMEOUT
from .ports import NOT_FOUND, find_port


def add_port_option(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port", default=None, help="serial port (auto-detected if omitted)")


def resolve_port(args) -> str:
    port = args.port or find_port()
    if port is None:
        print(NOT_FOUND)
        raise SystemExit(1)
    return port


def open_bus(args, baud: int = BAUD, timeout: float = PORT_TIMEOUT) -> tuple[Bus, str]:
    port = resolve_port(args)
    return Bus.open(port, baud, timeout), port


def confirm(yes: bool, message: str) -> bool:
    """Prompt unless --yes. Returns False on an explicit 'n', or with no terminal."""
    if yes:
        print(message + "  [auto]")
        return True
    try:
        return input(message).strip().lower() != "n"
    except EOFError:
        # Piped or scripted: don't assume consent for something destructive.
        print(f"{message}\n  no terminal to prompt on — pass --yes to proceed")
        return False


def build_parser() -> argparse.ArgumentParser:
    from .commands import COMMANDS

    ap = argparse.ArgumentParser(
        prog="servo",
        description="Tools for Feetech STS3215 servos on a half-duplex bus.",
        epilog="Run `servo <command> --help` for per-command options.",
    )
    ap.add_argument("--version", action="store_true", help="print the version and exit")
    sub = ap.add_subparsers(dest="command", metavar="<command>")
    for mod in COMMANDS:
        mod.add_parser(sub)
    return ap


def main(argv: list[str] | None = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.version:
        from . import __version__
        print(__version__)
        return 0
    if not args.command:
        ap.print_help()
        return 2
    return args.run(args) or 0


if __name__ == "__main__":
    sys.exit(main())
