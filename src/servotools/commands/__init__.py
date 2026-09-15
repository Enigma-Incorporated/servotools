"""One module per `servo` subcommand."""

from . import (
    autonumber,
    defaults,
    flatten,
    health,
    scan,
    setid,
    torque,
    watch,
    wiggle,
)

COMMANDS = [health, scan, watch, torque, defaults, setid, autonumber, flatten, wiggle]
