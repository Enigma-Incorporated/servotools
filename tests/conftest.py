import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for p in (HERE, ROOT / "src", ROOT, HERE / "legacy"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import pytest  # noqa: E402
import scenarios  # noqa: E402
from clock import VirtualClock  # noqa: E402
from fakebus import FakeBus, FakeSerial  # noqa: E402


@pytest.fixture
def clock():
    c = VirtualClock()
    with c:
        yield c


def make_bus(clock, scenario="healthy", seed=7):
    bus = FakeBus(scenarios.ALL[scenario](), clock, seed=seed)
    FakeSerial.bus = bus
    return bus


@pytest.fixture
def bus_factory(clock, monkeypatch):
    import serial
    monkeypatch.setattr(serial, "Serial", FakeSerial)

    created = []

    def factory(scenario="healthy", seed=7):
        b = make_bus(clock, scenario, seed)
        created.append(b)
        return b

    yield factory
    FakeSerial.bus = None
