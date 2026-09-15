# tests/legacy

The tools exactly as they were before the refactor, on `scservo_sdk` and three copies of
`_common.py`. **Nothing ships from here and nothing imports it except the test suite.**

They are kept because they are the specification. `servotools` was extracted from this code
while it was known to work on real hardware, and `tests/test_bus_equivalence.py` and
`tests/test_split_equivalence.py` drive both implementations against identically seeded fake
buses and assert the wire traffic, printed output and resulting servo state all match. Delete
these files and that proof stops being runnable.

`chain_autonumber.py` also preserves the voltage-sag chain-ordering implementation
(`measure_order`, `measure_boot`) and its CLI, which did not survive into the package — the
approach was measured against ground truth and was not reliable enough to ship. See
`experiments/README.md`.

Frozen: not linted, not collected by pytest, not packaged, not fixed.

To re-run the migration comparison:

```bash
python tests/capture.py --impl legacy --check --diff
```

The golden records now track the current implementation, so this reports the differences
listed in the "Accept the intentional behavior changes" commit. That is expected.
