"""Module-side pytest bridge into trunk's system-test harness (RFC #379 §5).

The test modules under this directory are trunk's OptiTrack tests, moved here
verbatim during extraction. They import trunk's harness surface
(``from conftest import ...``, ``from system.test_fixed_trajectory import ...``)
and are designed to run through trunk's suite, never copied from it:

- **CI**: trunk's reusable ``module-system-tests.yml`` appends this directory
  to a pytest invocation rooted at trunk's ``tests/`` (the ``module_tests_dir``
  input), so trunk's ``tests/conftest.py`` is loaded as the root conftest and
  drives options, fixtures, and metrics. This file then only tops up
  ``sys.path`` (idempotent inserts).

- **Local / standalone**: point ``AIRSTACK_ROOT`` at an AirStack checkout and
  run pytest against this directory directly::

      AIRSTACK_ROOT=~/AirStack python3 -m pytest --collect-only -q tests/

  This conftest then puts trunk's ``tests/`` dir on ``sys.path`` (for
  ``harness`` and ``system.*`` imports) and loads trunk's ``tests/conftest.py``
  under the module name ``conftest`` so ``from conftest import <name>``
  resolves to trunk's helper API. Trunk's pytest *hooks* are deliberately not
  registered this way — importable names only. For executing the system tests
  (which need trunk's addoption/fixtures), run through trunk::

      cd $AIRSTACK_ROOT && pytest tests/ modules/asm_optitrack/tests/ \
          -c tests/pytest.ini -m optitrack

- **Without AIRSTACK_ROOT**: collection is skipped cleanly with a message
  instead of failing on unresolvable trunk imports.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

_MODULE_ROOT = Path(__file__).resolve().parents[1]
_EMULATOR_ROOT = _MODULE_ROOT / "exts" / "optitrack.natnet.emulator"


def _path_insert(path: Path) -> None:
    p = str(path)
    if path.is_dir() and p not in sys.path:
        sys.path.insert(0, p)


# The emulator package (`optitrack.natnet.emulator`) and its sibling test
# helpers (`natnet_test_helpers`). In a trunk-rooted run tests/conftest.py
# inserts these from tests/colcon_unit_test_packages.yaml; standalone runs get
# them here. Idempotent either way.
_path_insert(_EMULATOR_ROOT)
_path_insert(_EMULATOR_ROOT / "test")

_AIRSTACK_ROOT = os.environ.get("AIRSTACK_ROOT", "")
_TRUNK_TESTS = Path(_AIRSTACK_ROOT).expanduser() / "tests" if _AIRSTACK_ROOT else None

_SKIP_MESSAGE = (
    "asm_optitrack tests skipped: AIRSTACK_ROOT is not set. These tests run "
    "against trunk's system-test harness — set AIRSTACK_ROOT to an AirStack "
    "checkout (or run via trunk's reusable module-system-tests.yml workflow, "
    "which does this for you)."
)

if _TRUNK_TESTS is not None and (_TRUNK_TESTS / "conftest.py").is_file():
    # `from harness import ...` and `from system.test_fixed_trajectory import ...`
    # (trunk's conftest also self-inserts this dir, but only once it is loaded).
    _path_insert(_TRUNK_TESTS)

    _existing = sys.modules.get("conftest")
    if _existing is None or getattr(_existing, "__file__", None) == __file__:
        # Standalone run: pytest imports THIS file as module "conftest"
        # (prepend import mode), so the test modules' `from conftest import
        # <harness name>` resolves here. Load trunk's tests/conftest.py under
        # an alias and re-export its public surface (trunk re-exports
        # `from harness import *`) into this module's globals. pytest_* hooks
        # are deliberately NOT re-exported: registering trunk's hooks from
        # here would double-drive them in a trunk-rooted run and write run
        # dirs into the trunk checkout in a standalone one — standalone runs
        # are for collection/import checks; execute the system tests through
        # trunk (see module docstring).
        _spec = importlib.util.spec_from_file_location(
            "airstack_trunk_conftest", _TRUNK_TESTS / "conftest.py"
        )
        assert _spec is not None and _spec.loader is not None
        _trunk_conftest = importlib.util.module_from_spec(_spec)
        sys.modules["airstack_trunk_conftest"] = _trunk_conftest
        _spec.loader.exec_module(_trunk_conftest)
        globals().update(
            {
                _name: _value
                for _name, _value in vars(_trunk_conftest).items()
                if not _name.startswith(("_", "pytest_"))
            }
        )
else:
    collect_ignore_glob = ["*"]

    def pytest_report_header(config):  # noqa: ARG001 - pytest hook signature
        return _SKIP_MESSAGE

    sys.stderr.write(_SKIP_MESSAGE + "\n")
