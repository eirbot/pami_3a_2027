#!/bin/bash
# Lance toute la suite de tests (sans robot) : Python, firmware moteurs.
# Usage : tools/run_tests.sh [--rapide]   (--rapide : saute test_strategies, ~70 s)
set -u
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." >/dev/null 2>&1 && pwd )"
cd "$ROOT"
FAIL=0

# Env uv du dépôt (pyproject.toml : numpy, OpenCV) si uv est installé, sinon python3 système
if command -v uv >/dev/null 2>&1; then PY="uv run --quiet python"; else PY="python3"; fi

echo "=== Python (test/) ==="
if [ "${1:-}" = "--rapide" ]; then
    $PY - <<'PY' || FAIL=1
import sys, unittest
suite = unittest.defaultTestLoader.discover("test", pattern="test_*.py")
def drop(s):
    out = unittest.TestSuite()
    for t in s:
        if isinstance(t, unittest.TestSuite):
            out.addTest(drop(t))
        elif "test_strategies" not in t.id():
            out.addTest(t)
    return out
sys.exit(0 if unittest.TextTestRunner(verbosity=1).run(drop(suite)).wasSuccessful() else 1)
PY
else
    $PY -m unittest discover -s test -p 'test_*.py' || FAIL=1
fi

echo "=== Firmware moteurs (pio test -e native) ==="
if command -v pio >/dev/null; then
    pio test -d ESP -e native || FAIL=1
else
    echo "pio absent : tests firmware sautés"
fi

[ $FAIL -eq 0 ] && echo "TOUS LES TESTS PASSENT" || echo "ÉCHEC"
exit $FAIL
