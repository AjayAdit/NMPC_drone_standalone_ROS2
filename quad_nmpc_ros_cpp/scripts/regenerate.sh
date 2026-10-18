#!/usr/bin/env bash
# Re-emit acados_generated/ from the Python model.
#
#   ./scripts/regenerate.sh
#
# WHY THIS EXISTS
#
# acados's code generator is acados_template, which is Python-only. This C++
# package can link the generated solver but cannot produce it, so the generated
# C is committed -- that is what lets a clean clone build without running the
# Python node first.
#
# The cost of committing it is that it goes stale the moment the model or the
# cost weights change in nmpc_controller.py. Run this after any such change,
# and commit the result.

set -euo pipefail

PKG="$(cd "$(dirname "$0")/.." && pwd)"
PY_PKG="$(cd "$PKG/../quad_nmpc_ros" && pwd)"
OUT="$PKG/acados_generated"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "python package : $PY_PKG"
echo "output         : $OUT"

PYTHONPATH="$PY_PKG:${PYTHONPATH:-}" python3 - "$TMP" "$PY_PKG" <<'PY'
import sys
import mujoco
from quad_nmpc_ros.nmpc_controller import NMPC

export_dir, py_pkg = sys.argv[1], sys.argv[2]
model = mujoco.MjModel.from_xml_path(f'{py_pkg}/worlds/bare_world.xml')
NMPC.from_mujoco_model(model, export_dir=export_dir)
print('generated')
PY

SRC="$TMP/c_generated_code"
[ -d "$SRC" ] || { echo "no $SRC -- generation did not run"; exit 1; }

rm -rf "$OUT"
cp -r "$SRC" "$OUT"

# main_*.c carry their own main(); the sim solver is a separate integrator we
# do not link. Both would only fight the ROS node at link time.
rm -f "$OUT"/main_*.c "$OUT"/acados_sim_solver_*.c "$OUT"/acados_sim_solver_*.h

# The Python generator also compiles what it emits. Those objects and the
# shared library are machine-specific; colcon rebuilds them from the .c anyway.
find "$OUT" \( -name '*.o' -o -name '*.so' \) -delete

echo
echo "wrote $(find "$OUT" -name '*.c' | wc -l) .c files to $OUT"
echo "rebuild with:  python -m colcon build --symlink-install --packages-select quad_nmpc_ros_cpp"
echo "then commit $OUT"
