#!/usr/bin/env bash
# run_questa.sh — launch the Ramulator DPI-C SV smoke test in QuestaSim
# Must be run from the project root (atalla-ramulator2/).
#
# Usage:
#   cd atalla-ramulator2
#   bash sim/run_questa.sh
#
# Optional: set QUESTA_HOME if vsim is not already in PATH.
#   QUESTA_HOME=/path/to/questasim bash sim/run_questa.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT}"

# ----------------------------------------------------------------
# Locate vsim
# ----------------------------------------------------------------
if [[ -n "${QUESTA_HOME:-}" ]]; then
    export PATH="${QUESTA_HOME}/bin:${PATH}"
fi

# Fallback: known install location on this machine
if ! command -v vsim &>/dev/null; then
    QUESTA_BIN="/package/eda/mg/questa2021.4/questasim/linux_x86_64"
    if [[ -x "${QUESTA_BIN}/vsim" ]]; then
        export PATH="${QUESTA_BIN}:${PATH}"
    else
        echo "ERROR: vsim not found. Set QUESTA_HOME or add QuestaSim to PATH." >&2
        exit 1
    fi
fi

echo "Using vsim: $(command -v vsim)"
echo "Working dir: ${ROOT}"

# ----------------------------------------------------------------
# The .so files live in the project root; tell the linker where
# ----------------------------------------------------------------
export LD_LIBRARY_PATH="${ROOT}:${LD_LIBRARY_PATH:-}"

# ----------------------------------------------------------------
# Run
# ----------------------------------------------------------------
vsim -c -do sim/run_vsim.do
