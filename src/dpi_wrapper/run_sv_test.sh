#!/bin/bash
# run_sv_test.sh — compile and run the SV smoke test with QuestaSim
#
# Usage (from project root):
#   src/dpi_wrapper/run_sv_test.sh [config.yaml]
#
# Optional argument overrides the default config (configs/ddr4_config.yaml).
# The libramulator_dpi.so must already be built (cmake + make).

set -e
cd "$(dirname "$0")/../.."   # always run from project root

CFG="${1:-configs/ddr4_config.yaml}"

echo "=== Building Ramulator shared library (if needed) ==="
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" > /dev/null 2>&1 || true
cmake --build build --target ramulator_dpi_shared -j"$(nproc)" 2>&1 | tail -5

echo ""
echo "=== Compiling SV ==="
vlib work 2>/dev/null || true
vlog -sv \
    +define+CFG=\"${CFG}\" \
    src/dpi_wrapper/ramulator_sv_wrapper.sv \
    src/dpi_wrapper/test_ramulator.sv

echo ""
echo "=== Running simulation (config: ${CFG}) ==="
vsim -c \
    -sv_lib ./libramulator_dpi \
    -G CFG="${CFG}" \
    work.test_ramulator \
    -do "run -all; quit -f"
