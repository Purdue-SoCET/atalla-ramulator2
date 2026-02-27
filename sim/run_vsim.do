# run_vsim.do
# QuestaSim do-script for the Ramulator DPI-C SV smoke test.
# Run from the project root:
#   vsim -do sim/run_vsim.do
# or interactively inside the QuestaSim GUI:
#   do sim/run_vsim.do

# ----------------------------------------------------------------
# 1. Work library
# ----------------------------------------------------------------
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

# ----------------------------------------------------------------
# 2. Compile SV sources
# ----------------------------------------------------------------
vlog -sv -work work \
    src/dpi_wrapper/ramulator_sv_wrapper.sv \
    src/dpi_wrapper/tb_ramulator_dpi.sv

# ----------------------------------------------------------------
# 3. Load design
#    -sv_lib strips the "lib" prefix and ".so" suffix automatically.
# ----------------------------------------------------------------
vsim -sv_lib ./libramulator_dpi \
     -sv_lib ./libramulator \
     -t 1ps \
     work.tb_ramulator_dpi

# ----------------------------------------------------------------
# 4. Open GUI panels and set the active scope so Objects populates
# ----------------------------------------------------------------
view wave
view structure
view objects

# Set active scope to the testbench — this fills the Objects panel
env /tb_ramulator_dpi

# Log everything so you can add any signal to waves after the run
log -r /*

# ----------------------------------------------------------------
# 5. Pre-populate the Wave window with key signals
# ----------------------------------------------------------------
add wave -divider "Clock / Reset"
add wave    /tb_ramulator_dpi/clk
add wave    /tb_ramulator_dpi/rst_n
add wave    /tb_ramulator_dpi/init_done
add wave -divider "Request"
add wave    /tb_ramulator_dpi/req_valid
add wave    /tb_ramulator_dpi/req_ready
add wave -hex /tb_ramulator_dpi/req_addr
add wave    /tb_ramulator_dpi/req_type
add wave -divider "Response"
add wave    /tb_ramulator_dpi/resp_valid
add wave -hex /tb_ramulator_dpi/resp_addr
add wave -divider "Scoreboard"
add wave -dec /tb_ramulator_dpi/inflight
add wave -dec /tb_ramulator_dpi/total_completed

# ----------------------------------------------------------------
# 6. Run
# ----------------------------------------------------------------
run -all
