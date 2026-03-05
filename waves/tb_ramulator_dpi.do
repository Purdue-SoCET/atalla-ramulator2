# waves/tb_ramulator_dpi.do
# Wave setup for tb_ramulator_dpi — sourced by run_questa.sh in GUI mode.

view wave
view structure
view objects

# Set active scope so Objects panel populates
env /tb_ramulator_dpi

# Log everything so you can add any signal to waves after the run
log -r /*

# ----------------------------------------------------------------
# Pre-populate the Wave window with key signals
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
