# waves/test_ramulator.do
# Wave setup for test_ramulator — sourced by run_sv_test.sh in GUI mode.

view wave
view structure
view objects

env /test_ramulator
log -r /*

add wave -divider "Clock / Reset"
add wave    /test_ramulator/clk
add wave    /test_ramulator/nrst
add wave    /test_ramulator/init_done

add wave -divider "AR channel (read address)"
add wave    /test_ramulator/axi/ar_o_valid
add wave    /test_ramulator/axi/ar_o_ready
add wave -hex /test_ramulator/axi/ar_o

add wave -divider "R channel (read data)"
add wave    /test_ramulator/axi/r_i_valid
add wave    /test_ramulator/axi/r_i_ready
add wave -hex /test_ramulator/axi/r_i

add wave -divider "AW channel (write address)"
add wave    /test_ramulator/axi/aw_o_valid
add wave    /test_ramulator/axi/aw_o_ready
add wave -hex /test_ramulator/axi/aw_o

add wave -divider "W channel (write data)"
add wave    /test_ramulator/axi/w_o_valid
add wave    /test_ramulator/axi/w_o_ready
add wave -hex /test_ramulator/axi/w_o

add wave -divider "B channel (write response)"
add wave    /test_ramulator/axi/b_i_valid
add wave    /test_ramulator/axi/b_i_ready
add wave -hex /test_ramulator/axi/b_i

add wave -divider "Scoreboard"
add wave -dec /test_ramulator/wr_acc
add wave -dec /test_ramulator/wr_cmp
add wave -dec /test_ramulator/rd_acc
add wave -dec /test_ramulator/rd_cmp
add wave -dec /test_ramulator/func_ok
add wave -dec /test_ramulator/func_fail
