// test_ramulator.sv — SystemVerilog smoke test for ramulator_sv_wrapper
//
// Drives the AXI subordinate-side channels of axi_bus_if:
//   AR channel  → issue reads
//   R  channel  ← receive read data (r_i_ready tied 1)
//   AW+W channel→ issue writes (both presented simultaneously)
//   B  channel  ← receive write ack (b_i_ready tied 1)
//
// Test flow mirrors test_dpi.c:
//   [1] Init — wait for init_done
//   [2] Issue 128 requests (every 4th a write), one at a time
//   [3] Accept check  — all requests accepted
//   [4] Completion    — all reads completed
//   [5] Functional    — read data matches shadow memory / address default
//   [6] Finalize      — ramulator_exit (wrapper final block calls ramulator_finalize)
//
// Compile + run (from project root):
//   vlib work
//   vlog -sv -mfcu +acc +incdir+src/axi \
//        src/axi/axi_bus_pkg.sv src/axi/axi_bus_if.sv \
//        src/dpi_wrapper/ramulator_sv_wrapper.sv \
//        src/dpi_wrapper/test_ramulator.sv
//   vsim -c -sv_lib ./libramulator_dpi work.test_ramulator \
//        -do "run -all; quit -f"

`timescale 1ns / 1ps

module test_ramulator;

    import axi_bus_pkg::*;

    // Use ramulator_exit() instead of $finish to bypass QuestaSim's
    // post-simulation cleanup, which hits heap corruption left by Ramulator2.
    import "DPI-C" function void ramulator_exit(input int code);

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam string  CFG         = "configs/ddr4_config.yaml";
    localparam int     NUM_REQS    = 128;
    localparam int     MAX_CYCLES  = 500000;
    localparam longint ADDR_STRIDE = 64;
    localparam longint WR_PATTERN  = 64'hDEADBEEFCAFEBABE;

    // ----------------------------------------------------------------
    // Clock and reset
    // ----------------------------------------------------------------
    logic clk   = 0;
    logic rst_n = 0;

    always #5 clk = ~clk;   // 10 ns / 100 MHz

    // ----------------------------------------------------------------
    // AXI bus interface + DUT
    // ----------------------------------------------------------------
    axi_bus_if axi(.CLK(clk), .nRST(rst_n));
    logic init_done;

    ramulator_sv_wrapper #(
        .CONFIG_FILE(CFG)
    ) dut (
        .axi      (axi),
        .init_done(init_done)
    );

    // TB always accepts R and B responses
    assign axi.r_i_ready = 1'b1;
    assign axi.b_i_ready = 1'b1;

    // ----------------------------------------------------------------
    // Per-request tracking
    // ----------------------------------------------------------------
    logic [ARADDR-1:0] t_addr   [NUM_REQS];
    logic              t_type   [NUM_REQS];   // 0=read, 1=write
    logic [WDATA-1:0]  t_wrdata [NUM_REQS];

    // Shadow functional memory: addr -> last value written
    longint shadow_mem  [longint];
    bit     was_written [longint];

    // Counters (shared between tasks and main)
    int accepted_r  = 0;
    int accepted_w  = 0;
    int completed_r = 0;
    int completed_w = 0;
    int func_ok     = 0;
    int func_fail   = 0;

    // ----------------------------------------------------------------
    // Task: issue one AXI read, wait for R response, check data
    // ----------------------------------------------------------------
    task automatic issue_read(input int idx);
        longint exp_data;
        longint got_data;

        // Drive AR channel
        axi.ar_o_valid  = 1'b1;
        axi.ar_o.addr   = t_addr[idx];
        axi.ar_o.mid_id = MID_ARID'(idx);
        axi.ar_o.size   = 3'b011;    // 8-byte beat
        axi.ar_o.len    = '0;        // 1 beat
        axi.ar_o.burst  = 2'b01;     // INCR

        // Wait for ar_o_ready handshake (retry if Ramulator queue full)
        @(posedge clk); #1;
        while (!axi.ar_o_ready) begin
            @(posedge clk); #1;
        end
        axi.ar_o_valid = 1'b0;
        accepted_r++;

        // Wait for R response (r_i_ready=1, so one-cycle pulse)
        while (!axi.r_i_valid) begin
            @(posedge clk); #1;
        end
        got_data = longint'(axi.r_i.data);
        completed_r++;

        // Functional check
        if (was_written.exists(longint'(t_addr[idx])))
            exp_data = shadow_mem[longint'(t_addr[idx])];
        else
            exp_data = longint'(t_addr[idx]);   // wrapper default: returns addr

        if (got_data === exp_data) begin
            func_ok++;
        end else begin
            $display("  [FUNC MISMATCH] idx=%0d addr=0x%h  got=0x%h  expected=0x%h",
                     idx, t_addr[idx], got_data, exp_data);
            func_fail++;
        end
    endtask

    // ----------------------------------------------------------------
    // Task: issue one AXI write (AW+W together), wait for B response
    // ----------------------------------------------------------------
    task automatic issue_write(input int idx);
        // Drive AW and W channels simultaneously (wrapper requires both)
        axi.aw_o_valid  = 1'b1;
        axi.aw_o.addr   = t_addr[idx];
        axi.aw_o.mid_id = MID_AWID'(idx);
        axi.aw_o.size   = 3'b011;
        axi.aw_o.len    = '0;
        axi.aw_o.burst  = 2'b01;

        axi.w_o_valid   = 1'b1;
        axi.w_o.data    = t_wrdata[idx];
        axi.w_o.mid_id  = MID_ARID'(idx);
        axi.w_o.last    = 1'b1;
        axi.w_o.strb    = '1;

        // Wait for AW handshake (w_o_ready pulses simultaneously)
        @(posedge clk); #1;
        while (!axi.aw_o_ready) begin
            @(posedge clk); #1;
        end
        axi.aw_o_valid = 1'b0;
        axi.w_o_valid  = 1'b0;
        accepted_w++;

        // Update shadow memory
        shadow_mem[longint'(t_addr[idx])]  = longint'(t_wrdata[idx]);
        was_written[longint'(t_addr[idx])] = 1;

        // Wait for B response (b_i_ready=1; wrapper sets b_pending same cycle
        // as aw_o_ready, so b_i_valid is already 1 here in most cases)
        while (!axi.b_i_valid) begin
            @(posedge clk); #1;
        end
        completed_w++;
    endtask

    // ----------------------------------------------------------------
    // Main test flow
    // ----------------------------------------------------------------
    int num_reads;
    int num_writes;

    initial begin : main_test
        // Quiesce all master-side AXI outputs
        axi.ar_o_valid = 1'b0;
        axi.ar_o       = '0;
        axi.aw_o_valid = 1'b0;
        axi.aw_o       = '0;
        axi.w_o_valid  = 1'b0;
        axi.w_o        = '0;

        // Reset sequence
        rst_n = 1'b0;
        repeat(4) @(posedge clk);
        rst_n = 1'b1;

        // ----------------------------------------------------------
        // [1] Wait for init_done
        // ----------------------------------------------------------
        $display("=== Ramulator AXI SV wrapper smoke test ===");
        $display("Config: %s\n", CFG);
        $write("[1] ramulator_init ... ");
        wait(init_done);
        @(posedge clk);
        $display("OK");

        // Build request list
        num_reads  = 0;
        num_writes = 0;
        for (int i = 0; i < NUM_REQS; i++) begin
            t_addr[i]   = ARADDR'(longint'(i) * ADDR_STRIDE);
            t_type[i]   = (i % 4 == 0) ? 1'b1 : 1'b0;
            t_wrdata[i] = longint'(t_addr[i]) ^ WR_PATTERN;
            if (t_type[i]) num_writes++;
            else           num_reads++;
        end

        // ----------------------------------------------------------
        // [2] Issue all requests (one at a time; sequential for
        //     simplicity — one outstanding read at a time lets the
        //     TB match each R response to its request by position)
        // ----------------------------------------------------------
        $display("[2] Issuing %0d requests (%0d reads, %0d writes) ...",
                 NUM_REQS, num_reads, num_writes);

        for (int i = 0; i < NUM_REQS; i++) begin
            if (t_type[i])
                issue_write(i);
            else
                issue_read(i);
        end

        // ----------------------------------------------------------
        // Print results
        // ----------------------------------------------------------
        $display("\n--- Results ---");
        $display("  Reads  accepted / completed : %0d / %0d  (of %0d)",
                 accepted_r, completed_r, num_reads);
        $display("  Writes accepted / completed : %0d / %0d  (of %0d)",
                 accepted_w, completed_w, num_writes);
        $display("  Functional checks OK / FAIL : %0d / %0d  (of %0d reads)",
                 func_ok, func_fail, completed_r);

        // [3] Accept check
        $write("\n[3] ramulator_send_request ... ");
        if (accepted_r == num_reads && accepted_w == num_writes)
            $display("OK (all %0d accepted)", NUM_REQS);
        else begin
            $display("FAIL (reads %0d/%0d, writes %0d/%0d)",
                     accepted_r, num_reads, accepted_w, num_writes);
            ramulator_exit(1);
        end

        // [4] Completion check
        $write("[4] ramulator_check_response ... ");
        if (completed_r == num_reads)
            $display("OK (%0d read completions, %0d write acks)", completed_r, completed_w);
        else begin
            $display("FAIL - only %0d / %0d reads completed", completed_r, num_reads);
            ramulator_exit(1);
        end

        // [5] Functional check
        $write("[5] Functional model ... ");
        if (func_fail == 0 && func_ok == completed_r)
            $display("OK (all %0d reads returned correct data)", func_ok);
        else begin
            $display("FAIL - %0d / %0d reads returned wrong data", func_fail, completed_r);
            ramulator_exit(1);
        end

        // [6] Done
        $display("[6] ramulator_finalize ... OK (called in wrapper final block)");
        $display("\n=== Smoke test PASSED ===");
        ramulator_exit(0);
    end

    // ----------------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------------
    initial begin : watchdog
        repeat(MAX_CYCLES) @(posedge clk);
        $display("WATCHDOG: simulation exceeded %0d cycles — aborting.", MAX_CYCLES);
        ramulator_exit(1);
    end

endmodule
