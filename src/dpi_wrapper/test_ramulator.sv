// test_ramulator.sv — AXI master-side smoketest for ramulator_sv_wrapper
//
// Drives the subordinate-facing channels of axi_bus_if using proper
// AXI4 valid/ready handshaking on all five channels:
//
//   AR  : drive ar_o_valid / ar_o,   await ar_o_ready    (wrapper output)
//   R   : drive r_i_ready,           await r_i_valid / r_i
//   AW  : drive aw_o_valid / aw_o,   await aw_o_ready    (simultaneous with W)
//   W   : drive w_o_valid  / w_o,    await w_o_ready
//   B   : drive b_i_ready,           await b_i_valid / b_i
//
// Test plan:
//   [1] Reset + ramulator_init
//   [2] Write phase      — NUM_WR writes, verify B ack per write
//   [3] Read-after-write — read back each written address, check vs shadow mem
//   [4] Raw reads        — read unwritten addresses, check wrapper returns addr
//   [5] Pass/fail summary

`timescale 1ns / 1ps

module test_ramulator;
    import axi_bus_pkg::*;

    // Use ramulator_exit() to bypass QuestaSim post-sim cleanup which
    // hits heap corruption left by Ramulator2 (see ramulator_dpi.cpp).
    import "DPI-C" function void ramulator_exit(input int code);

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam string  CFG        = "configs/ddr4_config.yaml";
    localparam int     NUM_WR     = 16;
    localparam int     NUM_RD_RAW = 8;      // reads to addresses never written
    localparam int     MAX_CYCLES = 500_000;
    localparam longint WR_BASE    = 64'h0000_0000;
    localparam longint RD_BASE    = 64'h0010_0000;  // separate unwritten region
    localparam longint STRIDE     = 64;             // cache-line aligned

    // ----------------------------------------------------------------
    // Clock / reset
    // ----------------------------------------------------------------
    logic clk  = 0;
    logic nrst = 0;
    always #5 clk = ~clk;  // 100 MHz

    // ----------------------------------------------------------------
    // AXI bus interface + DUT
    // ----------------------------------------------------------------
    axi_bus_if axi(.CLK(clk), .nRST(nrst));
    logic init_done;

    ramulator_sv_wrapper #(.CONFIG_FILE(CFG)) dut (
        .axi      (axi),
        .init_done(init_done)
    );

    // ----------------------------------------------------------------
    // Tick counter — increments every rising clock edge
    // ----------------------------------------------------------------
    longint tick = 0;
    always @(posedge clk) tick++;

    // ----------------------------------------------------------------
    // Scoreboard
    // ----------------------------------------------------------------
    longint shadow   [longint];      // functional model: addr → last written data
    int     wr_acc   = 0, wr_cmp   = 0;
    int     rd_acc   = 0, rd_cmp   = 0;
    int     func_ok  = 0, func_fail = 0;

    // Latency stats (cycles): write = AW valid → B ack; read = AR valid → R valid
    longint wr_lat_min = '1, wr_lat_max = 0, wr_lat_total = 0;
    longint rd_lat_min = '1, rd_lat_max = 0, rd_lat_total = 0;

    // ----------------------------------------------------------------
    // Task: AXI write
    //
    //   Drives AW + W in the same cycle (wrapper constraint: both must
    //   arrive simultaneously).  Waits for aw_o_ready handshake, then
    //   collects the B response before returning.
    // ----------------------------------------------------------------
    task automatic axi_write(
        input longint              addr,
        input logic [WDATA-1:0]    data,
        input logic [MID_AWID-1:0] mid_id
    );
        longint t0, lat;
        t0 = tick;

        // Assert AW and W channels in the same cycle
        axi.aw_o_valid  = 1'b1;
        axi.aw_o.addr   = AWADDR'(addr);
        axi.aw_o.mid_id = mid_id;
        axi.aw_o.size   = 3'b011;   // 8-byte beat
        axi.aw_o.len    = 4'h0;     // single beat
        axi.aw_o.burst  = 2'b01;    // INCR

        axi.w_o_valid   = 1'b1;
        axi.w_o.data    = data;
        axi.w_o.mid_id  = MID_ARID'(mid_id);  // sub_w_channel_t uses MID_ARID width
        axi.w_o.last    = 1'b1;
        axi.w_o.strb    = '1;       // all byte-enables asserted

        // Wait for AW ready handshake (retry each cycle if Ramulator queue full)
        @(posedge clk); #1;
        while (!axi.aw_o_ready) begin
            @(posedge clk); #1;
        end
        axi.aw_o_valid = 1'b0;
        axi.w_o_valid  = 1'b0;
        wr_acc++;
        shadow[addr] = longint'(data);

        // B response: wrapper sets b_pending same cycle as aw_o_ready,
        // so b_i_valid is already 1 here in the common case.
        axi.b_i_ready = 1'b1;
        while (!axi.b_i_valid) begin
            @(posedge clk); #1;
        end
        @(posedge clk); #1;     // complete the handshake
        axi.b_i_ready = 1'b0;
        wr_cmp++;

        lat = tick - t0;
        $display("    wr[%0d] addr=0x%08h  ticks=%0d", wr_cmp - 1, addr, lat);
        if (lat < wr_lat_min) wr_lat_min = lat;
        if (lat > wr_lat_max) wr_lat_max = lat;
        wr_lat_total += lat;
    endtask

    // ----------------------------------------------------------------
    // Task: AXI read
    //
    //   Drives AR channel, waits for handshake, then collects the R
    //   response.  Returns received data in got_data.
    // ----------------------------------------------------------------
    task automatic axi_read(
        input  longint               addr,
        input  logic [MID_ARID-1:0]  mid_id,
        output longint               got_data
    );
        longint t0, lat;
        t0 = tick;

        axi.ar_o_valid  = 1'b1;
        axi.ar_o.addr   = ARADDR'(addr);
        axi.ar_o.mid_id = mid_id;
        axi.ar_o.size   = 3'b011;
        axi.ar_o.len    = 4'h0;
        axi.ar_o.burst  = 2'b01;

        // Wait for AR ready handshake
        @(posedge clk); #1;
        while (!axi.ar_o_ready) begin
            @(posedge clk); #1;
        end
        axi.ar_o_valid = 1'b0;
        rd_acc++;

        // R response: assert ready, wait for valid (Ramulator latency = many cycles)
        axi.r_i_ready = 1'b1;
        while (!axi.r_i_valid) begin
            @(posedge clk); #1;
        end
        got_data = longint'(axi.r_i.data);
        @(posedge clk); #1;     // complete the handshake
        axi.r_i_ready = 1'b0;
        rd_cmp++;

        lat = tick - t0;
        $display("    rd[%0d] addr=0x%08h  ticks=%0d", rd_cmp - 1, addr, lat);
        if (lat < rd_lat_min) rd_lat_min = lat;
        if (lat > rd_lat_max) rd_lat_max = lat;
        rd_lat_total += lat;
    endtask

    // ----------------------------------------------------------------
    // Main test
    // ----------------------------------------------------------------
    initial begin : main
        longint got, exp;
        int     total_rd;

        // Quiesce all master-driven channels before reset
        axi.ar_o_valid = 1'b0;  axi.ar_o = '0;
        axi.aw_o_valid = 1'b0;  axi.aw_o = '0;
        axi.w_o_valid  = 1'b0;  axi.w_o  = '0;
        axi.r_i_ready  = 1'b0;
        axi.b_i_ready  = 1'b0;

        // Reset sequence
        nrst = 1'b0;
        repeat(4) @(posedge clk);
        nrst = 1'b1;

        // [1] Init
        $display("=== AXI Ramulator wrapper smoketest ===");
        $display("Config : %s", CFG);
        $write("[1] ramulator_init ... ");
        wait(init_done);
        @(posedge clk); #1;
        $display("OK");

        // [2] Write phase
        $display("[2] %0d writes (AW+W simultaneous, B ack per write) ...", NUM_WR);
        for (int i = 0; i < NUM_WR; i++) begin
            axi_write(
                WR_BASE + longint'(i) * STRIDE,
                WDATA'(64'hC0FFEE00_00000000 | longint'(i)),
                MID_AWID'(i % (1 << MID_AWID))
            );
        end
        $display("    accepted=%0d  B-acks=%0d", wr_acc, wr_cmp);

        // [3] Read-after-write: expect shadow values
        $display("[3] %0d reads (read-after-write, verify vs shadow) ...", NUM_WR);
        for (int i = 0; i < NUM_WR; i++) begin
            axi_read(
                WR_BASE + longint'(i) * STRIDE,
                MID_ARID'(i % (1 << MID_ARID)),
                got
            );
            exp = shadow[WR_BASE + longint'(i) * STRIDE];
            if (got === exp) begin
                func_ok++;
            end else begin
                $display("  [MISMATCH] i=%0d addr=0x%08h  got=0x%016h  exp=0x%016h",
                         i, WR_BASE + longint'(i) * STRIDE, got, exp);
                func_fail++;
            end
        end
        $display("    accepted=%0d  R-data=%0d  OK=%0d  FAIL=%0d",
                 rd_acc, rd_cmp, func_ok, func_fail);

        // [4] Raw reads: addresses never written; wrapper returns address as data
        $display("[4] %0d raw reads (unwritten region, expect addr as return value) ...", NUM_RD_RAW);
        for (int i = 0; i < NUM_RD_RAW; i++) begin
            axi_read(
                RD_BASE + longint'(i) * STRIDE,
                MID_ARID'(i % (1 << MID_ARID)),
                got
            );
            exp = RD_BASE + longint'(i) * STRIDE;
            if (got === exp) begin
                func_ok++;
            end else begin
                $display("  [MISMATCH] i=%0d addr=0x%08h  got=0x%016h  exp=0x%016h",
                         i, RD_BASE + longint'(i) * STRIDE, got, exp);
                func_fail++;
            end
        end
        total_rd = NUM_WR + NUM_RD_RAW;
        $display("    accepted=%0d  R-data=%0d  OK=%0d  FAIL=%0d",
                 rd_acc, rd_cmp, func_ok, func_fail);

        // [5] Summary
        $display("\n--- Summary ---");
        $display("  Total ticks        : %0d", tick);
        $display("  Writes   : accepted=%0d / %0d   B-acks=%0d / %0d",
                 wr_acc, NUM_WR, wr_cmp, NUM_WR);
        $display("  Write latency (ticks): min=%0d  max=%0d  avg=%0d",
                 wr_lat_min, wr_lat_max, wr_lat_total / wr_cmp);
        $display("  Reads    : accepted=%0d / %0d   R-data=%0d / %0d",
                 rd_acc, total_rd, rd_cmp, total_rd);
        $display("  Read  latency (ticks): min=%0d  max=%0d  avg=%0d",
                 rd_lat_min, rd_lat_max, rd_lat_total / rd_cmp);
        $display("  Functional: OK=%0d / %0d   FAIL=%0d",
                 func_ok, total_rd, func_fail);

        if (wr_acc   == NUM_WR   && wr_cmp  == NUM_WR   &&
            rd_acc   == total_rd && rd_cmp  == total_rd &&
            func_fail == 0)
        begin
            $display("\n=== PASSED ===");
            ramulator_exit(0);
        end else begin
            $display("\n=== FAILED ===");
            ramulator_exit(1);
        end
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
