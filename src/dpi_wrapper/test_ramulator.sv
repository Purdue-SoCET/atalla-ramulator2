// test_ramulator.sv — AXI master-side smoketest for ramulator_sv_wrapper
//
// Test plan:
//   [1] Reset + ramulator_init
//   [2] Write phase      — NUM_WR writes, verify B ack per write
//   [3] Read-after-write — read back each written address, check vs shadow mem
//   [4] Raw reads        — read unwritten addresses, check wrapper returns addr
//   [5] Read backpressure — flood Ramulator read queue with r_i_ready=0;
//                           verify stall cycles increase; drain all responses
//   [6] Write backpressure — fill B FIFO (B_DEPTH writes, b_i_ready=0);
//                            verify next write stalls; drain all B-acks
//   [7] Simultaneous R+W  — assert AR + AW+W in the same cycle; verify both
//                            accepted; check R data; read-back written addr
//   [8] Multi-beat INCR bursts — 4-beat and 8-beat write+read bursts;
//                            verify per-beat data and last flag
//   [9] Multiple outstanding reads — issue NUM_PIPE ARs with r_i_ready=0,
//                            verify SR FIFO buffers all responses; drain and check
//   [10] WSTRB byte masking — lower-half, upper-half, zero, single-byte strobes
//   [11] Pass/fail summary

`timescale 1ns / 1ps

module test_ramulator #(
    // Overridable from the command line: make sim CFG=configs/other.yaml
    // vsim passes it via -G CFG="..." and vlog via +define+... is NOT used here.
    parameter string CFG = "configs/ddr4_config.yaml"
);
    import axi_bus_pkg::*;

    // Use ramulator_exit() to bypass QuestaSim post-sim cleanup which
    // hits heap corruption left by Ramulator2 (see ramulator_dpi.cpp).
    import "DPI-C" function void ramulator_exit(input int code);

    // ----------------------------------------------------------------
    // Parameters  — B_DEPTH must match ramulator_sv_wrapper's B_DEPTH
    // ----------------------------------------------------------------
    // CFG is now a top-level parameter (see module header above)
    localparam int     NUM_WR       = 16;
    localparam int     NUM_RD_RAW   = 8;
    localparam int     NUM_FLOOD    = 48;   // ARs to fire in read-BP test
    localparam int     B_DEPTH      = 4;    // must match wrapper parameter
    localparam int     BP_HOLD      = 20;   // cycles to observe write stall
    localparam int     MAX_CYCLES   = 2_000_000;
    localparam longint WR_BASE      = 64'h0000_0000;
    localparam longint RD_BASE      = 64'h0010_0000;
    localparam longint FLOOD_BASE   = 64'h0020_0000;
    localparam longint BP_WR_BASE   = 64'h0030_0000;
    localparam longint SIMUL_BASE   = 64'h0040_0000;
    localparam longint BURST4_BASE  = 64'h0050_0000;  // [8] 4-beat burst test
    localparam longint BURST8_BASE  = 64'h0060_0000;  // [8] 8-beat burst test
    localparam longint PIPE_BASE    = 64'h0070_0000;  // [9] SR FIFO pipeline test
    localparam longint STRB_BASE    = 64'h0080_0000;  // [10] WSTRB byte-mask test
    localparam longint STRIDE       = 64;
    localparam int     NUM_PIPE     = 16;              // [9] must be <= SR_DEPTH

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

    ramulator_sv_wrapper #(
        .CONFIG_FILE(CFG),
        .B_DEPTH    (B_DEPTH)
    ) dut (
        .axi      (axi),
        .init_done(init_done)
    );

    // ----------------------------------------------------------------
    // Tick counter
    // ----------------------------------------------------------------
    longint tick = 0;
    always @(posedge clk) tick++;

    // ----------------------------------------------------------------
    // Scoreboard — [2]-[4]
    // ----------------------------------------------------------------
    longint shadow   [longint];
    int     wr_acc   = 0, wr_cmp   = 0;
    int     rd_acc   = 0, rd_cmp   = 0;
    int     func_ok  = 0, func_fail = 0;
    longint wr_lat_min = 2_000_000_000, wr_lat_max = 0, wr_lat_total = 0;
    longint rd_lat_min = 2_000_000_000, rd_lat_max = 0, rd_lat_total = 0;

    // Scoreboard — [5] read flood
    int     flood_acc = 0, flood_cmp = 0;
    longint flood_stall_max = 0;

    // Scoreboard — [6] write backpressure
    // bp_wr_acc : writes accepted (aw_o_ready seen)
    // bp_wr_cmp : B-acks collected
    // bp_stall  : cycles write #B_DEPTH stalled before FIFO had room
    int     bp_wr_acc = 0, bp_wr_cmp = 0;
    longint bp_stall  = 0;

    // Scoreboard — [7] simultaneous
    int     sim_rd_acc = 0, sim_rd_cmp = 0;
    int     sim_wr_acc = 0, sim_wr_cmp = 0;
    int     sim_func_ok = 0, sim_func_fail = 0;

    // Scoreboard — [8] multi-beat INCR bursts
    int     burst_wr_ok = 0, burst_wr_fail = 0;
    int     burst_rd_ok = 0, burst_rd_fail = 0;

    // Scoreboard — [9] SR FIFO pipeline test
    int     pipe_acc = 0, pipe_cmp = 0, pipe_ok = 0, pipe_fail = 0;

    // Scoreboard — [10] WSTRB byte-masking test
    int     strb_ok = 0, strb_fail = 0;

    // ----------------------------------------------------------------
    // Task: full AXI write (issue AW+W, wait for aw_o_ready, collect B)
    // ----------------------------------------------------------------
    task automatic axi_write(
        input longint              addr,
        input logic [WDATA-1:0]    data,
        input logic [MID_AWID-1:0] mid_id
    );
        longint t0, lat;
        t0 = tick;

        axi.aw_o_valid  = 1'b1;
        axi.aw_o.addr   = AWADDR'(addr);
        axi.aw_o.mid_id = mid_id;
        axi.aw_o.size   = 3'b011;
        axi.aw_o.len    = 4'h0;
        axi.aw_o.burst  = 2'b01;

        axi.w_o_valid   = 1'b1;
        axi.w_o.data    = data;
        axi.w_o.mid_id  = MID_ARID'(mid_id);
        axi.w_o.last    = 1'b1;
        axi.w_o.strb    = '1;

        @(posedge clk); #1;
        while (!axi.aw_o_ready) begin @(posedge clk); #1; end
        axi.aw_o_valid = 1'b0;
        axi.w_o_valid  = 1'b0;
        wr_acc++;
        shadow[addr] = longint'(data);

        axi.b_i_ready = 1'b1;
        while (!axi.b_i_valid) begin @(posedge clk); #1; end
        @(posedge clk); #1;
        axi.b_i_ready = 1'b0;
        wr_cmp++;

        lat = tick - t0;
        $display("    wr[%0d] addr=0x%08h  ticks=%0d", wr_cmp - 1, addr, lat);
        if (lat < wr_lat_min) wr_lat_min = lat;
        if (lat > wr_lat_max) wr_lat_max = lat;
        wr_lat_total += lat;
    endtask

    // ----------------------------------------------------------------
    // Task: full AXI read (issue AR, collect R response)
    // ----------------------------------------------------------------
    task automatic axi_read(
        input  longint              addr,
        input  logic [MID_ARID-1:0] mid_id,
        output longint              got_data
    );
        longint t0, lat;
        t0 = tick;

        axi.ar_o_valid  = 1'b1;
        axi.ar_o.addr   = ARADDR'(addr);
        axi.ar_o.mid_id = mid_id;
        axi.ar_o.size   = 3'b011;
        axi.ar_o.len    = 4'h0;
        axi.ar_o.burst  = 2'b01;

        @(posedge clk); #1;
        while (!axi.ar_o_ready) begin @(posedge clk); #1; end
        axi.ar_o_valid = 1'b0;
        rd_acc++;

        axi.r_i_ready = 1'b1;
        while (!axi.r_i_valid) begin @(posedge clk); #1; end
        got_data = longint'(axi.r_i.data);
        @(posedge clk); #1;
        axi.r_i_ready = 1'b0;
        rd_cmp++;

        lat = tick - t0;
        $display("    rd[%0d] addr=0x%08h  ticks=%0d", rd_cmp - 1, addr, lat);
        if (lat < rd_lat_min) rd_lat_min = lat;
        if (lat > rd_lat_max) rd_lat_max = lat;
        rd_lat_total += lat;
    endtask

    // ----------------------------------------------------------------
    // Task: issue AR only, no R collection. Returns stall cycles.
    // r_i_ready is NOT touched — caller controls it.
    // ----------------------------------------------------------------
    task automatic axi_ar_send(
        input  longint              addr,
        input  logic [MID_ARID-1:0] mid_id,
        output longint              stall_cycles
    );
        longint t0;
        t0 = tick;

        axi.ar_o_valid  = 1'b1;
        axi.ar_o.addr   = ARADDR'(addr);
        axi.ar_o.mid_id = mid_id;
        axi.ar_o.size   = 3'b011;
        axi.ar_o.len    = 4'h0;
        axi.ar_o.burst  = 2'b01;

        @(posedge clk); #1;
        while (!axi.ar_o_ready) begin @(posedge clk); #1; end
        axi.ar_o_valid = 1'b0;
        stall_cycles = tick - t0;
        flood_acc++;
    endtask

    // ----------------------------------------------------------------
    // Task: collect one R response (asserts/deasserts r_i_ready)
    // ----------------------------------------------------------------
    task automatic axi_r_collect(output longint got_data);
        axi.r_i_ready = 1'b1;
        while (!axi.r_i_valid) begin @(posedge clk); #1; end
        got_data = longint'(axi.r_i.data);
        @(posedge clk); #1;
        axi.r_i_ready = 1'b0;
        flood_cmp++;
    endtask

    // ----------------------------------------------------------------
    // Task: issue AW+W without collecting B (non-blocking write).
    // Waits for aw_o_ready, then returns. Caller must drain B-acks.
    // Increments bp_wr_acc.
    // ----------------------------------------------------------------
    task automatic axi_aw_nb(
        input  longint              addr,
        input  logic [WDATA-1:0]    data,
        input  logic [MID_AWID-1:0] mid_id,
        output longint              stall_cycles
    );
        longint t0;
        t0 = tick;

        axi.aw_o_valid  = 1'b1;
        axi.aw_o.addr   = AWADDR'(addr);
        axi.aw_o.mid_id = mid_id;
        axi.aw_o.size   = 3'b011;
        axi.aw_o.len    = 4'h0;
        axi.aw_o.burst  = 2'b01;

        axi.w_o_valid   = 1'b1;
        axi.w_o.data    = data;
        axi.w_o.mid_id  = MID_ARID'(mid_id);
        axi.w_o.last    = 1'b1;
        axi.w_o.strb    = '1;

        @(posedge clk); #1;
        while (!axi.aw_o_ready) begin @(posedge clk); #1; end
        axi.aw_o_valid = 1'b0;
        axi.w_o_valid  = 1'b0;
        shadow[addr] = longint'(data);
        bp_wr_acc++;
        stall_cycles = tick - t0;
    endtask

    // ----------------------------------------------------------------
    // Task: INCR write burst.
    //   Sends AW + W[0] together, then W[1]..W[len] in successive cycles.
    //   Waits for the single B ack that follows the last W beat.
    //   Data for beat i: base_data + i.  Shadow memory updated per beat.
    // ----------------------------------------------------------------
    task automatic axi_write_burst(
        input  longint              base_addr,
        input  longint              base_data,
        input  int                  len,        // beats - 1  (0 = single beat)
        input  logic [2:0]          size,       // log2(bytes per beat)
        input  logic [MID_AWID-1:0] mid_id
    );
        int stride;
        stride = 1 << int'(size);

        // --- Issue AW + W[0] ---
        axi.aw_o_valid  = 1'b1;
        axi.aw_o.addr   = AWADDR'(base_addr);
        axi.aw_o.mid_id = mid_id;
        axi.aw_o.size   = size;
        axi.aw_o.len    = AWLEN'(len);
        axi.aw_o.burst  = 2'b01;       // INCR

        axi.w_o_valid   = 1'b1;
        axi.w_o.data    = WDATA'(base_data);
        axi.w_o.mid_id  = MID_ARID'(mid_id);
        axi.w_o.last    = (len == 0) ? 1'b1 : 1'b0;
        axi.w_o.strb    = '1;

        @(posedge clk); #1;
        while (!axi.aw_o_ready) begin @(posedge clk); #1; end
        // AW and W[0] accepted simultaneously
        axi.aw_o_valid = 1'b0;
        shadow[base_addr] = base_data;

        // --- Issue W[1]..W[len] ---
        for (int i = 1; i <= len; i++) begin
            longint baddr, bdata;
            baddr = base_addr + longint'(i) * longint'(stride);
            bdata = base_data + longint'(i);
            // Update W channel before the next posedge so wrapper sees it
            axi.w_o.data = WDATA'(bdata);
            axi.w_o.last = (i == len) ? 1'b1 : 1'b0;
            axi.w_o_valid = 1'b1;
            @(posedge clk); #1;
            while (!axi.w_o_ready) begin @(posedge clk); #1; end
            shadow[baddr] = bdata;
        end
        axi.w_o_valid = 1'b0;

        // --- Wait for B ack ---
        axi.b_i_ready = 1'b1;
        while (!axi.b_i_valid) begin @(posedge clk); #1; end
        @(posedge clk); #1;
        axi.b_i_ready = 1'b0;
    endtask

    // ----------------------------------------------------------------
    // Task: INCR read burst.
    //   Issues AR with len/size, collects len+1 R beats in order.
    //   Verifies each beat's data against shadow (or address if unwritten).
    //   Also checks the last flag.  Updates burst_rd_ok / burst_rd_fail.
    // ----------------------------------------------------------------
    task automatic axi_read_burst(
        input  longint              base_addr,
        input  int                  len,
        input  logic [2:0]          size,
        input  logic [MID_ARID-1:0] mid_id
    );
        int stride;
        stride = 1 << int'(size);

        // --- Issue AR ---
        axi.ar_o_valid  = 1'b1;
        axi.ar_o.addr   = ARADDR'(base_addr);
        axi.ar_o.mid_id = mid_id;
        axi.ar_o.size   = size;
        axi.ar_o.len    = ARLEN'(len);
        axi.ar_o.burst  = 2'b01;       // INCR

        @(posedge clk); #1;
        while (!axi.ar_o_ready) begin @(posedge clk); #1; end
        axi.ar_o_valid = 1'b0;

        // --- Collect R beats ---
        for (int i = 0; i <= len; i++) begin
            longint baddr, got, exp;
            logic   got_last;
            baddr = base_addr + longint'(i) * longint'(stride);

            axi.r_i_ready = 1'b1;
            while (!axi.r_i_valid) begin @(posedge clk); #1; end
            got      = longint'(axi.r_i.data);
            got_last = axi.r_i.last;
            @(posedge clk); #1;
            axi.r_i_ready = 1'b0;

            // Verify data
            exp = shadow.exists(baddr) ? shadow[baddr] : baddr;
            if (got === exp) burst_rd_ok++;
            else begin
                $display("  [BURST RD MISMATCH] beat=%0d addr=0x%08h got=0x%016h exp=0x%016h",
                         i, baddr, got, exp);
                burst_rd_fail++;
            end

            // Verify last flag
            if ((i == len) && !got_last) begin
                $display("  [BURST RD ERROR] beat=%0d: expected last=1, got last=0", i);
                burst_rd_fail++;
            end else if ((i < len) && got_last) begin
                $display("  [BURST RD ERROR] beat=%0d: expected last=0, got last=1", i);
                burst_rd_fail++;
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Task: single-beat AXI write with an explicit WSTRB value.
    //   Issues AW+W simultaneously, waits for aw_o_ready, collects B ack.
    //   Does NOT update shadow[] — caller manages expected values.
    // ----------------------------------------------------------------
    task automatic axi_write_strb(
        input longint              addr,
        input logic [WDATA-1:0]    data,
        input logic [WSTRB-1:0]    strb,
        input logic [MID_AWID-1:0] mid_id
    );
        axi.aw_o_valid  = 1'b1;
        axi.aw_o.addr   = AWADDR'(addr);
        axi.aw_o.mid_id = mid_id;
        axi.aw_o.size   = 3'b011;
        axi.aw_o.len    = 4'h0;
        axi.aw_o.burst  = 2'b01;

        axi.w_o_valid   = 1'b1;
        axi.w_o.data    = data;
        axi.w_o.mid_id  = MID_ARID'(mid_id);
        axi.w_o.last    = 1'b1;
        axi.w_o.strb    = strb;

        @(posedge clk); #1;
        while (!axi.aw_o_ready) begin @(posedge clk); #1; end
        axi.aw_o_valid = 1'b0;
        axi.w_o_valid  = 1'b0;

        axi.b_i_ready = 1'b1;
        while (!axi.b_i_valid) begin @(posedge clk); #1; end
        @(posedge clk); #1;
        axi.b_i_ready = 1'b0;
    endtask

    // ----------------------------------------------------------------
    // Task: single-beat AXI read without touching global rd counters.
    //   Used by [10] WSTRB tests so they don't skew [3-4] pass checks.
    // ----------------------------------------------------------------
    task automatic axi_read_raw(
        input  longint              addr,
        input  logic [MID_ARID-1:0] mid_id,
        output longint              got_data
    );
        axi.ar_o_valid  = 1'b1;
        axi.ar_o.addr   = ARADDR'(addr);
        axi.ar_o.mid_id = mid_id;
        axi.ar_o.size   = 3'b011;
        axi.ar_o.len    = 4'h0;
        axi.ar_o.burst  = 2'b01;

        @(posedge clk); #1;
        while (!axi.ar_o_ready) begin @(posedge clk); #1; end
        axi.ar_o_valid = 1'b0;

        axi.r_i_ready = 1'b1;
        while (!axi.r_i_valid) begin @(posedge clk); #1; end
        got_data = longint'(axi.r_i.data);
        @(posedge clk); #1;
        axi.r_i_ready = 1'b0;
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

        // [3] Read-after-write
        $display("[3] %0d reads (read-after-write, verify vs shadow) ...", NUM_WR);
        for (int i = 0; i < NUM_WR; i++) begin
            axi_read(
                WR_BASE + longint'(i) * STRIDE,
                MID_ARID'(i % (1 << MID_ARID)),
                got
            );
            exp = shadow[WR_BASE + longint'(i) * STRIDE];
            if (got === exp) func_ok++;
            else begin
                $display("  [MISMATCH] i=%0d addr=0x%08h got=0x%016h exp=0x%016h",
                         i, WR_BASE + longint'(i) * STRIDE, got, exp);
                func_fail++;
            end
        end
        $display("    accepted=%0d  R-data=%0d  OK=%0d  FAIL=%0d",
                 rd_acc, rd_cmp, func_ok, func_fail);

        // [4] Raw reads (unwritten region — wrapper returns addr as data)
        $display("[4] %0d raw reads (unwritten region, expect addr as data) ...", NUM_RD_RAW);
        for (int i = 0; i < NUM_RD_RAW; i++) begin
            axi_read(
                RD_BASE + longint'(i) * STRIDE,
                MID_ARID'(i % (1 << MID_ARID)),
                got
            );
            exp = RD_BASE + longint'(i) * STRIDE;
            if (got === exp) func_ok++;
            else begin
                $display("  [MISMATCH] i=%0d addr=0x%08h got=0x%016h exp=0x%016h",
                         i, RD_BASE + longint'(i) * STRIDE, got, exp);
                func_fail++;
            end
        end
        total_rd = NUM_WR + NUM_RD_RAW;
        $display("    accepted=%0d  R-data=%0d  OK=%0d  FAIL=%0d",
                 rd_acc, rd_cmp, func_ok, func_fail);

        // ============================================================
        // [5] Read backpressure: flood Ramulator read queue
        //
        // Issue NUM_FLOOD AR requests without consuming any R responses
        // (r_i_ready held 0). Each AR is sent to Ramulator's internal
        // request buffer (capacity ~32).  Once the buffer fills,
        // ar_o_ready stalls for multiple cycles per request.
        // After all ARs are sent, drain all responses.
        // ============================================================
        $display("[5] Read backpressure: %0d ARs issued with r_i_ready=0 ...", NUM_FLOOD);
        axi.r_i_ready = 1'b0;

        for (int i = 0; i < NUM_FLOOD; i++) begin
            longint stall;
            axi_ar_send(
                FLOOD_BASE + longint'(i) * STRIDE,
                MID_ARID'(i % (1 << MID_ARID)),
                stall
            );
            if (stall > flood_stall_max) flood_stall_max = stall;
            if (stall > 1)
                $display("    flood AR[%0d] addr=0x%08h  stall=%0d cycles (backpressure)",
                         i, FLOOD_BASE + longint'(i) * STRIDE, stall);
        end
        $display("    All %0d ARs accepted. Max stall=%0d cycles. Draining ...",
                 flood_acc, flood_stall_max);

        for (int i = 0; i < NUM_FLOOD; i++) begin
            axi_r_collect(got);
        end
        $display("    Drained %0d R responses.", flood_cmp);

        // ============================================================
        // [6] Write backpressure: fill B FIFO, verify stall, drain
        //
        // With b_i_ready=0:
        //   Writes #0..#B_DEPTH-1 are accepted immediately — each one
        //   pushes a B entry into the FIFO (b_cnt climbs 0→B_DEPTH).
        //
        //   Write #B_DEPTH is driven next.  The wrapper condition
        //     b_cnt_next < B_DEPTH  evaluates false (FIFO full) so
        //     aw_o_ready never pulses → AW channel stalls.
        //
        // After BP_HOLD stall cycles, b_i_ready=1 is asserted:
        //   The FIFO head is drained in the same clock edge that write
        //   #B_DEPTH is accepted (simultaneous drain+push: b_cnt stays
        //   B_DEPTH). aw_o_ready pulses this cycle.
        //
        // Finally all B_DEPTH+1 B-acks are collected.
        // ============================================================
        $display("[6] Write backpressure: fill %0d-deep B FIFO then stall ...", B_DEPTH);
        axi.b_i_ready = 1'b0;

        // -- Phase A: fill B FIFO with B_DEPTH writes --
        for (int i = 0; i < B_DEPTH; i++) begin
            longint stall;
            axi_aw_nb(
                BP_WR_BASE + longint'(i) * STRIDE,
                WDATA'(64'hAAAA_0000_0000_0000 | longint'(i)),
                MID_AWID'(i % (1 << MID_AWID)),
                stall
            );
            $display("    write[%0d] accepted (stall=%0d, FIFO count=%0d/4)", i, stall, i+1);
        end
        $display("    B FIFO full. Driving write[%0d] with b_i_ready=0 ...", B_DEPTH);

        // -- Phase B: drive write #B_DEPTH, observe stall --
        axi.aw_o_valid  = 1'b1;
        axi.aw_o.addr   = AWADDR'(BP_WR_BASE + longint'(B_DEPTH) * STRIDE);
        axi.aw_o.mid_id = MID_AWID'(B_DEPTH % (1 << MID_AWID));
        axi.aw_o.size   = 3'b011;
        axi.aw_o.len    = 4'h0;
        axi.aw_o.burst  = 2'b01;
        axi.w_o_valid   = 1'b1;
        axi.w_o.data    = WDATA'(64'hBBBB_0000_0000_0000 | longint'(B_DEPTH));
        axi.w_o.mid_id  = MID_ARID'(B_DEPTH % (1 << MID_ARID));
        axi.w_o.last    = 1'b1;
        axi.w_o.strb    = '1;

        begin
            int early_ready;
            early_ready = 0;    // separate assignment avoids implicit-static warning
            repeat(BP_HOLD) begin
                @(posedge clk); #1;
                if (axi.aw_o_ready) early_ready++;
                bp_stall++;
            end
            if (early_ready == 0)
                $display("    write[%0d]: correctly stalled for %0d cycles (FIFO full).",
                         B_DEPTH, bp_stall);
            else
                $display("    write[%0d]: [WARN] aw_o_ready pulsed early %0d times.",
                         B_DEPTH, early_ready);
        end

        // -- Phase C: release b_i_ready → drain head + accept write #B_DEPTH --
        // Wrapper step 3 drains one B entry, step 4 pushes write #B_DEPTH in
        // the same clock edge (b_cnt stays B_DEPTH). aw_o_ready pulses.
        axi.b_i_ready = 1'b1;
        @(posedge clk); #1;
        while (!axi.aw_o_ready) begin @(posedge clk); #1; end
        axi.aw_o_valid = 1'b0;
        axi.w_o_valid  = 1'b0;
        shadow[BP_WR_BASE + longint'(B_DEPTH) * STRIDE] =
            longint'(64'hBBBB_0000_0000_0000 | longint'(B_DEPTH));
        bp_wr_acc++;
        $display("    write[%0d] accepted after stall.", B_DEPTH);

        // -- Phase D: drain all B_DEPTH+1 B-acks --
        // b_i_ready is still 1. The FIFO holds B_DEPTH entries (one was
        // drained simultaneously with write #B_DEPTH's acceptance, one new
        // entry was pushed). Count that implicit drain separately, then drain
        // the remaining B_DEPTH entries explicitly.
        bp_wr_cmp++;  // B[0] was consumed in the simultaneous drain+push
        for (int i = 0; i < B_DEPTH; i++) begin
            while (!axi.b_i_valid) begin @(posedge clk); #1; end
            @(posedge clk); #1;
            bp_wr_cmp++;
        end
        axi.b_i_ready = 1'b0;
        $display("    All %0d B-acks collected. bp_wr_acc=%0d  bp_wr_cmp=%0d",
                 B_DEPTH + 1, bp_wr_acc, bp_wr_cmp);

        // ============================================================
        // [7] Simultaneous read + write
        //
        // Assert AR + AW+W in the same cycle. Wrapper processes AR in
        // step 2 and AW+W in step 4 of the same always block, so both
        // ar_o_ready and aw_o_ready can pulse simultaneously.
        //
        //   Read  : SIMUL_BASE+0      (never written → expect addr as data)
        //   Write : SIMUL_BASE+STRIDE (then read-back to verify write)
        // ============================================================
        $display("[7] Simultaneous read + write ...");
        begin
            bit     ar_done, aw_done;
            longint simul_rd_addr, simul_wr_addr, simul_wr_data;
            longint t_issue;

            simul_rd_addr = SIMUL_BASE;
            simul_wr_addr = SIMUL_BASE + STRIDE;
            simul_wr_data = 64'hFEED_FACE_CAFE_BABE;
            ar_done = 0;
            aw_done = 0;

            // Assert AR and AW+W simultaneously
            axi.ar_o_valid  = 1'b1;
            axi.ar_o.addr   = ARADDR'(simul_rd_addr);
            axi.ar_o.mid_id = MID_ARID'(0);
            axi.ar_o.size   = 3'b011;
            axi.ar_o.len    = 4'h0;
            axi.ar_o.burst  = 2'b01;

            axi.aw_o_valid  = 1'b1;
            axi.aw_o.addr   = AWADDR'(simul_wr_addr);
            axi.aw_o.mid_id = MID_AWID'(0);
            axi.aw_o.size   = 3'b011;
            axi.aw_o.len    = 4'h0;
            axi.aw_o.burst  = 2'b01;

            axi.w_o_valid   = 1'b1;
            axi.w_o.data    = WDATA'(simul_wr_data);
            axi.w_o.mid_id  = MID_ARID'(0);
            axi.w_o.last    = 1'b1;
            axi.w_o.strb    = '1;

            axi.b_i_ready   = 1'b0;

            t_issue = tick;
            @(posedge clk); #1;

            // Collect handshakes — both may fire in the same cycle
            while (!ar_done || !aw_done) begin
                if (!ar_done && axi.ar_o_ready) begin
                    axi.ar_o_valid = 1'b0;
                    sim_rd_acc++;
                    ar_done = 1;
                end
                if (!aw_done && axi.aw_o_ready) begin
                    axi.aw_o_valid = 1'b0;
                    axi.w_o_valid  = 1'b0;
                    shadow[simul_wr_addr] = simul_wr_data;
                    sim_wr_acc++;
                    aw_done = 1;
                end
                if (!ar_done || !aw_done) begin @(posedge clk); #1; end
            end

            if (tick - t_issue <= 2)
                $display("    AR and AW accepted in same cycle (tick=%0d).", tick);
            else
                $display("    AR and AW accepted in different cycles (%0d ticks).",
                         tick - t_issue);

            // Collect B for the write
            axi.b_i_ready = 1'b1;
            while (!axi.b_i_valid) begin @(posedge clk); #1; end
            @(posedge clk); #1;
            axi.b_i_ready = 1'b0;
            sim_wr_cmp++;

            // Collect R for the simultaneous read (unwritten → expect addr)
            axi.r_i_ready = 1'b1;
            while (!axi.r_i_valid) begin @(posedge clk); #1; end
            got = longint'(axi.r_i.data);
            @(posedge clk); #1;
            axi.r_i_ready = 1'b0;
            sim_rd_cmp++;

            exp = simul_rd_addr;
            if (got === exp) begin
                sim_func_ok++;
                $display("    Simul-read  OK : got=0x%016h", got);
            end else begin
                sim_func_fail++;
                $display("    Simul-read  MISMATCH: got=0x%016h exp=0x%016h", got, exp);
            end

            // Read back the written address to verify the write landed
            axi.ar_o_valid  = 1'b1;
            axi.ar_o.addr   = ARADDR'(simul_wr_addr);
            axi.ar_o.mid_id = MID_ARID'(1);
            axi.ar_o.size   = 3'b011;
            axi.ar_o.len    = 4'h0;
            axi.ar_o.burst  = 2'b01;
            @(posedge clk); #1;
            while (!axi.ar_o_ready) begin @(posedge clk); #1; end
            axi.ar_o_valid = 1'b0;
            sim_rd_acc++;

            axi.r_i_ready = 1'b1;
            while (!axi.r_i_valid) begin @(posedge clk); #1; end
            got = longint'(axi.r_i.data);
            @(posedge clk); #1;
            axi.r_i_ready = 1'b0;
            sim_rd_cmp++;

            exp = simul_wr_data;
            if (got === exp) begin
                sim_func_ok++;
                $display("    Read-back   OK : got=0x%016h", got);
            end else begin
                sim_func_fail++;
                $display("    Read-back   MISMATCH: got=0x%016h exp=0x%016h", got, exp);
            end
        end

        // ============================================================
        // [8] Multi-beat INCR bursts
        //
        // 8a. 4-beat write burst (len=3, size=3, 8 B/beat) → verify B ack
        // 8b. 4-beat read burst  → verify R data matches shadow + last flag
        // 8c. 8-beat write burst (len=7)
        // 8d. 8-beat read burst  → verify data + last flag
        // ============================================================
        $display("[8] Multi-beat INCR bursts ...");

        // 8a/8b: 4-beat burst
        $display("  [8a] 4-beat write burst at 0x%08h (len=3, size=3)", BURST4_BASE);
        axi_write_burst(BURST4_BASE, 64'hCAFE_0000_0050_0000, 3, 3'b011, MID_AWID'(0));
        $display("  [8b] 4-beat read burst  at 0x%08h", BURST4_BASE);
        axi_read_burst(BURST4_BASE, 3, 3'b011, MID_ARID'(0));
        $display("    4-beat: rd_ok=%0d  rd_fail=%0d", burst_rd_ok, burst_rd_fail);

        // 8c/8d: 8-beat burst
        $display("  [8c] 8-beat write burst at 0x%08h (len=7, size=3)", BURST8_BASE);
        axi_write_burst(BURST8_BASE, 64'hDEAD_0000_0060_0000, 7, 3'b011, MID_AWID'(1));
        $display("  [8d] 8-beat read burst  at 0x%08h", BURST8_BASE);
        axi_read_burst(BURST8_BASE, 7, 3'b011, MID_ARID'(1));
        $display("    8-beat: rd_ok=%0d  rd_fail=%0d",
                 burst_rd_ok - (burst_rd_ok >= 4 ? 4 : burst_rd_ok),
                 burst_rd_fail);

        $display("  [8] Total burst rd_ok=%0d/12  rd_fail=%0d  wr_ok=%0d  wr_fail=%0d",
                 burst_rd_ok, burst_rd_fail, burst_wr_ok, burst_wr_fail);

        // ============================================================
        // [9] Multiple outstanding reads (SR FIFO pipeline test)
        //
        // Issue NUM_PIPE single-beat ARs back-to-back while keeping
        // r_i_ready=0.  Ramulator queues all requests simultaneously;
        // completions accumulate in the SR FIFO (SR_DEPTH=16 entries).
        // In Phase B, r_i_ready=1 drains all responses; each returned
        // data value must equal the corresponding request address
        // (unwritten region — functional model default).
        // ============================================================
        $display("[9] Multiple outstanding reads: %0d ARs pipelined (r_i_ready=0) ...", NUM_PIPE);
        begin
            // --- Phase A: issue all ARs back-to-back, r_i_ready=0 ---
            axi.r_i_ready = 1'b0;
            for (int i = 0; i < NUM_PIPE; i++) begin
                longint paddr;
                paddr = PIPE_BASE + longint'(i) * STRIDE;

                axi.ar_o_valid  = 1'b1;
                axi.ar_o.addr   = ARADDR'(paddr);
                axi.ar_o.mid_id = MID_ARID'(i % (1 << MID_ARID));
                axi.ar_o.size   = 3'b011;
                axi.ar_o.len    = 4'h0;
                axi.ar_o.burst  = 2'b01;

                @(posedge clk); #1;
                while (!axi.ar_o_ready) begin @(posedge clk); #1; end
                axi.ar_o_valid = 1'b0;
                pipe_acc++;
            end
            $display("    All %0d ARs accepted. Draining SR FIFO ...", pipe_acc);

            // --- Phase B: drain all responses, verify data = addr ---
            axi.r_i_ready = 1'b1;
            for (int i = 0; i < NUM_PIPE; i++) begin
                longint got_d;
                while (!axi.r_i_valid) begin @(posedge clk); #1; end
                got_d = longint'(axi.r_i.data);
                @(posedge clk); #1;
                pipe_cmp++;
                // Unwritten region: wrapper returns addr as data.
                // Responses may arrive out-of-order; verify got_d is
                // one of the valid PIPE_BASE addresses.
                if (got_d >= PIPE_BASE &&
                    got_d < PIPE_BASE + longint'(NUM_PIPE) * STRIDE &&
                    ((got_d - PIPE_BASE) % STRIDE) == 0) begin
                    pipe_ok++;
                    $display("    pipe R[%0d] data=0x%016h OK", i, got_d);
                end else begin
                    pipe_fail++;
                    $display("    pipe R[%0d] data=0x%016h MISMATCH (not in expected set)", i, got_d);
                end
            end
            axi.r_i_ready = 1'b0;
            $display("    Pipeline: acc=%0d  cmp=%0d  OK=%0d  FAIL=%0d",
                     pipe_acc, pipe_cmp, pipe_ok, pipe_fail);
        end

        // ============================================================
        // [10] WSTRB byte masking
        //
        // All writes use axi_write_strb (explicit strobe) and reads use
        // axi_read_raw (does not touch global rd counters).
        // The wrapper maintains its own wr_shadow and merges bytes before
        // passing to Ramulator — no masking logic reaches Ramulator.
        //
        // [10a] Lower-half strobe (0x0F): upper 4 B preserved from first write
        // [10b] Upper-half strobe (0xF0): lower 4 B preserved from first write
        // [10c] Zero strobe (0x00): entire word unchanged (no-op write)
        // [10d] Single-byte strobe (0x01) to a fresh address: only byte 0 set
        // ============================================================
        $display("[10] WSTRB byte masking ...");
        begin
            longint got, exp;

            // [10a] Lower-half mask: establish full word, then update bytes 0-3 only
            axi_write_strb(STRB_BASE,          64'hDEADBEEF_12345678, 8'hFF, MID_AWID'(0));
            axi_write_strb(STRB_BASE,          64'hAAAAAAAA_BBBBBBBB, 8'h0F, MID_AWID'(0));
            exp = 64'hDEADBEEF_BBBBBBBB;  // bytes 4-7 from 1st write, bytes 0-3 from 2nd
            axi_read_raw(STRB_BASE, MID_ARID'(0), got);
            if (got === exp) begin
                strb_ok++;
                $display("  [10a] lower-half mask OK  got=0x%016h", got);
            end else begin
                strb_fail++;
                $display("  [10a] lower-half mask FAIL got=0x%016h exp=0x%016h", got, exp);
            end

            // [10b] Upper-half mask: establish full word, then update bytes 4-7 only
            axi_write_strb(STRB_BASE+STRIDE,   64'h11111111_22222222, 8'hFF, MID_AWID'(0));
            axi_write_strb(STRB_BASE+STRIDE,   64'h99999999_00000000, 8'hF0, MID_AWID'(0));
            exp = 64'h99999999_22222222;  // bytes 4-7 from 2nd write, bytes 0-3 from 1st
            axi_read_raw(STRB_BASE+STRIDE, MID_ARID'(0), got);
            if (got === exp) begin
                strb_ok++;
                $display("  [10b] upper-half mask OK  got=0x%016h", got);
            end else begin
                strb_fail++;
                $display("  [10b] upper-half mask FAIL got=0x%016h exp=0x%016h", got, exp);
            end

            // [10c] Zero strobe: write should be a complete no-op for data
            axi_write_strb(STRB_BASE+2*STRIDE, 64'h5A5A5A5A_5A5A5A5A, 8'hFF, MID_AWID'(0));
            axi_write_strb(STRB_BASE+2*STRIDE, 64'hFFFFFFFF_FFFFFFFF, 8'h00, MID_AWID'(0));
            exp = 64'h5A5A5A5A_5A5A5A5A;  // unchanged
            axi_read_raw(STRB_BASE+2*STRIDE, MID_ARID'(0), got);
            if (got === exp) begin
                strb_ok++;
                $display("  [10c] zero strobe OK      got=0x%016h", got);
            end else begin
                strb_fail++;
                $display("  [10c] zero strobe FAIL    got=0x%016h exp=0x%016h", got, exp);
            end

            // [10d] Single-byte strobe to fresh address (shadow defaults to 0)
            //   data=0xFEDCBA98_76543210, strb=0x01 → only byte 0 = 0x10
            axi_write_strb(STRB_BASE+3*STRIDE, 64'hFEDCBA98_76543210, 8'h01, MID_AWID'(0));
            exp = 64'h0000000000000010;
            axi_read_raw(STRB_BASE+3*STRIDE, MID_ARID'(0), got);
            if (got === exp) begin
                strb_ok++;
                $display("  [10d] single-byte OK      got=0x%016h", got);
            end else begin
                strb_fail++;
                $display("  [10d] single-byte FAIL    got=0x%016h exp=0x%016h", got, exp);
            end

            $display("  WSTRB: OK=%0d/4  FAIL=%0d", strb_ok, strb_fail);
        end

        // ============================================================
        // [11] Summary
        // ============================================================
        $display("\n--- Summary ---");
        $display("  Total ticks   : %0d", tick);
        $display("  [2] Writes    : acc=%0d/%0d  B-acks=%0d/%0d  lat(min/max/avg)=%0d/%0d/%0d",
                 wr_acc, NUM_WR, wr_cmp, NUM_WR,
                 wr_lat_min, wr_lat_max, wr_lat_total / (wr_cmp > 0 ? wr_cmp : 1));
        $display("  [3-4] Reads   : acc=%0d/%0d  R-data=%0d/%0d  lat(min/max/avg)=%0d/%0d/%0d",
                 rd_acc, total_rd, rd_cmp, total_rd,
                 rd_lat_min, rd_lat_max, rd_lat_total / (rd_cmp > 0 ? rd_cmp : 1));
        $display("  [3-4] Functional: OK=%0d/%0d  FAIL=%0d",
                 func_ok, total_rd, func_fail);
        $display("  [5] RD flood  : ARs acc=%0d/%0d  R drained=%0d/%0d  max_stall=%0d cycles",
                 flood_acc, NUM_FLOOD, flood_cmp, NUM_FLOOD, flood_stall_max);
        $display("  [6] WR BP     : writes acc=%0d/%0d  B-acks=%0d/%0d  stall=%0d cycles",
                 bp_wr_acc, B_DEPTH + 1, bp_wr_cmp, B_DEPTH + 1, bp_stall);
        $display("  [7] Simul     : rd acc=%0d/2  rd cmp=%0d/2  wr acc=%0d/1  wr cmp=%0d/1  func OK=%0d  FAIL=%0d",
                 sim_rd_acc, sim_rd_cmp, sim_wr_acc, sim_wr_cmp, sim_func_ok, sim_func_fail);
        $display("  [8] Bursts    : rd_ok=%0d/12  rd_fail=%0d  wr_fail=%0d",
                 burst_rd_ok, burst_rd_fail, burst_wr_fail);
        $display("  [9] SR FIFO   : acc=%0d/%0d  cmp=%0d/%0d  OK=%0d  FAIL=%0d",
                 pipe_acc, NUM_PIPE, pipe_cmp, NUM_PIPE, pipe_ok, pipe_fail);
        $display("  [10] WSTRB    : OK=%0d/4  FAIL=%0d",
                 strb_ok, strb_fail);

        if (wr_acc    == NUM_WR    && wr_cmp  == NUM_WR    &&
            rd_acc    == total_rd  && rd_cmp  == total_rd  &&
            func_fail == 0         &&
            flood_acc == NUM_FLOOD && flood_cmp == NUM_FLOOD &&
            bp_wr_acc == B_DEPTH + 1 && bp_wr_cmp == B_DEPTH + 1 &&
            bp_stall  >= BP_HOLD   &&
            sim_rd_acc == 2        && sim_rd_cmp == 2       &&
            sim_wr_acc == 1        && sim_wr_cmp == 1       &&
            sim_func_fail == 0     &&
            burst_rd_ok == 12      && burst_rd_fail == 0    &&
            burst_wr_fail == 0     &&
            pipe_acc == NUM_PIPE   && pipe_cmp == NUM_PIPE  &&
            pipe_ok == NUM_PIPE    && pipe_fail == 0        &&
            strb_ok == 4           && strb_fail == 0)
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
