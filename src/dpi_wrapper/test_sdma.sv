// test_sdma.sv — SDMA tile load testbench for ramulator_sv_wrapper
//
// Models the AXI reads an SDMA instruction generates when loading tiles from a
// 1024x1024 int16 matrix. Each SDMA loads one full 32×32 tile:
//   32 rows × (32 cols × 2 B) = 32 rows × 64 B = 2048 B = 256 beats per instr.
//
// Two access patterns:
//   [3] Row-major  — scan the matrix row by row. Each SDMA covers 32 cols of
//                    one row; all 8 addresses are consecutive (stride = 8 B).
//   [4] Tile-major — scan tile by tile (32×32 grid of 32×32 tiles). One SDMA
//                    loads the entire tile row-by-row: 32 rows × 8 beats each
//                    = 256 beats. Within a tile-row beats are consecutive
//                    (stride = 8 B); between tile-rows stride = ROW_BYTES = 2048 B.
//
// Matrix layout:
//   1024 × 1024 × 2 bytes = 2 MB  |  row width = 2048 B
//   SDMA = 256 × 8 B = 2048 B/instr  |  1024 instrs per layout, 262144 txns total
//
// Data pattern: element[row][col] = (row * MAT_COLS + col) & 0xFFFF
//   Each beat packs 4 elements little-endian:
//     beat[15:0]  = col_base+0,  beat[31:16] = col_base+1, etc.
//
// Test plan:
//   [1] Reset + init
//   [2] Write matrix (262144 writes × 2 regions) — skipped with USE_MEMINIT=1
//   [3] Row-major read — 262144 beats, verify vs shadow
//   [4] Tile-major read — 262144 beats, verify vs shadow
//   [5] Timing report + pass/fail
//
// To skip the write phase: python3 scripts/gen_sdma_meminit.py
//   then: make sdma SDMA_MEMINIT=configs/sdma_meminit.bin

`timescale 1ns / 1ps

module test_sdma #(
    parameter string  CFG            = "configs/ddr4_config.yaml",
    // Forwarded to ramulator_sv_wrapper for functional_mem preload.
    // With USE_MEMINIT=1, phase [2] is skipped and shadow[] is filled from
    // pack_beat instead — no clock cycles wasted on writes.
    // Generate the binary: python3 scripts/gen_sdma_meminit.py
    parameter string  MEMINIT_FILE   = "",
    parameter string  MEMINIT_TYPE   = "bin",
    parameter longint MEMINIT_BASE   = 0,
    parameter bit     USE_MEMINIT    = 0
);
    import axi_bus_pkg::*;

    import "DPI-C" function void ramulator_exit(input int code);

    // ----------------------------------------------------------------
    // Matrix / tile / SDMA parameters
    // ----------------------------------------------------------------
    localparam int     B_DEPTH      = 4;
    localparam int     MAT_ROWS     = 1024;  // full matrix rows
    localparam int     MAT_COLS     = 1024;  // full matrix cols
    localparam int     TILE_ROWS    = 32;    // tile rows
    localparam int     TILE_COLS    = 32;    // tile cols
    localparam int     SDMA_ELEM_B  = 2;    // bytes per element (int16)
    localparam int     SDMA_TRANS_B = 8;    // bytes per AXI beat (64-bit)
    // One SDMA covers one full tile: TILE_ROWS * TILE_COLS * SDMA_ELEM_B / SDMA_TRANS_B = 256
    localparam int     SDMA_TRANS_N = TILE_ROWS * (TILE_COLS * SDMA_ELEM_B) / SDMA_TRANS_B;

    // Derived
    localparam int     ROW_BYTES    = MAT_COLS  * SDMA_ELEM_B;          // 2048 B per matrix row
    localparam int     ELEMS_PER_T  = SDMA_TRANS_B / SDMA_ELEM_B;      // 4 elements per beat
    localparam int     COL_GROUPS   = (MAT_COLS  * SDMA_ELEM_B) / SDMA_TRANS_B; // 256 beats per row
    localparam int     TILE_COL_GRP = (TILE_COLS * SDMA_ELEM_B) / SDMA_TRANS_B; // 8 beats per tile row
    localparam int     TOTAL_TRANS  = MAT_ROWS  * COL_GROUPS;           // 262144

    // two separate 2 MB regions, both hold the same matrix data
    localparam longint ROW_BASE     = 64'h0000_0000;
    localparam longint TILE_BASE    = 64'h0020_0000;   // 2 MB offset (≥ matrix size)

    localparam int     MAX_CYCLES   = 50_000_000;

    // ----------------------------------------------------------------
    // Clock / reset
    // ----------------------------------------------------------------
    logic clk  = 0;
    logic nrst = 0;
    always #0.625 clk = ~clk;   // 800 MHz

    // ----------------------------------------------------------------
    // AXI bus interface + DUT
    // ----------------------------------------------------------------
    axi_bus_if axi(.CLK(clk), .nRST(nrst));
    logic init_done;

    ramulator_sv_wrapper #(
        .CONFIG_FILE   (CFG),
        .B_DEPTH       (B_DEPTH),
        .MEM_INIT_FILE (MEMINIT_FILE),
        .MEM_INIT_TYPE (MEMINIT_TYPE),
        .MEM_INIT_BASE (MEMINIT_BASE)
    ) dut (
        .axi      (axi),
        .init_done(init_done)
    );

    // ----------------------------------------------------------------
    // Tick counter + shadow memory
    // ----------------------------------------------------------------
    longint tick = 0;
    always @(posedge clk) tick++;

    longint shadow [longint];   // addr → expected 64-bit beat value

    // ----------------------------------------------------------------
    // Scoreboards
    // ----------------------------------------------------------------
    int     wr_acc   = 0;
    int     row_ok   = 0, row_fail   = 0;
    int     tile_ok  = 0, tile_fail  = 0;
    longint row_cycles  = 0;
    longint tile_cycles = 0;

    // Pack 4 int16 elements starting at col_base into one 64-bit beat.
    // element value = (row * MAT_COLS + col_base + e) & 0xFFFF, little-endian.
    function automatic longint pack_beat(input int row, input int col_base);
        longint d;
        d = 64'h0;
        for (int e = 0; e < ELEMS_PER_T; e++)
            d |= (longint'(row * MAT_COLS + col_base + e) & 64'hFFFF) << (e * 16);
        return d;
    endfunction

    // Write the full matrix row by row, recording each value in shadow[].
    // Called twice (ROW_BASE and TILE_BASE) with identical data.
    task automatic write_matrix(input longint base_addr);
        longint addr, data;
        for (int row = 0; row < MAT_ROWS; row++) begin
            for (int cg = 0; cg < COL_GROUPS; cg++) begin
                addr = base_addr
                     + longint'(row) * ROW_BYTES
                     + longint'(cg)  * SDMA_TRANS_B;
                data = pack_beat(row, cg * ELEMS_PER_T);

                axi.aw_o_valid  = 1'b1;
                axi.aw_o.addr   = AWADDR'(addr);
                axi.aw_o.mid_id = '0;
                axi.aw_o.size   = 3'b011;
                axi.aw_o.len    = 4'h0;
                axi.aw_o.burst  = 2'b01;

                axi.w_o_valid   = 1'b1;
                axi.w_o.data    = WDATA'(data);
                axi.w_o.mid_id  = '0;
                axi.w_o.last    = 1'b1;
                axi.w_o.strb    = '1;

                @(posedge clk); #1;
                while (!axi.aw_o_ready) begin @(posedge clk); #1; end
                axi.aw_o_valid = 1'b0;
                axi.w_o_valid  = 1'b0;
                shadow[addr]   = data;
                wr_acc++;

                axi.b_i_ready = 1'b1;
                while (!axi.b_i_valid) begin @(posedge clk); #1; end
                @(posedge clk); #1;
                axi.b_i_ready = 1'b0;
            end
        end
    endtask

    // Fill q with row-major addresses: scan left to right, top to bottom.
    // Each entry is the start of one SDMA batch (8 consecutive 8-byte beats).
    task automatic sdma_gen_row_major(
        input longint base_addr,
        ref   longint q[$]
    );
        for (int r = 0; r < MAT_ROWS; r++) begin
            for (int t = 0; t < COL_GROUPS; t++) begin
                q.push_back(base_addr
                    + longint'(r) * ROW_BYTES
                    + longint'(t) * SDMA_TRANS_B);
            end
        end
    endtask

    // Fill q with tile-major addresses: iterate over 32×32 tiles, and within
    // each tile load row by row. One SDMA = one full tile = TILE_ROWS rows ×
    // TILE_COL_GRP beats each = 256 beats. Within a tile-row beats are
    // consecutive (stride = 8 B); between tile-rows stride = ROW_BYTES.
    task automatic sdma_gen_tile_major(
        input longint base_addr,
        ref   longint q[$]
    );
        for (int tr = 0; tr < MAT_ROWS / TILE_ROWS; tr++) begin    // 32 tile-row blocks
            for (int tc = 0; tc < MAT_COLS / TILE_COLS; tc++) begin // 32 tile-col blocks
                // Top-left byte address of this tile in the matrix
                longint tile_base = base_addr
                    + longint'(tr) * TILE_ROWS * ROW_BYTES
                    + longint'(tc) * TILE_COLS * SDMA_ELEM_B;
                // One SDMA loads the full tile row-by-row
                for (int r = 0; r < TILE_ROWS; r++) begin
                    for (int cg = 0; cg < TILE_COL_GRP; cg++) begin
                        q.push_back(tile_base
                            + longint'(r)  * ROW_BYTES
                            + longint'(cg) * SDMA_TRANS_B);
                    end
                end
            end
        end
    endtask

    // Drain the address queue SDMA_TRANS_N beats at a time.
    // Each iteration = one SDMA instruction (256 beats = one full tile):
    //   Phase A: fire SDMA_TRANS_N ARs with r_i_ready=0 (pipelined issue)
    //   Phase B: open r_i_ready, collect SDMA_TRANS_N responses, verify vs shadow
    //   Responses may come back out of order, so we match against the whole
    //   batch rather than expecting them in address order.
    task automatic sdma_drain(
        ref    longint              q[$],
        input  logic [MID_ARID-1:0] mid_id,
        output longint              total_cycles,
        ref    int                  ok_cnt,
        ref    int                  fail_cnt
    );
        longint t0;
        longint batch_addrs[SDMA_TRANS_N];
        longint batch_exp  [SDMA_TRANS_N];
        bit     consumed   [SDMA_TRANS_N];
        longint got;
        int     n;
        t0 = tick;

        while (q.size() > 0) begin
            n = (int'(q.size()) >= SDMA_TRANS_N) ? SDMA_TRANS_N : int'(q.size());

            // Pre-load expected values for this batch
            for (int i = 0; i < n; i++) begin
                batch_addrs[i] = q.pop_front();
                batch_exp[i]   = shadow[batch_addrs[i]];
                consumed[i]    = 0;
            end

            // Phase A: issue n ARs back-to-back, hold r_i_ready low
            axi.r_i_ready = 1'b0;
            for (int i = 0; i < n; i++) begin
                axi.ar_o_valid  = 1'b1;
                axi.ar_o.addr   = ARADDR'(batch_addrs[i]);
                axi.ar_o.mid_id = mid_id;
                axi.ar_o.size   = 3'b011;
                axi.ar_o.len    = 4'h0;
                axi.ar_o.burst  = 2'b01;

                @(posedge clk); #1;
                while (!axi.ar_o_ready) begin @(posedge clk); #1; end
                axi.ar_o_valid = 1'b0;
            end

            // Phase B: collect n responses and verify against shadow
            axi.r_i_ready = 1'b1;
            for (int i = 0; i < n; i++) begin
                while (!axi.r_i_valid) begin @(posedge clk); #1; end
                got = longint'(axi.r_i.data);
                @(posedge clk); #1;

                // find a matching unconsumed entry in the batch
                begin
                    bit found;
                    found = 0;
                    for (int j = 0; j < n; j++) begin
                        if (!consumed[j] && got === batch_exp[j]) begin
                            found     = 1;
                            consumed[j] = 1;
                            break;
                        end
                    end
                    if (found) ok_cnt++;
                    else begin
                        $display("    [MISMATCH] got=0x%016h not in expected batch (addrs 0x%08h..0x%08h)",
                                 got, batch_addrs[0], batch_addrs[n-1]);
                        fail_cnt++;
                    end
                end
            end
            axi.r_i_ready = 1'b0;
        end

        total_cycles = tick - t0;
    endtask

    // ----------------------------------------------------------------
    // Main test
    // ----------------------------------------------------------------
    initial begin : main
        longint q[$];

        // start with all master channels idle
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
        $display("=== SDMA Tile Load Test ===");
        $display("Config : %s", CFG);
        $display("Matrix : %0dx%0d int16  (%0d bytes)",
                 MAT_ROWS, MAT_COLS, MAT_ROWS * MAT_COLS * SDMA_ELEM_B);
        $display("Tile   : %0dx%0d int16  (%0d bytes)  |  %0dx%0d tile grid",
                 TILE_ROWS, TILE_COLS, TILE_ROWS * TILE_COLS * SDMA_ELEM_B,
                 MAT_ROWS / TILE_ROWS, MAT_COLS / TILE_COLS);
        $display("SDMA   : %0d beats x %0d bytes = %0d bytes/instr  |  %0d instrs/layout",
                 SDMA_TRANS_N, SDMA_TRANS_B, SDMA_TRANS_N * SDMA_TRANS_B,
                 TOTAL_TRANS / SDMA_TRANS_N);
        $write("[1] ramulator_init ... ");
        wait(init_done);
        @(posedge clk); #1;
        $display("OK");

        // ============================================================
        // [2] Write matrix to both regions — or just fill shadow[] if using meminit.
        // With meminit the wrapper already loaded functional_mem from file; we just
        // need shadow[] to match so phases [3]/[4] know what to expect.
        // ============================================================
        if (USE_MEMINIT) begin
            $display("[2] (meminit active) Skipping AXI writes - pre-loading shadow[] ...");
            for (int row = 0; row < MAT_ROWS; row++) begin
                for (int cg = 0; cg < COL_GROUPS; cg++) begin
                    longint addr_row, addr_tile, data;
                    data      = pack_beat(row, cg * ELEMS_PER_T);
                    addr_row  = ROW_BASE  + longint'(row) * ROW_BYTES
                                          + longint'(cg)  * SDMA_TRANS_B;
                    addr_tile = TILE_BASE + longint'(row) * ROW_BYTES
                                          + longint'(cg)  * SDMA_TRANS_B;
                    shadow[addr_row]  = data;
                    shadow[addr_tile] = data;
                end
            end
            $display("    shadow pre-loaded: %0d entries (ROW_BASE + TILE_BASE)",
                     2 * TOTAL_TRANS);
        end else begin
            $display("[2] Writing matrix to ROW_BASE=0x%08h ...", ROW_BASE);
            write_matrix(ROW_BASE);
            $display("    %0d writes done (ROW_BASE)", wr_acc);

            $display("[2] Writing matrix to TILE_BASE=0x%08h ...", TILE_BASE);
            write_matrix(TILE_BASE);
            $display("    %0d writes done total", wr_acc);
        end

        // ============================================================
        // [3] Row-major load — scan left to right, top to bottom.
        //     32 SDMAs per row (32 cols × 2 B = 8 beats each); addresses
        //     within one SDMA are consecutive so they hit the same DRAM row.
        // ============================================================
        $display("[3] Row-major load: %0d SDMAs x %0d beats = %0d transactions ...",
                 TOTAL_TRANS / SDMA_TRANS_N, SDMA_TRANS_N, TOTAL_TRANS);
        sdma_gen_row_major(ROW_BASE, q);
        $display("    Queue: %0d entries  stride within instr = %0d B",
                 q.size(), SDMA_TRANS_B);
        sdma_drain(q, MID_ARID'(0), row_cycles, row_ok, row_fail);
        $display("    Row-major:  OK=%0d/%0d  FAIL=%0d  cycles=%0d  avg=%0d/trans",
                 row_ok, TOTAL_TRANS, row_fail,
                 row_cycles, row_cycles / (row_ok + row_fail > 0 ? row_ok + row_fail : 1));

        // ============================================================
        // [4] Tile-major load — read tile by tile (32×32 grid of 32×32 tiles).
        //     Within each tile: column-by-column, each SDMA strides ROW_BYTES
        //     (2048 B) across 8 consecutive rows. Every beat is in a different
        //     DRAM row, so expect more row activations than row-major.
        // ============================================================
        $display("[4] Tile-major load: %0d SDMAs x %0d beats = %0d transactions ...",
                 TOTAL_TRANS / SDMA_TRANS_N, SDMA_TRANS_N, TOTAL_TRANS);
        sdma_gen_tile_major(TILE_BASE, q);
        $display("    Queue: %0d entries  stride within tile-row = %0d B  tile-row stride = %0d B",
                 q.size(), SDMA_TRANS_B, ROW_BYTES);
        sdma_drain(q, MID_ARID'(0), tile_cycles, tile_ok, tile_fail);
        $display("    Tile-major: OK=%0d/%0d  FAIL=%0d  cycles=%0d  avg=%0d/trans",
                 tile_ok, TOTAL_TRANS, tile_fail,
                 tile_cycles, tile_cycles / (tile_ok + tile_fail > 0 ? tile_ok + tile_fail : 1));

        // ============================================================
        // [5] Timing report + pass/fail
        //     100 MHz clock → 1 cycle = 10 ns
        //     Bandwidth = 2 MB / cycles [B/cycle] or / time [GB/s at 100 MHz]
        // ============================================================
        begin
            longint data_bytes, row_ps, tile_ps;
            longint row_bw_int, row_bw_frac;   // bandwidth integer and centesimal fraction
            longint tile_bw_int, tile_bw_frac;
            longint row_us_int, row_us_frac;   // microseconds integer and decimal
            longint tile_us_int, tile_us_frac;
            int     num_sdmas;

            data_bytes = longint'(MAT_ROWS) * MAT_COLS * SDMA_ELEM_B;  // 2 097 152 B
            num_sdmas  = TOTAL_TRANS / SDMA_TRANS_N;

            // 800 MHz: 1 cycle = 1250 ps
            row_ps  = row_cycles  * 1250;
            tile_ps = tile_cycles * 1250;

            row_us_int   = row_ps  / 1_000_000;
            row_us_frac  = (row_ps  % 1_000_000) / 10_000;
            tile_us_int  = tile_ps / 1_000_000;
            tile_us_frac = (tile_ps % 1_000_000) / 10_000;

            // scale ×100 so we can print two decimal places without floats
            row_bw_int   = (data_bytes * 100) / (row_cycles  > 0 ? row_cycles  : 1);
            tile_bw_int  = (data_bytes * 100) / (tile_cycles > 0 ? tile_cycles : 1);
            row_bw_frac  = row_bw_int  % 100;  row_bw_int  /= 100;
            tile_bw_frac = tile_bw_int % 100;  tile_bw_int /= 100;

            $display("\n--- SDMA Load Timing Report ---");
            $display("  Config         : %s", CFG);
            $display("  Matrix         : %0dx%0d int16 = %0d KB",
                     MAT_ROWS, MAT_COLS, data_bytes / 1024);
            $display("  Tile           : %0dx%0d  |  %0dx%0d tile grid  |  %0d tiles",
                     TILE_ROWS, TILE_COLS,
                     MAT_ROWS/TILE_ROWS, MAT_COLS/TILE_COLS,
                     (MAT_ROWS/TILE_ROWS)*(MAT_COLS/TILE_COLS));
            $display("  SDMA instr     : %0d beats x %0d B = %0d B/instr  |  %0d instrs/layout",
                     SDMA_TRANS_N, SDMA_TRANS_B, SDMA_TRANS_N*SDMA_TRANS_B, num_sdmas);
            $display("  Clock          : 800 MHz  (1 cycle = 1.25 ns)");
            if (USE_MEMINIT)
                $display("  [2] Writes     : SKIPPED (meminit preload)");
            else
                $display("  [2] Writes     : %0d total (%0d per region)", wr_acc, wr_acc/2);
            $display("");
            $display("  %-14s  %12s  %12s  %14s  %10s  %12s",
                     "Pattern", "Cycles", "Time (us)", "Cyc/SDMA instr", "Cyc/beat", "B/cycle");
            $display("  %-14s  %12s  %12s  %14s  %10s  %12s",
                     "--------------", "------------", "------------",
                     "--------------", "----------", "------------");
            $display("  %-14s  %12d  %9d.%02d  %14d  %10d  %8d.%02d",
                     "Row-major",
                     row_cycles,
                     row_us_int, row_us_frac,
                     row_cycles / (num_sdmas > 0 ? num_sdmas : 1),
                     row_cycles / (TOTAL_TRANS > 0 ? TOTAL_TRANS : 1),
                     row_bw_int, row_bw_frac);
            $display("  %-14s  %12d  %9d.%02d  %14d  %10d  %8d.%02d",
                     "Tile-major",
                     tile_cycles,
                     tile_us_int, tile_us_frac,
                     tile_cycles / (num_sdmas > 0 ? num_sdmas : 1),
                     tile_cycles / (TOTAL_TRANS > 0 ? TOTAL_TRANS : 1),
                     tile_bw_int, tile_bw_frac);
            $display("");
            $display("  Correctness    : row OK=%0d/%0d FAIL=%0d  |  tile OK=%0d/%0d FAIL=%0d",
                     row_ok, TOTAL_TRANS, row_fail, tile_ok, TOTAL_TRANS, tile_fail);
        end

        if ((USE_MEMINIT ? (wr_acc == 0) : (wr_acc == 2 * TOTAL_TRANS)) &&
            row_ok   == TOTAL_TRANS      && row_fail  == 0 &&
            tile_ok  == TOTAL_TRANS      && tile_fail == 0)
        begin
            $display("\n=== PASSED ===");
            ramulator_exit(0);
        end else begin
            $display("\n=== FAILED ===");
            ramulator_exit(1);
        end
    end

    // ----------------------------------------------------------------
    // Watchdog — abort if the sim runs away
    // ----------------------------------------------------------------
    initial begin : watchdog
        repeat(MAX_CYCLES) @(posedge clk);
        $display("WATCHDOG: simulation exceeded %0d cycles — aborting.", MAX_CYCLES);
        ramulator_exit(1);
    end

endmodule
