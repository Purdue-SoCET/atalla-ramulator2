// test_sdma.sv — SDMA tile load testbench for ramulator_sv_wrapper
//
// Emulates the AXI read transactions that an SDMA instruction would generate
// when loading tiles from a 1024x1024 matrix of int16 (2-byte) elements.
//
// The SDMA instruction issues 8 back-to-back 64-bit (8-byte) single-beat reads,
// covering one 32-element tile-row (32 cols × 2 bytes = 64 bytes = 8 beats).
// Two access patterns are tested:
//
//   [3] Row-major  — reads the matrix row by row.  Each SDMA covers 32 cols
//                    (one tile-width) of a single matrix row; 32 SDMAs span the
//                    full 1024-col row.  All 8 addresses within an SDMA are
//                    consecutive (stride = 8 bytes).
//
//   [4] Tile-major — reads the matrix tile by tile (32×32 tiles).  Within each
//                    tile the data is loaded column-by-column: each SDMA reads
//                    8 bytes from 8 consecutive tile rows at the same column
//                    position (stride = ROW_BYTES = 2048 bytes).  Each beat is
//                    in a different DRAM row → more row activations than row-major.
//
// Matrix layout:
//   1024 rows × 1024 cols × 2 bytes/elem = 2 MB total
//   Row width = 2048 bytes  |  SDMA instruction = 8 × 8 bytes = 64 bytes
//   Tile size = 32×32 elements = 2048 bytes  |  Tile grid = 32×32 = 1024 tiles
//   Total SDMA instructions per layout = 262144 / 8 = 32768
//   Total AXI transactions per layout  = 262144
//
// Data pattern:
//   Each int16 element stores (row * MAT_COLS + col) truncated to 16 bits.
//   Each 64-bit AXI beat packs 4 consecutive int16 elements (little-endian):
//     beat_data[15:0]  = element at col_base + 0
//     beat_data[31:16] = element at col_base + 1
//     beat_data[47:32] = element at col_base + 2
//     beat_data[63:48] = element at col_base + 3
//   shadow[] records the expected 64-bit value for every written address.
//
// Test plan:
//   [1] Reset + ramulator_init
//   [2] Write matrix — 262144 writes to ROW_BASE, 262144 writes to TILE_BASE
//   [3] Row-major load  — 262144 coalesced single-beat reads; verify vs shadow
//   [4] Tile-major load — 262144 strided  single-beat reads; verify vs shadow
//   [5] Pass/fail summary

`timescale 1ns / 1ps

module test_sdma #(
    parameter string CFG = "configs/ddr4_config.yaml"
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
    // One SDMA covers one tile-row: TILE_COLS * SDMA_ELEM_B / SDMA_TRANS_B = 8
    localparam int     SDMA_TRANS_N = (TILE_COLS * SDMA_ELEM_B) / SDMA_TRANS_B;

    // Derived
    localparam int     ROW_BYTES    = MAT_COLS  * SDMA_ELEM_B;          // 2048 B per matrix row
    localparam int     ELEMS_PER_T  = SDMA_TRANS_B / SDMA_ELEM_B;      // 4 elements per beat
    localparam int     COL_GROUPS   = (MAT_COLS  * SDMA_ELEM_B) / SDMA_TRANS_B; // 256 beats per row
    localparam int     TILE_COL_GRP = (TILE_COLS * SDMA_ELEM_B) / SDMA_TRANS_B; // 8 beats per tile row
    localparam int     TILE_ROW_GRP = TILE_ROWS / SDMA_TRANS_N;        // 4 SDMA row-groups per tile
    localparam int     TOTAL_TRANS  = MAT_ROWS  * COL_GROUPS;           // 262144

    // Base addresses — two separate 2 MB memory regions, same data layout
    localparam longint ROW_BASE     = 64'h0000_0000;
    localparam longint TILE_BASE    = 64'h0020_0000;   // 2 MB offset (≥ matrix size)

    localparam int     MAX_CYCLES   = 50_000_000;

    // ----------------------------------------------------------------
    // Clock / reset
    // ----------------------------------------------------------------
    logic clk  = 0;
    logic nrst = 0;
    always #5 clk = ~clk;   // 100 MHz

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

    // ----------------------------------------------------------------
    // Helper: pack 4 consecutive int16 elements into one 64-bit beat.
    //   element[row][col] = (row * MAT_COLS + col) & 0xFFFF (truncated to 16b)
    //   layout: beat[16*e +: 16] = element at (col_base + e)
    // ----------------------------------------------------------------
    function automatic longint pack_beat(input int row, input int col_base);
        longint d;
        d = 64'h0;
        for (int e = 0; e < ELEMS_PER_T; e++)
            d |= (longint'(row * MAT_COLS + col_base + e) & 64'hFFFF) << (e * 16);
        return d;
    endfunction

    // ----------------------------------------------------------------
    // Task: write the full 1024x1024 matrix to a memory region.
    //
    //   Sequentially writes all MAT_ROWS × COL_GROUPS x 8-byte beats
    //   (row by row, beat by beat) and records each value in shadow[].
    //   Both ROW_BASE and TILE_BASE are written with identical data so
    //   the same expected values apply regardless of read access order.
    // ----------------------------------------------------------------
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

    // ----------------------------------------------------------------
    // Task: push row-major SDMA load addresses into queue.
    //
    //   Reads the entire matrix row by row.  Each SDMA covers
    //   SDMA_TRANS_N = 8 consecutive 8-byte beats (one tile-row width
    //   = 32 cols × 2 bytes = 64 bytes).  Addresses within one SDMA
    //   are consecutive (stride = SDMA_TRANS_B = 8 bytes).
    // ----------------------------------------------------------------
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

    // ----------------------------------------------------------------
    // Task: push tile-major SDMA load addresses into queue.
    //
    //   Reads the matrix tile by tile (32×32-element tiles).  Within
    //   each tile the data is loaded column-by-column: each SDMA reads
    //   SDMA_TRANS_N = 8 bytes from 8 consecutive tile rows at the same
    //   column position.  Stride within one SDMA = ROW_BYTES = 2048 bytes.
    //   Each beat is in a different DRAM row → more row activations.
    // ----------------------------------------------------------------
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
                // Within tile: column groups × row sub-groups × beats
                for (int cg = 0; cg < TILE_COL_GRP; cg++) begin    // 8 col groups
                    for (int rg = 0; rg < TILE_ROW_GRP; rg++) begin // 4 row groups of 8 rows
                        longint sdma_base = tile_base
                            + longint'(cg) * SDMA_TRANS_B
                            + longint'(rg) * SDMA_TRANS_N * ROW_BYTES;
                        for (int t = 0; t < SDMA_TRANS_N; t++) begin // 8 rows
                            q.push_back(sdma_base + longint'(t) * ROW_BYTES);
                        end
                    end
                end
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Task: drain queue into Ramulator, SDMA_TRANS_N beats at a time.
    //
    //   Each iteration models one SDMA instruction:
    //     Phase A — issue SDMA_TRANS_N ARs back-to-back (r_i_ready=0).
    //     Phase B — assert r_i_ready, collect SDMA_TRANS_N responses.
    //               Each response is matched against shadow[addr] for
    //               its corresponding address.  A "consumed" flag ensures
    //               each expected value is matched exactly once even if
    //               Ramulator returns responses out of order.
    // ----------------------------------------------------------------
    task automatic sdma_drain(
        ref    longint              q[$],
        input  logic [MID_ARID-1:0] mid_id,
        output longint              total_cycles,
        ref    int                  ok_cnt,
        ref    int                  fail_cnt
    );
        longint t0;
        longint batch_addrs[8];
        longint batch_exp  [8];
        bit     consumed   [8];
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

            // --- Phase A: issue n ARs, hold r_i_ready low ---
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

            // --- Phase B: collect n R responses; verify vs shadow ---
            axi.r_i_ready = 1'b1;
            for (int i = 0; i < n; i++) begin
                while (!axi.r_i_valid) begin @(posedge clk); #1; end
                got = longint'(axi.r_i.data);
                @(posedge clk); #1;

                // Match got against an unconsumed expected value in batch
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

        // Quiesce all master-driven channels
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
        // [2] Write matrix to both memory regions
        //
        //   element[row][col] = (row*1024 + col) & 0xFFFF packed into
        //   64-bit beats.  Both ROW_BASE and TILE_BASE get the same
        //   data layout so shadow[] entries cover both read tests.
        // ============================================================
        $display("[2] Writing matrix to ROW_BASE=0x%08h ...", ROW_BASE);
        write_matrix(ROW_BASE);
        $display("    %0d writes done (ROW_BASE)", wr_acc);

        $display("[2] Writing matrix to TILE_BASE=0x%08h ...", TILE_BASE);
        write_matrix(TILE_BASE);
        $display("    %0d writes done total", wr_acc);

        // ============================================================
        // [3] Row-major load
        //
        //   Reads the full matrix row by row.  Each SDMA reads 32 cols
        //   (8 consecutive 8-byte beats, stride = 8 bytes).  32 SDMAs
        //   span one 1024-col row; all addresses are in the same DRAM row.
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
        // [4] Tile-major load
        //
        //   Reads the matrix tile by tile (32×32 tiles, 32×32 tile grid).
        //   Within each tile data is loaded column-by-column: each SDMA
        //   reads 8 bytes from 8 consecutive rows at the same column
        //   position (stride = ROW_BYTES = 2048 bytes).  Each beat is in
        //   a different DRAM row → more row activations than row-major.
        // ============================================================
        $display("[4] Tile-major load: %0d SDMAs x %0d beats = %0d transactions ...",
                 TOTAL_TRANS / SDMA_TRANS_N, SDMA_TRANS_N, TOTAL_TRANS);
        sdma_gen_tile_major(TILE_BASE, q);
        $display("    Queue: %0d entries  stride within instr = %0d B (= 1 row)",
                 q.size(), ROW_BYTES);
        sdma_drain(q, MID_ARID'(0), tile_cycles, tile_ok, tile_fail);
        $display("    Tile-major: OK=%0d/%0d  FAIL=%0d  cycles=%0d  avg=%0d/trans",
                 tile_ok, TOTAL_TRANS, tile_fail,
                 tile_cycles, tile_cycles / (tile_ok + tile_fail > 0 ? tile_ok + tile_fail : 1));

        // ============================================================
        // [5] Summary
        // ============================================================
        $display("\n--- Summary ---");
        $display("  Config        : %s", CFG);
        $display("  [2] Writes    : %0d total (%0d per region)", wr_acc, wr_acc / 2);
        $display("  [3] Row-major : OK=%0d/%0d  FAIL=%0d  cycles=%0d",
                 row_ok, TOTAL_TRANS, row_fail, row_cycles);
        $display("  [4] Tile-major: OK=%0d/%0d  FAIL=%0d  cycles=%0d",
                 tile_ok, TOTAL_TRANS, tile_fail, tile_cycles);
        $display("  Tile/row latency ratio: %0d.%02dx",
                 tile_cycles / (row_cycles > 0 ? row_cycles : 1),
                 (tile_cycles * 100 / (row_cycles > 0 ? row_cycles : 1)) % 100);

        if (wr_acc   == 2 * TOTAL_TRANS  &&
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
    // Watchdog
    // ----------------------------------------------------------------
    initial begin : watchdog
        repeat(MAX_CYCLES) @(posedge clk);
        $display("WATCHDOG: simulation exceeded %0d cycles — aborting.", MAX_CYCLES);
        ramulator_exit(1);
    end

endmodule
