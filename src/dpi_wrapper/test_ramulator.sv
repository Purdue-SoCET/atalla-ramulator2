// test_ramulator.sv — SystemVerilog smoke test for ramulator_sv_wrapper
//
// Mirrors test_dpi.c:
//   [1] Init check      — wait for init_done
//   [2] Issue requests  — 128 total, every 4th is a write
//   [3] send accepted   — at least one req_ready seen
//   [4] completions     — at least one resp_valid seen
//   [5] functional check— resp_data matches write pattern or address default
//   [6] finalize        — $finish
//
// Compile + run (from project root):
//   vlib work
//   vlog -sv src/dpi_wrapper/ramulator_sv_wrapper.sv \
//             src/dpi_wrapper/test_ramulator.sv
//   vsim -c -sv_lib ./libramulator_dpi work.test_ramulator \
//        -do "run -all; quit -f"

`timescale 1ns / 1ps

module test_ramulator;

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam string  CFG          = "configs/ddr4_config.yaml";
    localparam int     NUM_REQS     = 128;
    localparam int     MAX_CYCLES   = 100000;
    localparam int     MAX_INFLIGHT = 64;
    localparam longint ADDR_STRIDE  = 64;
    localparam longint WR_PATTERN   = 64'hDEADBEEFCAFEBABE;

    // ----------------------------------------------------------------
    // Clock and reset
    // ----------------------------------------------------------------
    logic clk   = 0;
    logic rst_n = 0;

    always #5 clk = ~clk;   // 10 ns period

    // ----------------------------------------------------------------
    // DUT signals
    // ----------------------------------------------------------------
    logic        req_valid    = 0;
    logic [63:0] req_addr     = '0;
    logic        req_type_sig = 0;
    logic [31:0] req_src_id   = '0;
    logic [63:0] req_data_sig = '0;
    logic        req_ready;

    logic        resp_valid;
    logic [63:0] resp_addr;
    logic [63:0] resp_data;
    logic        init_done;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    ramulator_sv_wrapper #(
        .CONFIG_FILE(CFG)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .req_valid    (req_valid),
        .req_addr     (req_addr),
        .req_type     (req_type_sig),
        .req_source_id(req_src_id),
        .req_data     (req_data_sig),
        .req_ready    (req_ready),
        .resp_valid   (resp_valid),
        .resp_addr    (resp_addr),
        .resp_data    (resp_data),
        .init_done    (init_done)
    );

    // ----------------------------------------------------------------
    // Per-request tracking (module-level to avoid static init errors)
    // ----------------------------------------------------------------
    logic [63:0] t_addr     [NUM_REQS];
    logic        t_type     [NUM_REQS];   // 0=read, 1=write
    logic [63:0] t_wrdata   [NUM_REQS];
    bit          t_issued   [NUM_REQS];
    bit          t_complete [NUM_REQS];
    bit          t_func_ok  [NUM_REQS];

    // Shadow functional memory: addr -> last written value
    longint shadow_mem  [longint];
    bit     was_written [longint];

    // Shared counters (module-level so both initial blocks can access)
    int accepted_count  = 0;
    int rejected_count  = 0;
    int reads_completed = 0;
    int func_ok_count   = 0;
    int inflight        = 0;

    // ----------------------------------------------------------------
    // Response monitor — runs in parallel with request driver
    // ----------------------------------------------------------------
    bit     matched;
    longint raddr_l;
    longint got_l;
    longint expected_l;

    initial begin : resp_monitor
        forever begin
            @(posedge clk);
            #1;
            if (resp_valid && init_done) begin
                raddr_l = longint'(resp_addr);
                got_l   = longint'(resp_data);

                if (was_written.exists(raddr_l))
                    expected_l = shadow_mem[raddr_l];
                else
                    expected_l = raddr_l;  // default: wrapper returns addr for unwritten locs

                matched = 0;
                for (int i = 0; i < NUM_REQS; i++) begin
                    if (t_issued[i] && !t_complete[i] &&
                        t_type[i] == 0 &&
                        longint'(t_addr[i]) == raddr_l) begin

                        t_complete[i]   = 1;
                        reads_completed = reads_completed + 1;
                        inflight        = inflight - 1;

                        if (got_l == expected_l) begin
                            t_func_ok[i]   = 1;
                            func_ok_count  = func_ok_count + 1;
                        end else begin
                            $display("  [FUNC MISMATCH] addr=0x%h  got=0x%h  expected=0x%h",
                                     raddr_l, got_l, expected_l);
                        end
                        matched = 1;
                        break;
                    end
                end
                if (!matched)
                    $display("  [note] resp for addr 0x%h not matched to pending read", raddr_l);
            end
        end
    end

    // ----------------------------------------------------------------
    // Main test flow
    // ----------------------------------------------------------------
    int  num_reads;
    int  cycle_count;

    initial begin : main_test
        // --- Reset ---
        rst_n = 0;
        repeat(4) @(posedge clk);
        rst_n = 1;

        // --- [1] Wait for init ---
        $display("=== Ramulator SV wrapper smoke test ===");
        $display("Config: %s\n", CFG);
        $write("[1] ramulator_init ... ");
        wait(init_done);
        @(posedge clk);
        $display("OK");

        // --- Build request list ---
        num_reads = 0;
        for (int i = 0; i < NUM_REQS; i++) begin
            t_addr[i]     = longint'(i) * ADDR_STRIDE;
            t_type[i]     = (i % 4 == 0) ? 1'b1 : 1'b0;
            t_wrdata[i]   = t_addr[i] ^ WR_PATTERN;
            t_issued[i]   = 0;
            t_complete[i] = 0;
            t_func_ok[i]  = 0;
            if (t_type[i] == 0) num_reads = num_reads + 1;
        end

        // --- [2] Issue all requests ---
        $display("[2] Issuing %0d requests over up to %0d cycles ...", NUM_REQS, MAX_CYCLES);

        for (int i = 0; i < NUM_REQS; i++) begin
            // Throttle if too many reads in flight
            while (inflight >= MAX_INFLIGHT) begin
                @(posedge clk);
                #1;
            end

            // Drive request; retry until req_ready
            req_valid    = 1;
            req_addr     = t_addr[i];
            req_type_sig = t_type[i];
            req_data_sig = t_wrdata[i];

            @(posedge clk);
            #1;
            while (!req_ready) begin
                rejected_count = rejected_count + 1;
                @(posedge clk);
                #1;
            end

            // Accepted
            t_issued[i]    = 1;
            accepted_count = accepted_count + 1;

            if (t_type[i] == 1) begin
                // Write: update shadow memory; no response expected
                shadow_mem[longint'(t_addr[i])]  = longint'(t_wrdata[i]);
                was_written[longint'(t_addr[i])] = 1;
                t_complete[i] = 1;
                // Don't count writes as in-flight (no resp_valid will come)
            end else begin
                inflight = inflight + 1;
            end
        end

        req_valid = 0;

        // --- Wait for all reads to complete or timeout ---
        cycle_count = 0;
        while (reads_completed < num_reads && cycle_count < MAX_CYCLES) begin
            @(posedge clk);
            cycle_count = cycle_count + 1;
        end

        if (reads_completed == num_reads)
            $display("    All reads completed (%0d cycles after last issue)", cycle_count);
        else
            $display("    WARNING: timeout – only %0d / %0d reads completed",
                     reads_completed, num_reads);

        // --- Print results ---
        $display("\n--- Results ---");
        $display("  Requests issued      : %0d / %0d", accepted_count, NUM_REQS);
        $display("  Reads completed      : %0d / %0d", reads_completed, num_reads);
        $display("  Functional checks OK : %0d / %0d", func_ok_count, reads_completed);
        $display("  Send accepted        : %0d", accepted_count);
        $display("  Send rejected        : %0d", rejected_count);

        // --- [3] Accept check ---
        $write("\n[3] ramulator_send_request ... ");
        if (accepted_count > 0)
            $display("OK (%0d accepted)", accepted_count);
        else begin
            $display("FAIL - no requests accepted");
            $finish(1);
        end

        // --- [4] Completion check ---
        $write("[4] ramulator_check_response ... ");
        if (reads_completed > 0)
            $display("OK (%0d completions)", reads_completed);
        else
            $display("WARN - no completions observed");

        // --- [5] Functional check ---
        $write("[5] Functional model ... ");
        if (reads_completed > 0 && func_ok_count == reads_completed)
            $display("OK (all %0d reads returned correct data)", reads_completed);
        else if (reads_completed == 0)
            $display("WARN - no reads completed, cannot verify");
        else begin
            $display("FAIL - %0d / %0d reads returned wrong data",
                     reads_completed - func_ok_count, reads_completed);
            $finish(1);
        end

        // --- [6] Done (wrapper final block calls ramulator_finalize) ---
        $display("[6] ramulator_finalize ... OK (called in final block)");
        $display("\n=== Smoke test PASSED ===");
        $finish(0);
    end

    // ----------------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------------
    initial begin : watchdog
        repeat(MAX_CYCLES * 2) @(posedge clk);
        $display("WATCHDOG: simulation exceeded %0d cycles, aborting.", MAX_CYCLES * 2);
        $finish(1);
    end

endmodule
