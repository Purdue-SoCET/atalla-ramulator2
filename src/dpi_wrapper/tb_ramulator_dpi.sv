// tb_ramulator_dpi.sv
//
// SV smoke-test testbench for ramulator_sv_wrapper.
// Mirrors the behaviour of test_dpi.c:
//   - 128 requests (every 4th is a Write, rest are Reads)
//   - up to MAX_INFLIGHT=64 outstanding at once
//   - runs for up to MAX_CYCLES=50000 cycles
//   - reports pass / fail to match the C test output
`timescale 1ns/1ps

module tb_ramulator_dpi;

    // ================================================================
    // Parameters (mirror test_dpi.c constants)
    // ================================================================
    localparam int    NUM_REQUESTS = 128;
    localparam int    MAX_INFLIGHT = 64;
    localparam int    MAX_CYCLES   = 50000;
    localparam int    STALL_LIMIT  = 10000;
    localparam [63:0] BASE_ADDR    = 64'h0000_0000_0000_0000;
    localparam [63:0] ADDR_STRIDE  = 64'h40;   // 64-byte cache-line
    localparam real   CLK_PERIOD   = 10.0;      // 100 MHz

    // ================================================================
    // DUT signals
    // ================================================================
    logic        clk           = 1'b0;
    logic        rst_n         = 1'b0;
    logic        req_valid     = 1'b0;
    logic [63:0] req_addr      = '0;
    logic        req_type      = 1'b0;   // 0=Rd 1=Wr
    logic [31:0] req_source_id = '0;
    logic        req_ready;
    logic        resp_valid;
    logic [63:0] resp_addr;
    logic        init_done;

    // ================================================================
    // DUT instantiation
    // ================================================================
    ramulator_sv_wrapper #(
        .CONFIG_FILE("dpi_test_config.yaml")
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .req_valid    (req_valid),
        .req_addr     (req_addr),
        .req_type     (req_type),
        .req_source_id(req_source_id),
        .req_ready    (req_ready),
        .resp_valid   (resp_valid),
        .resp_addr    (resp_addr),
        .init_done    (init_done)
    );

    // ================================================================
    // Clock
    // ================================================================
    always #(CLK_PERIOD/2.0) clk = ~clk;

    // ================================================================
    // Scoreboard
    // Shared between the response monitor (always block) and stimulus
    // (initial block). Initial block reads at posedge+#1 so the always
    // block has already committed its updates.
    // ================================================================
    logic req_issued    [NUM_REQUESTS];
    logic req_completed [NUM_REQUESTS];
    int   inflight        = 0;
    int   total_completed = 0;

    // ================================================================
    // Response monitor
    // ================================================================
    always @(posedge clk) begin
        if (resp_valid) begin
            for (int i = 0; i < NUM_REQUESTS; i++) begin
                if (req_issued[i] && !req_completed[i] &&
                    resp_addr == (BASE_ADDR + i * ADDR_STRIDE)) begin
                    req_completed[i]  = 1'b1;
                    total_completed   = total_completed + 1;
                    inflight          = inflight - 1;
                    break;
                end
            end
        end
    end

    // ================================================================
    // Stimulus
    // ================================================================
    initial begin
        // -- init scoreboard --
        for (int i = 0; i < NUM_REQUESTS; i++) begin
            req_issued[i]    = 1'b0;
            req_completed[i] = 1'b0;
        end

        // -- reset --
        repeat(4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk iff init_done);
        @(posedge clk);

        $display("[TB] === Ramulator DPI SV smoke test ===");
        $display("[TB] Issuing %0d requests (up to %0d cycles).",
                 NUM_REQUESTS, MAX_CYCLES);

        // -- request phase --
        begin
            automatic int accepted  = 0;
            automatic int rejected  = 0;
            automatic int stall_cnt = 0;

            for (int i = 0; i < NUM_REQUESTS; ) begin

                // Stall if inflight window is full
                if (inflight >= MAX_INFLIGHT) begin
                    req_valid = 1'b0;
                    @(posedge clk); #1;
                    continue;
                end

                // Present request i
                req_valid = 1'b1;
                req_addr  = BASE_ADDR + 64'(i) * ADDR_STRIDE;
                req_type  = (i % 4 == 0) ? 1'b1 : 1'b0;   // every 4th is write

                // req_ready is registered: give DUT one posedge to sample
                @(posedge clk); #1;

                if (req_ready) begin
                    req_issued[i] = 1'b1;
                    inflight++;
                    accepted++;
                    stall_cnt = 0;
                    i++;                    // advance to next request
                end else begin
                    rejected++;
                    stall_cnt++;
                    if (stall_cnt > STALL_LIMIT) begin
                        $display("[TB] WARNING: stalled for %0d cycles – deadlock?",
                                 STALL_LIMIT);
                        break;
                    end
                end
            end

            req_valid = 1'b0;
            $display("[TB] Send phase done. accepted=%0d rejected=%0d",
                     accepted, rejected);

            // -- drain completions (or timeout) --
            for (int c = 0; c < MAX_CYCLES; c++) begin
                if (total_completed >= NUM_REQUESTS) begin
                    $display("[TB] All requests completed at drain cycle %0d.", c);
                    break;
                end
                @(posedge clk); #1;
            end

            // -- report --
            $display("\n--- Results ---");
            $display("  Requests sent      : %0d / %0d", accepted,        NUM_REQUESTS);
            $display("  Requests completed : %0d / %0d", total_completed, accepted);
            $display("  Send accepted      : %0d",       accepted);
            $display("  Send rejected      : %0d",       rejected);

            $display("\n[TB] ramulator_send_request    ... %s",
                     (accepted > 0)        ? "OK" : "FAIL");
            $display("[TB] ramulator_check_response  ... %s",
                     (total_completed > 0) ? "OK" : "WARN (no completions seen)");

            if (accepted > 0 && total_completed > 0)
                $display("\n=== Smoke test PASSED ===\n");
            else
                $display("\n=== Smoke test FAILED ===\n");
        end

        repeat(4) @(posedge clk);
        $finish;
    end

    // ================================================================
    // Hard timeout (3x MAX_CYCLES) so the sim never hangs
    // ================================================================
    initial begin
        #(MAX_CYCLES * CLK_PERIOD * 3);
        $display("[TB] TIMEOUT – simulation exceeded %0d ns.", MAX_CYCLES * 30);
        $finish;
    end

endmodule
