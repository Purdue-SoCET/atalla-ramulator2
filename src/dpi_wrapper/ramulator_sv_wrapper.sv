// ramulator_sv_wrapper.sv
//
// AXI4 subordinate wrapper around the Ramulator DPI-C library.
//
// Connects to the subordinate side of axi_bus_if:
//   AR channel : ar_o_valid / ar_o / ar_o_ready
//   R  channel : r_i_valid  / r_i  / r_i_ready
//   AW channel : aw_o_valid / aw_o / aw_o_ready
//   W  channel : w_o_valid  / w_o  / w_o_ready
//   B  channel : b_i_valid  / b_i  / b_i_ready
//
// Read path — two modes, selected by AR.len:
//
//   Single-beat (len == 0): original pipelined path.  Multiple ARs may be
//     outstanding in Ramulator simultaneously; responses are forwarded to the
//     R channel one at a time via r_pending.  check_response is gated by
//     r-slot availability so the completion queue drains naturally.
//
//   Multi-beat INCR burst (len > 0): ar_active serialises bursts (one at a
//     time).  All len+1 Ramulator sub-requests are issued as fast as
//     Ramulator accepts them (one per cycle).  Responses are stored in a
//     16-entry reorder buffer and driven on R in order with correct last flag.
//     check_response is called unconditionally while ar_active so backpressure
//     on r_i_ready does not stall the completion queue.
//
// Write path — supports INCR bursts (len > 0):
//   AW+W[0] must arrive in the same cycle to start a write burst.  Subsequent
//   W beats are accepted one per cycle as they arrive.  One B response is
//   issued after the last W beat.  B responses are buffered in a small FIFO
//   (depth B_DEPTH) to decouple write acceptance from the B-channel drain.
//
// Other simplifications:
//   - At most one outstanding AR burst (len > 0) at a time.
//   - At most one outstanding AW burst at a time.

`timescale 1ns / 1ps

module ramulator_sv_wrapper #(
    parameter string CONFIG_FILE = "ramulator_config.yaml",
    parameter int    B_DEPTH     = 4        // B-response FIFO depth
)(
    axi_bus_if  axi,
    output logic init_done
);

    import axi_bus_pkg::*;

    // ----------------------------------------------------------------
    // DPI-C imports
    // ----------------------------------------------------------------
    import "DPI-C" function chandle ramulator_init(
        input string config_file
    );

    import "DPI-C" function int ramulator_send_request(
        input chandle handle,
        input longint addr,
        input int     req_type,
        input int     source_id,
        input longint data
    );

    import "DPI-C" function void ramulator_tick(
        input chandle handle
    );

    import "DPI-C" function longint ramulator_check_response(
        input  chandle handle,
        output longint data_out
    );

    import "DPI-C" function void ramulator_finalize(
        input chandle handle
    );

    // ----------------------------------------------------------------
    // Internal state
    // ----------------------------------------------------------------
    chandle handle;
    int     dpi_accepted;
    int     beat_idx;       // blocking temp for ROB index calculation
    longint dpi_resp;
    longint dpi_data_out;

    // R channel register (one beat presented on the AXI R channel at a time)
    logic           r_pending;
    sub_r_channel_t r_reg;

    // B-response FIFO
    sub_b_channel_t b_fifo [B_DEPTH];
    int             b_cnt;
    int             b_rd;
    int             b_wr;
    int             b_cnt_next;

    // --- Single-beat read path ---
    // Maps Ramulator response address → master ID so that concurrent
    // single-beat reads can be matched to their original AR transaction.
    logic [MID_ARID-1:0] read_mid_id [longint];

    // --- Multi-beat read burst state (len > 0 only) ---
    logic                ar_active;       // burst in progress
    longint              ar_base_addr;
    logic [ARLEN-1:0]    ar_len_reg;      // burst length (beats - 1)
    logic [ARSIZE-1:0]   ar_size_reg;     // beat size (log2 bytes)
    logic [MID_ARID-1:0] ar_id_reg;       // ID for all beats in burst
    int                  ar_beat_issued;  // next beat index to issue to Ramulator
    int                  ar_beat_out;     // next beat index to output on R channel
    longint              ar_rob [16];     // reorder buffer (max 16 beats, ARLEN=4)
    logic [15:0]         ar_rob_valid;    // valid flags for ROB slots

    // --- Write burst state ---
    logic                aw_active;       // multi-beat W burst in progress
    longint              aw_base_addr;
    logic [AWLEN-1:0]    aw_len_reg;
    logic [AWSIZE-1:0]   aw_size_reg;
    logic [MID_AWID-1:0] aw_id_reg;
    int                  aw_beat;         // next W beat index (beat 0 already sent)

    // ----------------------------------------------------------------
    // Combinatorial outputs
    // ----------------------------------------------------------------
    assign axi.r_i_valid = r_pending;
    assign axi.r_i       = r_reg;
    assign axi.b_i_valid = (b_cnt > 0);
    assign axi.b_i       = b_fifo[b_rd];

    // ----------------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------------
    initial begin
        handle       = null;
        init_done    = 1'b0;
        r_pending    = 1'b0;
        b_cnt        = 0;
        b_rd         = 0;
        b_wr         = 0;
        ar_active    = 1'b0;
        ar_rob_valid = '0;
        aw_active    = 1'b0;

        wait (axi.nRST === 1'b0);
        wait (axi.nRST === 1'b1);
        @(posedge axi.CLK);

        handle = ramulator_init(CONFIG_FILE);
        if (handle == null)
            $fatal(1, "[ramulator_sv_wrapper] ramulator_init() returned null. Config: %s",
                   CONFIG_FILE);

        init_done = 1'b1;
        $display("[ramulator_sv_wrapper] Init OK (config=%s)", CONFIG_FILE);
    end

    // ----------------------------------------------------------------
    // Finalize on simulation exit
    // ----------------------------------------------------------------
    final begin
        if (handle != null) begin
            ramulator_finalize(handle);
            $display("[ramulator_sv_wrapper] Finalized.");
        end
    end

    // ----------------------------------------------------------------
    // Clocked logic
    //
    // Each cycle (in order):
    //   1.  check_response → either ROB (burst active) or r_reg (single-beat)
    //   1b. Drive R channel in burst order from ROB  [burst path only]
    //   2.  Issue next burst beat to Ramulator        [burst path only]
    //   3.  Accept AR:
    //         len == 0 → single-beat path (read_mid_id, no ar_active)
    //         len  > 0 → burst path (ar_active, ROB)
    //   4.  Drain B FIFO head (blocking b_cnt_next)
    //   5a. Accept W beat for ongoing write burst  OR
    //   5b. Accept AW+W[0] to start write burst
    //   6.  Commit b_cnt
    //   7.  Advance Ramulator clock
    // ----------------------------------------------------------------
    always @(posedge axi.CLK or negedge axi.nRST) begin
        if (!axi.nRST) begin
            axi.ar_o_ready <= 1'b0;
            axi.aw_o_ready <= 1'b0;
            axi.w_o_ready  <= 1'b0;
            r_pending      <= 1'b0;
            r_reg          <= '0;
            b_cnt          <= 0;
            b_rd           <= 0;
            b_wr           <= 0;
            ar_active      <= 1'b0;
            ar_rob_valid   <= '0;
            aw_active      <= 1'b0;

        end else if (init_done) begin

            // Defaults: ready pulses for one cycle only
            axi.ar_o_ready <= 1'b0;
            axi.aw_o_ready <= 1'b0;
            axi.w_o_ready  <= 1'b0;

            // ----------------------------------------------------------
            // 1. Drain one Ramulator response.
            //
            //    Burst path  (ar_active): called unconditionally so the
            //      completion queue keeps draining under R backpressure.
            //      Response is stored in the reorder buffer.
            //
            //    Single-beat path (!ar_active): gated on r-slot availability
            //      (same as original design).  Response is written directly
            //      to r_reg.
            // ----------------------------------------------------------
            if (ar_active || !r_pending || axi.r_i_ready) begin
                dpi_resp = ramulator_check_response(handle, dpi_data_out);
                 if (dpi_resp !== -64'sd1) begin
                    if (ar_active) begin
                        // Route to reorder buffer; step 1b outputs in order
                        beat_idx = int'((dpi_resp - ar_base_addr)
                                        >> int'(ar_size_reg));

                        if (beat_idx >= 0 && beat_idx < 16 && beat_idx <= int'(ar_len_reg)) begin
                            ar_rob[beat_idx]       <= dpi_data_out;
                            ar_rob_valid[beat_idx] <= 1'b1;
                        end else begin
                            $fatal(1,
                                "[ramulator_sv_wrapper] Bad burst response: resp_addr=%0h base=%0h beat_idx=%0d len=%0d size=%0d",
                                dpi_resp, ar_base_addr, beat_idx, ar_len_reg, ar_size_reg);
                        end
                    end else begin
                        // Single-beat: forward directly to R channel register
                        r_reg.data <= dpi_data_out[RDATA-1:0];
                        r_reg.id   <= read_mid_id[dpi_resp][RID-1:0];
                        r_reg.last <= 1'b1;
                        r_reg.resp <= R_OKAY;
                        r_pending  <= 1'b1;
                    end
                end else begin
                    // No response this cycle; clear r_pending if consumed
                    if (!ar_active && r_pending && axi.r_i_ready)
                        r_pending <= 1'b0;
                end
            end

            // ----------------------------------------------------------
            // 1b. Drive R channel in burst order from reorder buffer.
            //     Load next beat when current one is accepted or slot empty.
            //     (ar_rob_valid reflects the previous cycle due to NB
            //     semantics, so response arrival has ≤1-cycle output latency.)
            // ----------------------------------------------------------
            if (ar_active) begin
                if (!r_pending || axi.r_i_ready) begin
                    if (ar_rob_valid[ar_beat_out]) begin
                        r_reg.data  <= ar_rob[ar_beat_out][RDATA-1:0];
                        r_reg.id    <= ar_id_reg[RID-1:0];
                        r_reg.last  <= (ar_beat_out == int'(ar_len_reg));
                        r_reg.resp  <= R_OKAY;
                        r_pending   <= 1'b1;
                        ar_rob_valid[ar_beat_out] <= 1'b0;
                        if (ar_beat_out == int'(ar_len_reg))
                            ar_active <= 1'b0;
                        ar_beat_out <= ar_beat_out + 1;
                    end else begin
                        if (r_pending && axi.r_i_ready)
                            r_pending <= 1'b0;
                    end
                end
            end

            // ----------------------------------------------------------
            // 2. Issue remaining beats of an active read burst to Ramulator.
            //    One attempt per cycle; retry next cycle if Ramulator stalls.
            // ----------------------------------------------------------
            if (ar_active && ar_beat_issued <= int'(ar_len_reg)) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    ar_base_addr + longint'(ar_beat_issued)
                                 * longint'(1 << int'(ar_size_reg)),
                    0,                      // Read
                    int'(ar_id_reg),
                    0
                );
                if (dpi_accepted)
                    ar_beat_issued <= ar_beat_issued + 1;
            end

            // ----------------------------------------------------------
            // 3. Accept AR.
            //    Gated on !ar_active for both paths (only one burst at a
            //    time; single-beat ARs are unaffected when ar_active=0).
            // ----------------------------------------------------------
            if (axi.ar_o_valid && !ar_active) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    longint'(axi.ar_o.addr),
                    0,                      // Read
                    int'(axi.ar_o.mid_id),
                    0
                );
                if (dpi_accepted) begin
                    axi.ar_o_ready <= 1'b1;
                    if (axi.ar_o.len == '0) begin
                        // Single-beat: record ID in map; no ar_active
                        read_mid_id[longint'(axi.ar_o.addr)] = axi.ar_o.mid_id;
                    end else begin
                        // Multi-beat burst: activate burst machinery
                        ar_base_addr   <= longint'(axi.ar_o.addr);
                        ar_len_reg     <= axi.ar_o.len;
                        ar_size_reg    <= axi.ar_o.size;
                        ar_id_reg      <= axi.ar_o.mid_id;
                        ar_beat_issued <= 1;
                        ar_beat_out    <= 0;
                        ar_rob_valid   <= '0;
                        ar_active      <= 1'b1;
                    end
                end
            end

            // ----------------------------------------------------------
            // 4. Drain head of B FIFO if accepted by response router.
            //    Uses blocking b_cnt_next so step 5 sees the updated count.
            // ----------------------------------------------------------
            b_cnt_next = b_cnt;

            if (b_cnt_next > 0 && axi.b_i_ready) begin
                b_rd       <= (b_rd + 1 == B_DEPTH) ? 0 : b_rd + 1;
                b_cnt_next = b_cnt_next - 1;
            end

            // ----------------------------------------------------------
            // 5a. Accept W beat for an ongoing write burst.
            // ----------------------------------------------------------
            if (aw_active && axi.w_o_valid) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    aw_base_addr + longint'(aw_beat)
                                 * longint'(1 << int'(aw_size_reg)),
                    1,                      // Write
                    int'(aw_id_reg),
                    longint'(axi.w_o.data)
                );
                if (dpi_accepted) begin
                    axi.w_o_ready <= 1'b1;
                    aw_beat       <= aw_beat + 1;
                    if (axi.w_o.last) begin
                        b_fifo[b_wr] <= '{id: aw_id_reg[BID-1:0], resp: B_OKAY};
                        b_wr         <= (b_wr + 1 == B_DEPTH) ? 0 : b_wr + 1;
                        b_cnt_next   = b_cnt_next + 1;
                        aw_active    <= 1'b0;
                    end
                end

            // ----------------------------------------------------------
            // 5b. Accept AW+W[0] → issue beat 0 and start write burst.
            //     Requires B FIFO space and no burst already in progress.
            // ----------------------------------------------------------
            end else if (!aw_active && axi.aw_o_valid && axi.w_o_valid
                         && b_cnt_next < B_DEPTH) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    longint'(axi.aw_o.addr),
                    1,                      // Write
                    int'(axi.aw_o.mid_id),
                    longint'(axi.w_o.data)
                );
                if (dpi_accepted) begin
                    axi.aw_o_ready <= 1'b1;
                    axi.w_o_ready  <= 1'b1;
                    if (axi.w_o.last) begin
                        // Single-beat or last beat arrives with AW
                        b_fifo[b_wr] <= '{id: axi.aw_o.mid_id[BID-1:0], resp: B_OKAY};
                        b_wr         <= (b_wr + 1 == B_DEPTH) ? 0 : b_wr + 1;
                        b_cnt_next   = b_cnt_next + 1;
                    end else begin
                        // Multi-beat: defer B push until w_last
                        aw_base_addr <= longint'(axi.aw_o.addr);
                        aw_len_reg   <= axi.aw_o.len;
                        aw_size_reg  <= axi.aw_o.size;
                        aw_id_reg    <= axi.aw_o.mid_id;
                        aw_beat      <= 1;
                        aw_active    <= 1'b1;
                    end
                end
            end

            // ----------------------------------------------------------
            // 6. Commit final B FIFO count
            // ----------------------------------------------------------
            b_cnt <= b_cnt_next;

            // ----------------------------------------------------------
            // 7. Advance Ramulator by one cycle
            // ----------------------------------------------------------
            ramulator_tick(handle);

        end
    end

endmodule
