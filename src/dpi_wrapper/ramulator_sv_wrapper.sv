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
// Simplifications vs full AXI4:
//   - One Ramulator request per AXI transaction (burst length ignored,
//     always returns a single R beat with last=1).
//   - AW and W must arrive in the same cycle for a write to be issued.
//   - At most one pending R response at a time (Ramulator drains one/cycle).

`timescale 1ns / 1ps

module ramulator_sv_wrapper #(
    parameter string CONFIG_FILE = "ramulator_config.yaml"
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
    longint dpi_resp;
    longint dpi_data_out;

    // Pending read response (at most one outstanding at a time)
    logic           r_pending;
    sub_r_channel_t r_reg;

    // Pending write response
    logic           b_pending;
    sub_b_channel_t b_reg;

    // Map read address -> mid_id so the R response carries the right ID
    logic [MID_ARID-1:0] read_mid_id [longint];

    // ----------------------------------------------------------------
    // r_i_valid / b_i_valid driven combinatorially from state
    // ----------------------------------------------------------------
    assign axi.r_i_valid = r_pending;
    assign axi.r_i       = r_reg;
    assign axi.b_i_valid = b_pending;
    assign axi.b_i       = b_reg;

    // ----------------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------------
    initial begin
        handle    = null;
        init_done = 1'b0;
        r_pending = 1'b0;
        b_pending = 1'b0;

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
    //   1. Drain one read completion from Ramulator → R response register
    //   2. Accept AR channel → issue Ramulator read
    //   3. Accept B response when r_i_ready
    //   4. Accept AW+W channels → issue Ramulator write → B response
    //   5. Advance Ramulator clock
    // ----------------------------------------------------------------
    always @(posedge axi.CLK or negedge axi.nRST) begin
        if (!axi.nRST) begin
            axi.ar_o_ready <= 1'b0;
            axi.aw_o_ready <= 1'b0;
            axi.w_o_ready  <= 1'b0;
            r_pending      <= 1'b0;
            r_reg          <= '0;
            b_pending      <= 1'b0;
            b_reg          <= '0;

        end else if (init_done) begin

            // Default: ready pulses for one cycle only
            axi.ar_o_ready <= 1'b0;
            axi.aw_o_ready <= 1'b0;
            axi.w_o_ready  <= 1'b0;

            // ----------------------------------------------------------
            // 1. Drain one read completion
            //    Only call check_response when we have room to hold it:
            //    either no pending response, or the current one is being
            //    consumed this cycle (r_i_ready).
            // ----------------------------------------------------------
            if (!r_pending || axi.r_i_ready) begin
                dpi_resp = ramulator_check_response(handle, dpi_data_out);
                if (dpi_resp !== -64'sd1) begin
                    r_reg.data <= dpi_data_out[RDATA-1:0];
                    r_reg.id   <= read_mid_id[dpi_resp][RID-1:0];
                    r_reg.last <= 1'b1;
                    r_reg.resp <= R_OKAY;
                    r_pending  <= 1'b1;
                end else begin
                    // No new completion; clear pending if current was accepted
                    if (r_pending && axi.r_i_ready)
                        r_pending <= 1'b0;
                end
            end

            // ----------------------------------------------------------
            // 2. Accept AR → issue Ramulator read request
            // ----------------------------------------------------------
            if (axi.ar_o_valid) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    longint'(axi.ar_o.addr),
                    0,                          // Read
                    int'(axi.ar_o.mid_id),
                    0                           // data unused for reads
                );
                if (dpi_accepted) begin
                    read_mid_id[longint'(axi.ar_o.addr)] = axi.ar_o.mid_id;
                    axi.ar_o_ready <= 1'b1;
                end
            end

            // ----------------------------------------------------------
            // 3. Clear B response when accepted by response router
            // ----------------------------------------------------------
            if (b_pending && axi.b_i_ready)
                b_pending <= 1'b0;

            // ----------------------------------------------------------
            // 4. Accept AW+W → issue Ramulator write → B response
            //    Require both valid simultaneously; wait until B is free.
            // ----------------------------------------------------------
            if (axi.aw_o_valid && axi.w_o_valid && (!b_pending || axi.b_i_ready)) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    longint'(axi.aw_o.addr),
                    1,                          // Write
                    int'(axi.aw_o.mid_id),
                    longint'(axi.w_o.data)
                );
                if (dpi_accepted) begin
                    b_reg.id       <= axi.aw_o.mid_id[BID-1:0];
                    b_reg.resp     <= B_OKAY;
                    b_pending      <= 1'b1;
                    axi.aw_o_ready <= 1'b1;
                    axi.w_o_ready  <= 1'b1;
                end
            end

            // ----------------------------------------------------------
            // 5. Advance Ramulator by one cycle
            // ----------------------------------------------------------
            ramulator_tick(handle);

        end
    end

endmodule
