// ramulator_sv_wrapper.sv
//
// SystemVerilog DPI-C wrapper around the Ramulator C wrapper.
//
// C API exposed (ramulator_dpi.h):
//   ramulator_handle_t  ramulator_init(const char* config_file)
//   int                 ramulator_send_request(handle, addr, req_type, source_id, data)
//   void                ramulator_tick(handle)
//   long long           ramulator_check_response(handle, uint64_t* data_out)  // -1 = none
//   void                ramulator_finalize(handle)
//
// Interface
// ---------
// Request (valid/ready handshake):
//   req_valid, req_addr[63:0], req_type (0=Rd,1=Wr), req_source_id[31:0], req_data[63:0]
//   req_ready is asserted in the cycle AFTER the request is sent (registered output).
//
// Response (one per cycle):
//   resp_valid, resp_addr[63:0], resp_data[63:0]
//   resp_data holds the functional model value for the completed read address.

`timescale 1ns / 1ps

module ramulator_sv_wrapper #(
    parameter string CONFIG_FILE    = "ramulator_config.yaml",
    parameter int    MAX_RESP_DRAIN = 1
)(
    input  logic        clk,
    input  logic        rst_n,

    // ----------------------------------------------------------------
    // Request port — valid/ready handshake
    // ----------------------------------------------------------------
    input  logic        req_valid,
    input  logic [63:0] req_addr,
    input  logic        req_type,       // 0 = Read, 1 = Write
    input  logic [31:0] req_source_id,
    input  logic [63:0] req_data,       // write data; ignored for reads
    output logic        req_ready,

    // ----------------------------------------------------------------
    // Response port
    // ----------------------------------------------------------------
    output logic        resp_valid,
    output logic [63:0] resp_addr,
    output logic [63:0] resp_data,      // functional model value for completed read

    // ----------------------------------------------------------------
    // Status
    // ----------------------------------------------------------------
    output logic        init_done
);

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

    // ----------------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------------
    initial begin
        handle    = null;
        init_done = 1'b0;

        wait (rst_n === 1'b0);
        wait (rst_n === 1'b1);
        @(posedge clk);

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
    // Ordering:
    //   1. check_response — drain one completion from last tick
    //   2. send_request   — inject new request if presented
    //   3. tick           — advance Ramulator by one cycle
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_ready  <= 1'b0;
            resp_valid <= 1'b0;
            resp_addr  <= '0;
            resp_data  <= '0;

        end else if (init_done) begin

            // 1. Drain one response
            dpi_resp = ramulator_check_response(handle, dpi_data_out);
            if (dpi_resp !== -64'sd1) begin
                resp_valid <= 1'b1;
                resp_addr  <= dpi_resp[63:0];
                resp_data  <= dpi_data_out[63:0];
            end else begin
                resp_valid <= 1'b0;
                resp_addr  <= '0;
                resp_data  <= '0;
            end

            // 2. Issue request if presented
            req_ready <= 1'b0;
            if (req_valid) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    longint'(req_addr),
                    int'({31'b0, req_type}),
                    int'(req_source_id),
                    longint'(req_data)
                );
                req_ready <= (dpi_accepted == 1);
            end

            // 3. Advance Ramulator
            ramulator_tick(handle);

        end
    end

endmodule
