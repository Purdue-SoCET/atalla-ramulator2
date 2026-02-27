// ramulator_sv_wrapper.sv
//
// SystemVerilog DPI-C wrapper around the Ramulator C wrapper.
//
// C API exposed (ramulator_dpi.h):
//   ramulator_handle_t  ramulator_init(const char* config_file)
//   int                 ramulator_send_request(handle, addr, req_type, source_id)
//   void                ramulator_tick(handle)
//   long long           ramulator_check_response(handle)   // -1 = none
//   void                ramulator_finalize(handle)
//
// Interface
// ---------
// Request (valid/ready handshake):
//   req_valid, req_addr[63:0], req_type (0=Rd,1=Wr), req_source_id[31:0]
//   req_ready is asserted in the same cycle the request is accepted.
//
// Response (one per cycle; note: the Ramulator queue may accumulate more):
//   resp_valid, resp_addr[63:0]
//
// NOTE: Ramulator may complete multiple requests per cycle internally.
//       This wrapper drains only one response per clock edge.  If your
//       workload can saturate the queue, instantiate a response FIFO on
//       the resp_* interface or increase draining (see MAX_RESP_DRAIN).

`timescale 1ns / 1ps

module ramulator_sv_wrapper #(
    parameter string CONFIG_FILE     = "ramulator_config.yaml",
    // Maximum responses drained from C queue per clock edge.
    // Excess responses are held in Ramulator's internal queue and
    // delivered on subsequent cycles.
    parameter int    MAX_RESP_DRAIN  = 1
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
    output logic        req_ready,      // high when request is accepted

    // ----------------------------------------------------------------
    // Response port — one response per cycle (see NOTE above)
    // ----------------------------------------------------------------
    output logic        resp_valid,
    output logic [63:0] resp_addr,

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
        input chandle          handle,
        input longint unsigned addr,
        input int              req_type,
        input int              source_id
    );

    import "DPI-C" function void ramulator_tick(
        input chandle handle
    );

    import "DPI-C" function longint ramulator_check_response(
        input chandle handle
    );

    import "DPI-C" function void ramulator_finalize(
        input chandle handle
    );

    // ----------------------------------------------------------------
    // Internal state
    // ----------------------------------------------------------------
    chandle handle;

    // Scratch variables for DPI return values (module-scope so they
    // are visible across the always block without needing 'automatic').
    int    dpi_accepted;
    longint dpi_resp;

    // ----------------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------------
    initial begin
        handle    = null;
        init_done = 1'b0;

        // Wait for reset assertion then de-assertion, then one rising edge.
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
    // Ordering mirrors the C smoke test:
    //   1. check_response — drain completions from last tick
    //   2. send_request   — inject new request if presented
    //   3. tick           — advance Ramulator by one cycle
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_ready  <= 1'b0;
            resp_valid <= 1'b0;
            resp_addr  <= '0;

        end else if (init_done) begin

            // ----------------------------------------------------------
            // 1. Drain one response from Ramulator's completion queue.
            //    resp_valid is asserted for exactly one cycle when a
            //    response is available.
            // ----------------------------------------------------------
            dpi_resp = ramulator_check_response(handle);
            if (dpi_resp !== -64'sd1) begin
                resp_valid <= 1'b1;
                resp_addr  <= dpi_resp[63:0];
            end else begin
                resp_valid <= 1'b0;
                resp_addr  <= '0;
            end

            // ----------------------------------------------------------
            // 2. Issue a new request when the upstream presents one.
            //    req_ready pulses high for one cycle when accepted.
            // ----------------------------------------------------------
            req_ready <= 1'b0;
            if (req_valid) begin
                dpi_accepted = ramulator_send_request(
                    handle,
                    req_addr,                      // 64-bit address (logic[63:0] → longint unsigned)
                    int'({31'b0, req_type}),        // 0=Rd 1=Wr
                    int'(req_source_id)
                );
                req_ready <= (dpi_accepted == 1);
            end

            // ----------------------------------------------------------
            // 3. Advance Ramulator by one clock cycle.
            // ----------------------------------------------------------
            ramulator_tick(handle);

        end
    end

endmodule
