/*  Aryan Kadakia - kadakia0@purdue.edu */

`ifndef AXI_BUS_IF_SV
`define AXI_BUS_IF_SV

interface axi_bus_if(input logic CLK, input logic nRST);
    `include "axi_bus_params.svh"

    import axi_bus_pkg::*;

    // master side channel structs
    master_ar_channel_t ar_sp0_i;
    master_ar_channel_t ar_sp1_i;
    master_ar_channel_t ar_d_i;
    master_ar_channel_t ar_i_i;
    master_r_channel_t  r_sp0_o;
    master_r_channel_t  r_sp1_o;
    master_r_channel_t  r_d_o;
    master_r_channel_t  r_i_o;
    master_aw_channel_t aw_sp0_i;
    master_aw_channel_t aw_sp1_i;
    master_aw_channel_t aw_d_i;
    master_w_channel_t  w_sp0_i;
    master_w_channel_t  w_sp1_i;
    master_w_channel_t  w_d_i;
    master_b_channel_t  b_sp0_o;
    master_b_channel_t  b_sp1_o;
    master_b_channel_t  b_d_o;

    // subordinate side channel structs
    sub_ar_channel_t ar_o;
    sub_r_channel_t  r_i;
    sub_aw_channel_t aw_o;
    sub_w_channel_t  w_o;
    sub_b_channel_t  b_i;

    // read arbiter signals
    logic sp0_req_r, sp1_req_r, d_req_r, i_req_r, skid_ready_r;
    logic [ARGRANT-1:0] ar_grant;

    // write arbiter signals
    logic sp0_req_w, sp1_req_w, d_req_w, skid_ready_w;
    logic [AWLEN-1:0] sp0_len_w, sp1_len_w, d_len_w;
    logic [AWGRANT-1:0] aw_grant;

    logic ar_sp0_i_valid, ar_sp0_i_ready;
    logic ar_sp1_i_valid, ar_sp1_i_ready;
    logic ar_d_i_valid,   ar_d_i_ready;
    logic ar_i_i_valid,   ar_i_i_ready;
    logic ar_o_valid,     ar_o_ready;
    logic r_i_valid,      r_i_ready;
    logic r_sp0_o_valid,  r_sp0_o_ready;
    logic r_sp1_o_valid,  r_sp1_o_ready;
    logic r_d_o_valid,    r_d_o_ready;
    logic r_i_o_valid,    r_i_o_ready;
    logic aw_sp0_i_valid, aw_sp0_i_ready, w_sp0_i_valid, w_sp0_i_ready;
    logic aw_sp1_i_valid, aw_sp1_i_ready, w_sp1_i_valid, w_sp1_i_ready;
    logic aw_d_i_valid,   aw_d_i_ready,   w_d_i_valid,   w_d_i_ready;
    logic aw_o_valid,     aw_o_ready,     w_o_valid,     w_o_ready;
    logic b_i_valid,      b_i_ready;
    logic b_sp0_o_valid,  b_sp0_o_ready;
    logic b_sp1_o_valid,  b_sp1_o_ready;
    logic b_d_o_valid,    b_d_o_ready;

    modport ar_to_subordinate   (output ar_o_valid, ar_o,   input  ar_o_ready);
    modport subordinate_to_r    (output r_i_ready,          input  r_i_valid, r_i);
    modport aw_to_subordinate   (output aw_o_valid, aw_o,  input  aw_o_ready);
    modport w_to_subordinate    (output w_o_valid,  w_o,   input  w_o_ready);
    modport subordinate_to_b    (output b_i_ready,          input  b_i_valid, b_i);

    modport read_arbiter  (input  sp0_req_r, sp1_req_r, d_req_r, i_req_r, skid_ready_r,
                           output ar_grant);
    modport write_arbiter (input  sp0_req_w, sp1_req_w, d_req_w,
                           input  sp0_len_w, sp1_len_w, d_len_w, skid_ready_w,
                           output aw_grant);
    modport write_arbiter_tb (output sp0_req_w, sp1_req_w, d_req_w,
                              output sp0_len_w, sp1_len_w, d_len_w, skid_ready_w,
                              input  aw_grant);

endinterface
`endif // AXI_BUS_IF_SV
