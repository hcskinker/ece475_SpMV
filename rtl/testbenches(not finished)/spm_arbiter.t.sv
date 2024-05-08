`timescale 1ns / 1ps
`include "dcp.h"

///////////////////////////////////////////////////////////////////////////////////
// Incomplete Testbench 
//////////////////////////////////////////////////////////////////////////////////



module tb_spm_arbiter;

    parameter CHAN_NUM = 16;
    parameter SPM_ELE_W = 32;
    parameter DCP_PADDR_MASK = 39:0;
    parameter DCP_NOC_RES_DATA_SIZE = 64;

    // Signals for module interface
    reg clk;
    reg rst_n;
    reg spmv_init;
    reg spm_fetch;
    reg [39:0] spm_val_pntr;
    reg [39:0] spm_col_idx_pntr;
    reg [39:0] spm_row_len_pntr;
    reg [15:0] spm_nnz;
    reg [15:0] spm_nzzr;
    reg mem_req_rdy;
    wire mem_req_val;
    wire [5:0] mem_req_transid;
    wire [DCP_PADDR_MASK] mem_req_addr;
    reg mem_resp_val;
    reg [5:0] mem_resp_transid;
    reg [DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data;
    wire [SPM_ELE_W-1:0] spm_val[CHAN_NUM-1:0];
    wire [SPM_ELE_W-1:0] spm_row_len[CHAN_NUM-1:0];
    wire [SPM_ELE_W-1:0] spm_col_idx[CHAN_NUM-1:0];
    wire [CHAN_NUM-1:0] spm_chan_bubble;

    // Instantiate the spm_arbiter module
    spm_arbiter #(
        .CHAN_NUM(CHAN_NUM),
        .SPM_ELE_W(SPM_ELE_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .spmv_init(spmv_init),
        .spm_fetch(spm_fetch),
        .spm_val_pntr(spm_val_pntr),
        .spm_col_idx_pntr(spm_col_idx_pntr),
        .spm_row_len_pntr(spm_row_len_pntr),
        .spm_nnz(spm_nnz),
        .spm_nzzr(spm_nzzr),
        .mem_req_rdy(mem_req_rdy),
        .mem_req_val(mem_req_val),
        .mem_req_transid(mem_req_transid),
        .mem_req_addr(mem_req_addr),
        .mem_resp_val(mem_resp_val),
        .mem_resp_transid(mem_resp_transid),
        .mem_resp_data(mem_resp_data),
        .spm_val(spm_val),
        .spm_row_len(spm_row_len),
        .spm_col_idx(spm_col_idx),
        .spm_chan_bubble(spm_chan_bubble)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz Clock
    end

    // Test sequence
    initial begin
        // Initialize signals
        rst_n = 0;
        spmv_init = 0;
        spm_fetch = 0;
        spm_val_pntr = 0;
        spm_col_idx_pntr = 0;
        spm_row_len_pntr = 0;
        spm_nnz = 16; // example non-zero values
        spm_nzzr = 8; // example non-zero values per row
        mem_req_rdy = 0;
        mem_resp_val = 0;
        mem_resp_transid = 0;
        mem_resp_data = 0;

        @(posedge clk);
        #20 rst_n = 1;
        #10 spmv_init = 1;

        // Start SPM fetch operation
        #10;
        spm_fetch = 1;
        spm_val_pntr = 'h1000;
        spm_col_idx_pntr = 'h2000;
        spm_row_len_pntr = 'h3000;
        mem_req_rdy = 1;  // Ready to accept memory requests

        // Simulate memory response
        #30 mem_resp_val = 1;
        mem_resp_transid = 0;
        mem_resp_data = {2{32'hA5A5A5A5}}; // Example data pattern

        // Continue providing data
        #10 mem_resp_transid = 1;
        @(posedge clk);
        mem_resp_val = 0; // End of responses

        // End simulation
        #100 $finish;
    end

    // Display outputs for debugging
    initial begin
        $monitor("Time = %t, mem_req_val = %b, mem_req_addr = %h, spm_chan_bubble = %b",
                 $time, mem_req_val, mem_req_addr, spm_chan_bubble);
    end

endmodule
