`timescale 1ns / 1ps
`include "dcp.h"

module tb_vec_file;

    parameter VEC_W      = 32;
    parameter CHANNELS   = 16;
    parameter DCP_PADDR_MASK = 39:0;
    parameter DCP_NOC_RES_DATA_SIZE = 64;

    // Signals for the module interface
    reg clk;
    reg rst_n;
    reg spmv_init;
    reg prefetch;
    reg [DCP_PADDR_MASK] vec_pntr;
    reg [15:0] vec_len;
    reg [15:0] col_idx_in [CHANNELS-1:0];
    reg mem_req_rdy;
    wire mem_req_val;
    wire [5:0] mem_req_transid;
    wire [DCP_PADDR_MASK] mem_req_addr;
    reg mem_resp_val;
    reg [5:0] mem_resp_transid;
    reg [DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data;
    wire prefetch_done;
    wire [VEC_W-1:0] col_val_out [CHANNELS-1:0];

    // Instantiate the vec_file module
    vec_file #(
        .VEC_W(VEC_W),
        .CHANNELS(CHANNELS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .spmv_init(spmv_init),
        .prefetch(prefetch),
        .vec_pntr(vec_pntr),
        .vec_len(vec_len),
        .col_idx_in(col_idx_in),
        .mem_req_rdy(mem_req_rdy),
        .mem_req_val(mem_req_val),
        .mem_req_transid(mem_req_transid),
        .mem_req_addr(mem_req_addr),
        .mem_resp_val(mem_resp_val),
        .mem_resp_transid(mem_resp_transid),
        .mem_resp_data(mem_resp_data),
        .prefetch_done(prefetch_done),
        .col_val_out(col_val_out)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // Generate a clock with 100 MHz frequency
    end

    // Test sequence
    initial begin
        // Initialize inputs
        rst_n = 0;
        spmv_init = 0;
        prefetch = 0;
        vec_pntr = 0;
        vec_len = 16;
        mem_req_rdy = 0;
        mem_resp_val = 0;
        mem_resp_transid = 0;
        mem_resp_data = 0;

        @(posedge clk);
        #10 rst_n = 1; spmv_init = 1;
        @(posedge clk);
        #10 spmv_init = 0;
        
        // Set vector pointer and length
        vec_pntr = 'h1000; // Example memory address
        vec_len = 16; // Example vector length

        // Simulate starting the prefetch
        #10 prefetch = 1;
        #10 mem_req_rdy = 1; // Ready to accept memory requests

        // Simulate memory response
        #30 mem_resp_val = 1;
        mem_resp_transid = 0;
        mem_resp_data = {2{32'hA5A5A5A5}}; // Example data pattern

        // Continue providing data till all requests are serviced
        #10 mem_resp_transid = 1;
        #10 mem_resp_transid = 2;
        @(posedge clk);
        mem_resp_val = 0; // No more responses

        // End simulation
        #100 $finish;
    end

    // Display outputs for debugging
    initial begin
        $monitor("Time = %t, mem_req_val = %b, mem_req_addr = %h, prefetch_done = %b",
                 $time, mem_req_val, mem_req_addr, prefetch_done);
    end

endmodule
