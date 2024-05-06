`include "dcp.h"
//work in progress
module spmv_channel #(
    .PARAM         (1) 
) (
    input  wire                              clk,
    input  wire                              rst_n,

    // Mem Req Signal Interface
    input  wire                              mem_req_rdy,
    output reg                               mem_req_val,
    output reg [5:0]                         mem_req_transid,
    output reg [`DCP_PADDR_MASK]             mem_req_addr, 

    // Mem Input Signal Interface
    input  wire                              mem_resp_val,
    input  wire [5:0]                        mem_resp_transid, // up to 64 outstanding requests 
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data, //up to 64Bytes


    //inputs

    input wire[4][64]                        row_lengths_fetch_in,
    input wire[4][64]                        col_IDs__fetch_in,
    input wire[4][64]                        matrix_values_fetch_in,
    input wire[4][64]                        row_IDs_decoder_in,
    input wire[4][64]                        vector_values_BVB_in,

    // Output 
    output wire[4][64]                        row_lengths_decoder_out,
    output wire[4][64]                        row_IDs_accumulator_out,
    output wire[4][64]                        mul_accumulator_out,
    output wire[4][64]                        col_IDs_BVB_out


);

always @(posedge clk ) begin
    mul_accumulator_out = vector_values_BVB_in * matrix_values_fetch_in;
end


endmodule 