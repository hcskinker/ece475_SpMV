//`include "dcp.h"
//needs revision
module spm_channel #( 
    parameter DATA_W = 32,
    parameter CHAN_NUM = 16

) (
    input  wire                              clk,
    input  wire                              rst_n,

    //inputs from decoder and BVB
    input wire [DATA_W-1:0]               row_IDs_decoder_in,
    input wire [DATA_W-1:0]               vector_values_BVB_in,
    
    // inputs from arbiter
    input wire  [DATA_W-1:0]              spm_val,
    input wire  [DATA_W-1:0]              spm_row_len,
    input wire  [DATA_W-1:0]              spm_col_idx,
    input wire                               spm_fetch_stall, // Bubble signal if stall (don't push values)
    
    // Outputs 
    output reg [DATA_W-1:0]               row_lengths_decoder_out,
    output reg [DATA_W-1:0]               row_IDs_accumulator_out,
    output reg [DATA_W-1:0]               mul_accumulator_out,
    output reg [DATA_W-1:0]               col_IDs_BVB_out
);

reg [DATA_W-1:0]               mul_0;
reg [DATA_W-1:0]               mul_1 ;
reg [DATA_W-1:0]               mul_2;
reg [DATA_W-1:0]               mul_3;
reg [DATA_W-1:0]               bvb_in_buffer;
reg [DATA_W-1:0]               spm_val_buffer;
// fetch

always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
    row_lengths_decoder_out = spm_row_len;  // send to CISR Decoder
  	col_IDs_BVB_out = spm_col_idx; // send to BVB
    bvb_in_buffer = vector_values_BVB_in; //vector from BVB
    spm_val_buffer = spm_val; // Matrix Values from arbiter
    end
end

// mul 0
integer i;
always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
        mul_0 = spm_val_buffer * bvb_in_buffer;
    end
end

// mul 1
always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
        mul_0 = spm_val_buffer * bvb_in_buffer;
    end
end
// mul 2
always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
        mul_0 = spm_val_buffer * bvb_in_buffer;
    end
end
// mul 3
always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
        mul_0 = spm_val_buffer * bvb_in_buffer;
    end
end
// To Accumulator
always @(posedge clk ) begin
    if (!spm_fetch_stall) begin
        mul_accumulator_out = mul_3;
        row_IDs_accumulator_out = row_IDs_decoder_in;
    end
end
 
endmodule 