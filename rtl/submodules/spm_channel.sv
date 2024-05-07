//`include "dcp.h"
//needs revision
module spm_channel #( 
    parameter SPM_ELE_W = 32,
    parameter CHAN_NUM = 16

) (
    input  wire                              clk,
    input  wire                              rst_n,

    //inputs from decoder and BVB
    input wire [SPM_ELE_W-1:0]               row_IDs_decoder_in     [CHAN_NUM-1:0],
    input wire [SPM_ELE_W-1:0]               vector_values_BVB_in   [CHAN_NUM-1:0],
    
    // inputs from arbiter
    input wire  [SPM_ELE_W-1:0]              spm_val                [CHAN_NUM-1:0],
    input wire  [SPM_ELE_W-1:0]              spm_row_len            [CHAN_NUM-1:0],
    input wire  [SPM_ELE_W-1:0]              spm_col_idx            [CHAN_NUM-1:0],
    input wire                               spm_fetch_stall, // Bubble signal if stall (don't push values)
    
    // Outputs 
    output reg [SPM_ELE_W-1:0]               row_lengths_decoder_out [CHAN_NUM-1:0],
    output reg [SPM_ELE_W-1:0]               row_IDs_accumulator_out [CHAN_NUM-1:0],
    output reg [SPM_ELE_W-1:0]               mul_accumulator_out     [CHAN_NUM-1:0],
    output reg [SPM_ELE_W-1:0]               col_IDs_BVB_out         [CHAN_NUM-1:0]
);

reg [SPM_ELE_W-1:0]               mul_0                 [CHAN_NUM-1:0];
reg [SPM_ELE_W-1:0]               mul_1                 [CHAN_NUM-1:0];
reg [SPM_ELE_W-1:0]               mul_2                 [CHAN_NUM-1:0];
reg [SPM_ELE_W-1:0]               mul_3                 [CHAN_NUM-1:0];
reg [SPM_ELE_W-1:0]               bvb_in_buffer         [CHAN_NUM-1:0];
reg [SPM_ELE_W-1:0]               spm_val_buffer        [CHAN_NUM-1:0];
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
        for (i=0; i< CHAN_NUM; i=i+1) begin
            mul_0 [i] = spm_val_buffer[i] * bvb_in_buffer[i];
        end
    end
end

// mul 1
always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
        for (i=0; i< CHAN_NUM; i=i+1) begin
            mul_1 [i] = mul_0[i];
        end
    end
end
// mul 2
always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
        for (i=0; i< CHAN_NUM; i=i+1) begin
            mul_2 [i] = mul_1[i];
        end
    end
end
// mul 3
always @(posedge clk ) begin 
    if (!spm_fetch_stall) begin
        for (i=0; i< CHAN_NUM; i=i+1) begin
            mul_3 [i] = mul_2[i];
        end
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