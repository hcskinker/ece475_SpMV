//`include "dcp.h"
//needs revision
module spm_channel #( 
    parameter DATA_W = 32,
    parameter CHAN_NUM = 16

) (
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           spmv_init,
    input  wire                           spm_fetch_done,

    //inputs from decoder and BVB
    input wire [`DIM_W-1:0]               row_IDs_decoder_in,
    input wire [DATA_W-1:0]               vector_values_BVB_in,
    input wire                            decoder_pop_len,
    
    // inputs from arbiter
    input wire  [DATA_W-1:0]              spm_val,
    input wire  [DATA_W-1:0]              spm_row_len,
    input wire  [DATA_W-1:0]              spm_col_idx,
    input wire                            spm_fetch_stall, // Bubble signal if stall (don't push values)

    // Outputs 
    output wire [DATA_W-1:0]              decoder_row_lens_out,  
    output wire [`DIM_W-1:0]              row_IDs_accumulator_out,
    output reg  [DATA_W-1:0]              mul_accumulator_out,
    output reg  [`DIM_W-1:0]              col_IDs_BVB_out,

    output reg                            pipe_fetch_bubble               
);

wire reset = !rst_n || spmv_init;

// Row Length FIFO Buffer (Filled up continuously) Max Length = 1024/NUM_CHAN
localparam FIFO_LEN = `MAX_DIM_LEN/NUM_CH;
localparam ADDR_WIDTH = $clog2(FIFO_LEN);

row_fifo #(
  .DATA_WIDTH               (DATA_W),
  .FIFO_DEPTH               (FIFO_LEN),
  .ADDR_WIDTH               (ADDR_WIDTH)
) row_fifo (
  .clk                      (clk),
  .reset                    (reset),

  .wr_en                    (!spm_fetch_stall),
  .rd_en                    (decoder_pop_len),
  .data_in                  (spm_row_len), 
  .data_out                 (decoder_row_lens_out),
  .full                     (),
  .empty                    ()
);


reg [DATA_W-1:0]               mul_0;
reg [DATA_W-1:0]               mul_1 ;
reg [DATA_W-1:0]               mul_2;
reg [DATA_W-1:0]               mul_3;
reg [DATA_W-1:0]               bvb_in_buffer;
reg [DATA_W-1:0]               spm_val_buffer;

// Bubble Register (Nullify operation)
reg fetch_bubble, M0_bubble, M1_bubble, M2_bubble, M3_bubble;

// Finished Register
reg fetch_done, M0_done, M1_done, M2_done, M3_done;

// Fetch
always @(posedge clk ) begin 
    if (!rst_n || spmv_init) begin
        fetch_bubble <= 1;
        fetch_done <= 0;
    end
    else if (!spm_fetch_stall) begin
        spm_row_len;  // send to CISR Decoder
        col_IDs_BVB_out <= spm_col_idx; // send to BVB
        spm_val_buffer <= spm_val; // Matrix Values from arbiter
        fetch_bubble <= 0;
        fetch_done <= spm_fetch_done;
    end
    else begin 
        fetch_bubble <= 1;
        fetch_done <= spm_fetch_done;
    end
end

// mul 0
reg [`DIM_W-1:0] M0_row_idx;
always @(posedge clk ) begin 
    if (!rst_n || spmv_init) begin
        M0_bubble <= 1;
        M0_done <= 0;
    end
    else begin
        if (!fetch_bubble) begin
            mul_0 <= spm_val_buffer * vector_values_BVB_in; 
            M0_row_idx <= row_IDs_decoder_in; // Row ID for the CISR Decoder
        end// Vector Value retrieved from Col_ID
        M0_bubble <= fetch_bubble;
        M0_done <= fetch_done;
    end
end

// mul 1
reg [`DIM_W-1:0] M1_row_idx;
always @(posedge clk ) begin 
    if (!rst_n || spmv_init) begin
        M1_bubble <= 1;
        M1_done <= 0;
    end
    else begin 
        if (!M0_bubble) begin
            mul_1 <= mul_0;
            M1_row_idx <= M0_row_idx;
        end
        M1_bubble <= M0_bubble;
        M1_done <= M0_done;
    end
end
// mul 2
reg [`DIM_W-1:0] M2_row_idx;
always @(posedge clk ) begin 
    if (!rst_n || spmv_init) begin
        M2_bubble <= 1;
        M2_done <= 0;
    end
    else begin 
        if (!M1_bubble) begin
            mul_2 <= mul_1;
            M2_row_idx <= M1_row_idx;
        end
        M2_bubble <= M1_bubble;
        M2_done <= M1_done;
    end
end
// mul 3
reg [`DIM_W-1:0] M3_row_idx;
always @(posedge clk ) begin 
    if (!rst_n || spmv_init) begin
        M3_bubble <= 1;
        M3_done <= 0;
    end
    else begin 
        if (!M2_bubble) begin
            mul_3 <= mul_2;
            M3_row_idx <= M2_row_idx;
        end
        M3_bubble <= M2_bubble;
        M3_done <= M2_done;
    end
end

// Accumulator
wire [`DIM_W-1:0] next_row_idx = M3_row_idx;
reg [`DIM_W-1:0] acc_row_idx;
reg [DATA_W-1:0] mul_accumulator;
reg acc_new;
always @(posedge clk ) begin
    if (!rst_n || spmv_init) begin
        mul_accumulator <= 0;
        acc_row_idx <= 0;
        acc_new <= 0;
    end
    else begin
        acc_done <= 0;
        if (!M3_bubble) begin
            acc_row_idx <= next_row_idx;
            if (acc_row_idx != next_row_idx) begin // if new row index
                mul_accumulator <= mul3; // Start accumulator with multiplication result
                mul_accumulator_out <= mul_accumulator; // Send the old accumulation value for row
                acc_new <= 1;
            end 
            else begin
                mul_accumulator <= mul_accumulator + mul3; // Otherwise add the result in the pipeline to accumulator
            end
        end
    end
end

assign row_IDs_accumulator_out = acc_row_idx; // Send row_idx to output_vector, will store if acc_new 
 
endmodule 