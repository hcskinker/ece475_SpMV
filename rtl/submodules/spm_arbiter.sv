"include dcp.h"

module spm_arbiter #(
    parameter CHAN_NUM = 16, 
    parameter SPM_ELE_W = 32
) (
    input  wire                             clk,
    input  wire                             rst_n,
    
    // Fetching Inputs and SPM data
    input  wire                              spmv_init,
    input  wire                              spm_fetch,
    input  wire [39:0]                       spm_val_pntr,
    input  wire [39:0]                       spm_col_idx_pntr,
    input  wire [39:0]                       spm_row_len_pntr,
    input  wire [15:0]                       spm_nnz,
    input  wire [15:0]                       spm_nzzr,

    // Mem Req Signal Interface
    input  wire                              mem_req_rdy,
    output reg                               mem_req_val,
    output reg [5:0]                         mem_req_transid,
    output reg [`DCP_PADDR_MASK]             mem_req_addr, 

    // Mem Input Signal Interface
    input  wire                              mem_resp_val,
    input  wire [5:0]                        mem_resp_transid, // up to 64 outstanding requests 
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data, //up to 64Bytes

    // Output to the Channels
    output reg  [SPM_ELE_W-1:0]              spm_val        [CHAN_NUM-1:0],
    output reg  [SPM_ELE_W-1:0]              spm_row_len    [CHAN_NUM-1:0],
    output reg  [SPM_ELE_W-1:0]              spm_col_idx    [CHAN_NUM-1:0],
    output reg  [CHAN_NUM-1:0]               spm_chan_bubble // Bubble signal if stall
);  

// Grabbing Row Lengths (CHAN_NUM slots per channel issue cycle)
// 


reg [SPM_ELE_W-1:0] offset_queue [CHAN_NUM-1:0]; // Holds the Offset Values

mem_req_hsk = mem_req_rdy && mem_req_val;
assign mem_req_val = spm_fetch;

wire next_pulse_start = !pulse_start || (pulse_start && fetch_stall);
reg pulse_start;

// Align to Cache
wire [`DCP_PADDR_MASK] first_val_addr = spm_val_pntr & ((~40'd0) << 6);
wire [`DCP_PADDR_MASK] first_col_addr = spm_col_idx_pntr & ((~40'd0) << 6);
wire [`DCP_PADDR_MASK] first_len_addr = spm_row_len_pntr & ((~40'd0) << 6);
wire [3:0] offset = vec_ptr[5:2]; 

always_ff @(posedge clk) begin
    if(!rst_n) pulse_start <=0;
    else pulse_start <= next_pulse_start;
end

reg offset_stall; // Not equivalent (Call it something else (double request))
reg fetch_stall;

///////////////////////////////////////////////////////////////////////////////
// Memory Requests
///////////////////////////////////////////////////////////////////////////////

reg [15:0] iteration;

always_ff @(posedge clk) begin
    if(!rst_n || spmv_init) begin 
        iteration <= 0;
        mem_req_transid <= 0; 
    end
    else if (spm_fetch) begin
        // Change transaction ID for each type of request
        if (fetch_stall!) begin
            if (mem_req_transid == 2'd3) begin
                mem_req_transid <= 0;
                finish_req <= 1;
            end
            mem_req_transid <= mem_req_transid + 1;
        end
    end
end

///////////////////////////////////////////////////////////////////////////////
// Memory Responses 
///////////////////////////////////////////////////////////////////////////////

endmodule 