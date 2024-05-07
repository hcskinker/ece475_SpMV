"include dcp.h"

module vec_file #(
    parameter DATA_W      = 32,
    parameter NUM_CH   = 16
) (
    input  wire                              clk,
    input  wire                              rst_n,

    // Prefetch Inputs 
    input wire                               spmv_init,
    input wire                               prefetch,
    input wire [`DCP_PADDR_MASK]             vec_pntr,
    input wire [`DIM_W-1:0]                  vec_len,

    // Channel Inputs
    input wire [`DIM_W-1:0]                  col_idx_in [NUM_CH-1:0], 

    // Mem Req Signal Interface
    input  wire                              mem_req_rdy,
    output reg                               mem_req_val,
    output reg [5:0]                         mem_req_transid,
    output reg [`DCP_PADDR_MASK]             mem_req_addr, 

    // Mem Input Signal Interface
    input  wire                              mem_resp_val,
    input  wire [5:0]                        mem_resp_transid, // up to 64 outstanding requests 
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data, //up to 64Bytes

    // State Output 
    output reg                               prefetch_done,

    // Channel Outputs
    output reg [DATA_W-1:0]                   col_val_out [NUM_CH-1:0]    

);

localparam VAL_PER_LINE = `DCP_NOC_RES_DATA_SIZE / DATA_W; // Number of Values per line
localparam VAL_ALIGN = $clog2(VAL_PER_LINE);

// Vector for all Registers
reg [DATA_W-1:0] vec_row [1023:0]; 

////////////////////////////////////////////////////////////////////////////////////////
// Return Vector Components to Channel
////////////////////////////////////////////////////////////////////////////////////////

genvar k;
generate
    for (k = 0; k < NUM_CH; k = k + 1) begin: channel_output_line
        col_val_out[k] = vec_row[col_idx_in[k]];
    end
endgenerate 


////////////////////////////////////////////////////////////////////////////////////////
// Prefetch Request Logic
////////////////////////////////////////////////////////////////////////////////////////

wire mem_req_hsk = mem_req_rdy && mem_req_val;
assign mem_req_val = prefetch && !prefetch_req_done; 
wire is_first_line = mem_req_hsk && !(|trans_id);

// Other Line Addresses (offset from cache aligned first line)
wire [`DCP_PADDR_MASK] line_addr = first_line_addr + (mem_req_trans_id << VAL_ALIGN); // Send an address that is the trans_id offset (multiplexed)

// First Line Address (align to cache line)
wire [`DCP_PADDR_MASK] first_line_addr = vec_ptr & ((~40'd0) << 6);
wire [3:0] line_off = vec_ptr[5:2];

// Request Address
wire mem_req_addr = is_first_line ? first_line_addr : line_addr;

wire prefetch_req_done = (mem_req_transid == (vec_len-1));

// Flip Flop to update the transaction ID
always_ff @(posedge clk) begin
    if (!rst_n || spmv_init) begin
        mem_req_transid <= 0;
    end
    else if (mem_req_hsk) begin 
        if (prefetch_req_done) mem_req_trans_id <= 0; // Reset and 
        else begin 
            // Transaction ID Corresponds Directly to Ordering of Vector
            mem_req_transid <= mem_req_transid + 1;
        end
    end
end

////////////////////////////////////////////////////////////////////////////////
// Prefetch Response Logic 
////////////////////////////////////////////////////////////////////////////////
wire [3:0] const_vpl = VAL_PER_LINE;
wire [3:0] first_line_length = const_vpl - line_off;

wire [DATA_W-1:0] mem_data [VAL_PER_LINE-1:0];

reg [15:0] len_cnt;
wire vector_filled = (len_cnt >= vec_len);

genvar i;
generate
    for (i = 0; i < VAL_PER_LINE; i = i + 1) begin: cache_split
        mem_data[i] = mem_resp_data[(i+1)*DATA_W-1:i*DATA_W];
    end
endgenerate

integer j;
always_ff @(posedge clk) begin
    if (!rst_n || spmv_init) begin
        vec_row <= '{default: '0};
        len_cnt <= 0;
    end
    else if (prefetch && mem_resp_val) begin
        if (mem_resp_transid == 0) begin
            for (i=0; i < first_line_length; i = i + 1) begin
                vec_row[i] <= mem_data[line_off+i];
            end
            len_cnt = len_cnt + first_line_length;
        end
        else begin
            for (i=0; i < VAL_PER_LINE; i = i + 1) begin
                vec_row[((mem_req_trans_id-1)<<VAL_ALIGN) + i + line_off] <= mem_data[i];
            end
            len_cnt = len_cnt + VAL_PER_LINE;
        end
    end
end



endmodule 