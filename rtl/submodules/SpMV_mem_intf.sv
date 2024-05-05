"include dcp.h"

module vec_file #(
    parameter VEC_W      = 32,
    parameter CHANNELS   = 16
) (
    input  wire                              clk,
    input  wire                              rst_n,

    // Prefetch Inputs 
    input wire                               prefetch,
    input wire [`DCP_PADDR_MASK]             vec_pntr,
    input wire [15:0]                        vec_len,

    // Channel Inputs
    input wire [15:0]                   col_idx_in [CHANNELS-1:0], 

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
    output reg [VEC_W-1:0]                   col_val_out [CHANNELS-1:0]    

);

localparam LINE_VALS = `DCP_NOC_RES_DATA_SIZE / VEC_W; // Number of Values per line

wire mem_req_hsk = mem_req_rdy && mem_req_val;
wire mem_req_val = prefetch && !prefetch_done; 
// wire mem_req_addr = vec_pntr + (mem_req_trans_id << LINE_VALS); // Send an address that is the trans_id offset (multiplexed)

wire [`DCP_PADDR_MASK] first_addr = 

// Decomposed into trans_id / cache_line
typedef struct {
    reg [5:0] trans_id;
    reg [`DCP_NOC_RES_DATA_SIZE-1:0] cache_line;
} cache_response;

cache_response response_tbl [63:0];

reg [VEC_W-1:0] vec_components [1023:0];  // Vector Register

wire prefetch_done = (mem_req_transid == (vec_len-1));

// Every clock cycle I want to send out a request for 
always_ff @(posedge clk) begin
    if (!rst_n) begin
        response_tbl <= '{default: '0};
        vec_components <= '{default: '0};
        mem_req_transid <= 0;
    end
    else begin
        if (mem_req_hsk) begin
            mem_req_transid <= mem_req_transid + 1;
            if (prefect_done) mem_req_trans_id <= 0;
        end
    end
end


endmodule 