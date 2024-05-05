"include dcp.h"

module spmv_mem_intf #(
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
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data //up to 64Bytes

    // Output 
    
);

endmodule 