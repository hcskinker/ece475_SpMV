/*
Copyright (c) 2020 Princeton University
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

`include "dcp.h"

`ifdef DEFAULT_NETTYPE_NONE
`default_nettype none
`endif

module tight_acc_iface #(
    parameter CHANNELS = 16
)  (
    input  wire clk,
    input  wire rst_n,
    // Command iface to receive "instructions" and configurations
    input  wire                             cmd_val,        // New valid command
    output wire                             busy,           // effectively behaves as cmd_rdy
    input  wire [5:0]                       cmd_opcode,     // Command operation code, 64 values
    input  wire [63:0]                      cmd_config_data, // Payload of command if needed

    // Interface to respond to the core after the accelerator has processed data
    output wire                             resp_val,
    input  wire                             resp_rdy, //whether the core is ready to take the data
    output wire [63:0]                      resp_data,

    // Request iface to memory hierarchy
    input  wire                             mem_req_rdy, //whether the network is ready to take the request
    output wire                             mem_req_val,
    output wire [5:0]                       mem_req_transid, //can have up to 64 inflight requests
    output wire [`DCP_PADDR_MASK       ]    mem_req_addr, // physical memory addr

    // Response iface from memory hierarchy (L2 shared cache)
    input  wire                              mem_resp_val,
    input  wire [5:0]                        mem_resp_transid, // up to 64 outstanding requests 
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data //up to 64Bytes
);

// FILL ME
assign busy = 1'b0;
assign mem_req_val = 1'b0;
assign mem_req_transid = 6'b0;
assign mem_req_addr = 40'd0;
// FOO implementation, respond untouched every command
assign resp_val = cmd_val;
assign resp_data = cmd_config_data; 

// Command Manager
typedef enum reg[1:0] {
    INIT_SPMV,          // Send the SPM Matrix pointer 
    LD_SPM_DATA,            // Send the info regarding length
    LD_VEC_DATA          // Load the vecto          
} spmv_cmd;

spmv_cmd cmd;
wire cmd_hsk = !busy & cmd_val;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command Parsing
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

assign cmd = cmd_opcode[1:0];

wire [`DCP_PADDR_MASK       ] cmd_spm_base_pntr = cmd_config_data[`DCP_PADDR_MASK       ]; //The physical address is 40 bits (configurable) 1TB
wire [`DCP_PADDR_MASK       ] cmd_vec_pntr = cmd_config_data[`DCP_PADDR_MASK       ]; //The physical address is 40 bits (configurable) 1TB
wire [15:0] cmd_vec_len = cmd_config_data[46:41]; // Vector length when sending over the pointer (16 bits)
wire [15:0] cmd_spm_nnz = cmd_config_data[15:0];  // Non Zero Elements (16-bits)
wire [15:0] cmd_spm_nnzr = cmd_config_data[31:16]; // Non Zero Rows (Simplify fetching the rows) (16 bits)


////////////////////////////////////////////////////////////////////////////////////////////////////
// State Machine for Commands and Operation
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum reg [1:0] {
    IDLE,                   // Doing Nothing 
    SPMV_INIT,              // Handle Grabbbing Vectors 
    VEC_PREFETCH,          // Grab the Dense Vector (Assume that the size is the same as a cache line (max two unlocal memory access))     
    COMPUTE_SPMV
} spmv_state;

spmv_state op_state;

reg [1:0] spmv_init_checksum; //Doesn't begin prefetching until Checksum = 2

// Next State Logic 
always_ff @ (posedge clk) begin
    if (!rst_n) begin
        op_state <= IDLE;
    end
    else begin
        case (op_state) 
            IDLE: begin
                if (cmd_hsk && (cmd==INIT_SPMV)) op_state <= SPMV_INIT;
            end
            SPMV_INIT: begin
                if (spmv_init_checksum == 2) op_state <= VEC_PREFETCH;
            end
            VEC_PREFETCH: begin
                // Control signal from a memory control unit
            end
            COMPUTE_SPMV: begin

            end
            default: begin

            end
        endcase
    end
end

// State Outputs
always @ (*) begin
    init_spm_pntr = 0;
    init_ld_spm_arr = 0;
    init_ld_vec = 0;
    
    case (op_state)
        IDLE: begin
            init_spm_pntr = cmd_hsk && (cmd==INIT_SPMV);
        end
        SPMV_INIT: begin
            init_ld_spm_arr = cmd_hsk && (cmd == LD_SPM);
            init_ld_vec = cmd_hsk && (cmd == LD_VEC);
        end
        VEC_PREFETCH: begin
            // Output control for the vector prefetch from memory
            // For instance take control of the memory unit (Shared between the pipeline and the computational channels) 
        end
        COMPUTE_SPMV: begin
            // Associate Channel Control Logic for starting the computation
        end
        default : begin

        end
    endcase
    
end

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accelerator Registers Storing Pointers to Arrays and Lengths
////////////////////////////////////////////////////////////////////////////////////////////////////////

reg init_spm_pntr, init_ld_spm_arr, init_ld_vec; 

reg [39:0] spm_val_pntr, spm_col_idx_pntr, spm_row_len_pntr, vec_pntr;
reg [19:0] vec_len;
reg [15:0] spm_nnz, spm_nzzr; 

always_ff @ (poedge clk) begin
    if (!rst_n) begin
        spmv_init_checksum <= 0;
        spm_val_pntr <= 0;
        spm_col_idx_pntr <= 0;
        spm_row_len_pntr <= 0;
        spm_nnz <= 0;
        spm_nnzr <= 0;
    end
    else begin
        if (init_spm_pntr) begin
            spm_val_pntr <= cmd_spm_base_pntr;
            spmv_init_checksum <= 0; // Reset every initialization call
        end
        else if (init_ld_spm_arr) begin
            spm_nnz <= cmd_spm_nnz;
            spm_nnzr <= cmd_spm_nzzr;

            spm_col_idx_pntr <= spm_val_pntr + cmd_spm_nnz;
            spm_row_len_pntr <= spm_val_pntr + (cmd_spm_nnz << 1); // Multiply length by 2

            spmv_init_checksum <= spmv_init_checksum + 1; 
        end
        else if (init_ld_vec) begin
            vec_pntr <= cmd_vec_pntr;
            vec_len <= cmd_vec_len;

            spmv_init_checksum <= spmv_init_checksum + 1;
        end
    end
end

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Vector Prefetching Control 
///////////////////////////////////////////////////////////////////////////////////////////////////////




endmodule