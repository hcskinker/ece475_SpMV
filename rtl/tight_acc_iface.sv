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

module tight_acc_iface (
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
    INIT_SPMV,
    LD_SPM,             // With this command you also pass the length of the spm component arrays 
    LD_VECTOR,          // Load the vector
    BEGIN_SPMV          
}



typedef enum reg [1:0] {
    IDLE,                   // Doing Nothing 
    VECT_PREFETCH,          // Grab the Dense Vector (Assume that the size is the same as a cache line (max two unlocal memory access))     
    COMPUTE_SPMV
} spmv_state;

spmv_state op_state;

// Next State Logic 
always_ff @ (posedge clk) begin
    if (!rst_n) begin

    end
    else begin
        case (op_state) 
            IDLE: begin

            end
            VECT_PREFETCH: begin

            end
            COMPUTE_SPMV: begin

            end
            default: begin
                
            end

        endcase
    end
end

// State Output
always @ (*) begin
    if (!rst_n) begin

    end
    else begin
      case (op_state)
          IDLE: begin

          end
          VECT_PREFETCH: begin

          end
          COMPUTE_SPMV: begin

          end
          default : begin

          end
      endcase
    end

end

// Other things that we need are a software algorithm to convert a matrix into the CISR format


// Memory Handler for the Sparse Matrix and Converter to the CISR Format 

/*
Things needed:
- Data line width (how long do we want the words/do we want to fetch multiple words at a time)
- Do we need a buffer or is this handled on the NOC side 
- There are two components that will be accessing the memory
    - The Sparse Matrix Loader 
    - The vector loader 

- Cache line is 64 bytes (8 64 bit words 

- Ensure that the fetches are cache aligned otherwise the algorithm wouldn't work (requires that the matrices are aligned)
- Or you design the architecture such that if you miss on the some of the values in the cache line (ie a memory call returns less values then expected you can still align the fetch later)

- Do we want to use a large section of the RAM to store the vector instead of doing the crossbar 
- What access structure do we need if we are parallelizing the result 

- Need a physical memory address (Does this mean we need access to the memory virtualization process?)
- Or can we assume that the processor sends us the correct memory address? 


We can define the integer word size in C++ everything is assumed to be 64 bits and just broken up into smaller word sizes 


Vector Loads Independent (Spatially Local expect first cache miss and then multiple cache hits (Large Cache Words in the L2)) -> 

5 Commands recieved from the processor:
- Sparse Matrix Value Vector (Address pointer)
- Sparse Matrix Row Length (CISR) pointer 
- Sparse Matrix Vector Channel ID pointer 
- Sparse Matrix Vector Column Index pointer
- Multiplication Vector


Memory Request Backup in the Queue 
1. Assume that we are going to miss on one of the cache lines for the matrix (They are all stored locally)
2. Want another transaction ID for the next round of values stored in the CISR format (Or we can just wait for the original transaction ID to come back and stall)


*/

// Parallelized Computation Channels 

/*
We need to load the values from the sparse matrix memory into an arbriter struture that directs the vectors into whichever channel they should be in

- Things to note 
  - Need to know the memory offset for the matrixes in the physical memory address space

- We will have 4 channels starting out. (Depending on the cache line length and the number word)

4 channel ID's that the rows for the CISR will get converted for 
- Each channel has its own fifo involving an ID 


*/

// Pipeline Diagram Rough 


/*

Use buffers to segment the pipeline stages

Single Stage: (Wait until complete)
1. Load Input Vector 

Computational Pipeline: 

1. Fetch CISR Matrix element (Segemented into 4 data lines) have to wait until all three arrive (stall)
 - Value (Trans id 1)
 - Column Indice (Trans id 2)
 - Row Length (Trans id 3)

This fetch needs to be queued so that we can continuously pull the values 

2. Arbiter figures which channel to forward the values to depending on which channel is available

3. Channel Calculation (Mul 0)
 - Read the vector and forward to the appropiate channel 
 - Start Multiplication 

4. (Mul 1) // at some point insert a row id to the element to tell the accumulator which row the calculation belongs to 
5. (Mul 2) 
6. (Mul 3)

7. Move into accumulator (Sums for each row) Max # of sums being kept track of equivalent to number of channels
  - Multiple channels each keeping track of the row id conversion done inside the channel fifo
*/






endmodule