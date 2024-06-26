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

//`include "dcp.h" // Replace with a test file

`include "dcp_mock.svh"
`include "./submodules/spm_arbiter.sv"
`include "./submodules/spm_channel.sv"
`include "./submodules/spm_cisr_decoder.sv"
`include "./submodules/vec_file.sv"

`ifdef DEFAULT_NETTYPE_NONE 
`default_nettype none
`endif


module tight_acc_iface #(
    parameter NUM_CH = 16,
    parameter DATA_W = 32,
)  (
    input  wire clk,
    input  wire rst_n,
    // Command iface to receive "instructions" and configurations
    input  wire                             cmd_val,        // New valid command
    output reg                              busy,           // effectively behaves as cmd_rdy
    input  wire [5:0]                       cmd_opcode,     // Command operation code, 64 values
    input  wire [63:0]                      cmd_config_data, // Payload of command if needed

    // Interface to respond to the core after the accelerator has processed data
    output wire                             resp_val,
    input  wire                             resp_rdy, //whether the core is ready to take the data
    output reg [63:0]                       resp_data,

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


// assign mem_req_val = 1'b0;
// assign mem_req_transid = 6'b0;
// assign mem_req_addr = 40'd0;
// FOO implementation, respond untouched every command
assign resp_val = cmd_val;
assign resp_data = cmd_config_data; 

// Command Manager
typedef enum reg[1:0] {
    INIT_SPMV,           // Send the SPM Matrix pointer 
    LD_SPM_DATA,         // Send the info regarding length
    LD_VEC_DATA,         // Load the vector  
    GET_RESULT           // Return the Result       
} spmv_cmd;

spmv_cmd cmd;
wire cmd_hsk = !busy & cmd_val; // Command Hanshake with Processor 


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command Parsing
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

assign cmd = cmd_opcode[1:0];

wire [`DCP_PADDR_MASK       ] cmd_spm_base_pntr = cmd_config_data[`DCP_PADDR_MASK       ]; //The physical address is 40 bits (configurable) 1TB
wire [`DCP_PADDR_MASK       ] cmd_vec_pntr = cmd_config_data[`DCP_PADDR_MASK       ]; //The physical address is 40 bits (configurable) 1TB
wire [`DIM_W-1:0] cmd_vec_len = cmd_config_data[`DIM_W-1:40]; // Vector length when sending over the pointer (16 bits)
wire [`NNZ_W-1:0] cmd_spm_nnz = cmd_config_data[`NNZ_W-1:0];  // Non Zero Elements (16-bits)
wire [`DIM_W-1:0] cmd_spm_nr = cmd_config_data[`NNZ_W+`DIM_W-1:`NNZ_W]; // Non Zero Rows (Simplify fetching the rows) (16 bits)

////////////////////////////////////////////////////////////////////////////////////////////////////
// State Machine for Commands and Operation
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum reg [2:0] {
    IDLE,                   // Doing Nothing 
    SPMV_INIT,              // Handle Grabbing Vectors 
    VEC_PREFETCH,           // Grab the Dense Vector (Assume that the size is the same as a cache line (max two unlocal memory access))     
    SPMV_COMPUTE,           // Compute the Sparse Matrix Vector Multiplication
    SPMV_FINISHED           // Computation Finished 
} spmv_state;

spmv_state op_state;

reg [1:0] spmv_init_checksum; //Doesn't begin prefetching until Checksum = 2

// Next State Logic and Inputs
wire prefetch_done;
wire output_full;

always_ff @ (posedge clk) begin
    if (!rst_n) begin
        op_state <= IDLE; // Default State 
    end
    else begin
        case (op_state) 
            IDLE: begin
                // API Command To Initialize Computation
                if (cmd_hsk && (cmd==INIT_SPMV)) op_state <= SPMV_INIT;
            end
            SPMV_INIT: begin
                // If all the data has been loaded begin Prefetching Vector
                if (spmv_init_checksum == 2) op_state <= VEC_PREFETCH;
            end
            VEC_PREFETCH: begin
                // Control signal from a memory control unit
                if(prefetch_done) op_state <= COMPUTE_SPMV;
            end
            SPMV_COMPUTE: begin
                // Once the output buffer for the result vector is full go to finished state
                if(output_full) op_state <= SPMV_FINISHED;
            end
            SPMV_FINISHED: begin
                // If there is a new command to start another initialization begin
                if (cmd_hsk && (cmd==INIT_SPMV)) op_state <= SPMV_INIT;
                else if (result_returned) op_state <= IDLE; // Otherwise go to IDLE state
            end
            default: begin

            end
        endcase
    end
end

// State Outputs
reg spmv_init, spmv_prefetch, spm_fetch, get_result;

always @ (*) begin
    init_spm_pntr = 0;
    init_ld_spm_arr = 0;
    init_ld_vec = 0;
    spmv_init = 0; // Reset Signal for all other logic
    spmv_prefetch = 0;
    spm_fetch = 0;
    resp_val = 0;
    busy = 0;
    case (op_state)
        IDLE: begin
            init_spm_pntr = cmd_hsk && (cmd==INIT_SPMV);
        end
        // SPMV Init signal resets all of the modules to default values to prepare for operation
        SPMV_INIT: begin
            spmv_init = 1;
            init_ld_spm_arr = cmd_hsk && (cmd == LD_SPM_DATA);
            init_ld_vec = cmd_hsk && (cmd == LD_VEC_DATA);
        end
        // State to fetch the whole vector from memory before computing (Much faster than iterative calls to memory )
        VEC_PREFETCH: begin
            // Output control for the vector prefetch from memory
            spmv_prefetch = 1;
        end
        // Compute the Sparse Matrix Vector Product
        SPMV_COMPUTE: begin
            spm_fetch = 1; // Arbiter Begins fetching SPM values
            busy = 1; // Accelerator can no longer recieve commands
        end
        // Computation Finished
        SPMV_FINISHED: begin
            get_result = cmd_hsk && (cmd == LD_SPM_DATA); // Enable the processor to begin asking for values
            busy = !(results_returned) && retrieve_en; // Falls as soon as last cycle
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
reg [15:0] vec_len;
reg [`NNZ_W-1:0] spm_nnz; 
reg [`DIM_W-1:0] spm_nr;

always_ff @ (poedge clk) begin
    if (!rst_n) begin
        spmv_init_checksum <= 0;
        spm_val_pntr <= 0;
        spm_col_idx_pntr <= 0;
        spm_row_len_pntr <= 0;
        spm_nnz <= 0;
        spm_nr <= 0;
    end
    else begin
        if (init_spm_pntr) begin
            spm_val_pntr <= cmd_spm_base_pntr; // Load the Sparse Matrixes Base Vector
            spmv_init_checksum <= 0; // Reset every initialization call
        end
        else if (init_ld_spm_arr) begin
            spm_nnz <= cmd_spm_nnz; // Load the number of non-zero values
            spm_nr <= cmd_spm_nr; // Number of row lengths 

            spm_col_idx_pntr <= spm_val_pntr + cmd_spm_nnz; // Store the pointers
            spm_row_len_pntr <= spm_val_pntr + (cmd_spm_nnz << 1); // Multiply length by 2

            spmv_init_checksum <= spmv_init_checksum + 1; // Add to the Checksum
        end
        else if (init_ld_vec) begin
            vec_pntr <= cmd_vec_pntr; // 
            vec_len <= cmd_vec_len;

            spmv_init_checksum <= spmv_init_checksum + 1;
        end
    end
end

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel Pipeline
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Multi-channel Inputs
wire [`DIM_W-1:0]  cisr_decoder_row_id [NUM_CH-1:0];
wire [DATA_W-1:0]  vec_val_in [NUM_CH-1:0];

wire [DATA_W-1:0]  spm_val [NUM_CH-1:0];
wire [DATA_W-1:0]  spm_row_len [NUM_CH-1:0];
wire [DATA_W-1:0]  spm_col_idx [NUM_CH-1:0];

wire [NUM_CH-1:0]  decoder_pop_len;

wire [NUM_CH-1:0] last_not_valid;

// Multi-Channel Outputs
wire [DATA_W-1:0]  decoder_row_lens             [NUM_CH-1:0];
wire [`DIM_W-1:0]  row_IDS_accumulator_out      [NUM_CH-1:0];
wire [DATA_W-1:0]  mul_accumulator_out          [NUM_CH-1:0];
wire [`DIM_W-1:0]  col_IDs_BVB_out              [NUM_CH-1:0];

reg [NUM_CH-1:0] channel_acc_done; //Connected to Computation Pipeline

wire [NUM_CH-1:0] pipe_fetch_bubble;

// Fanned Input
wire spm_fetch_stall;

genvar h;
generate 
    for (h=0; h < NUM_CH; h=h+1) begin: create_parallel_channels
        spm_channel #(
            .NUM_CH                     (NUM_CH),
            .DATA_W                     (DATA_W)
        ) spm_channel (
            .clk                        (clk),
            .rst_n                      (rst_n),
            .spmv_init                  (spmv_init),

            // Inputs from other modules
            .row_IDs_decoder_in         (cisr_decoder_row_id[h]),
            .vector_values_BVB_in       (vec_val_in[h]), // Comes from the vec_file
            .decoder_pop_len            (decoder_pop_len[h]), // Comes from the decoder

            // Inputs
            .spm_val                    (spm_val[h]), // Values in the Sparse Matrix 
            .spm_row_len                (spm_row_len[h]), // Row lengths 
            .spm_fetch_stall            (spm_fetch_stall || last_not_valid[h]), // Each pipe can have a bubble if the last value 

            // Outputs 
            .decoder_row_lens_out       (decoder_row_lens[h]), // Row lengths going to the Decoder
            .row_IDS_accumulator_out    (row_IDs_accumulator_out[h]), // Row IDs 
            .mul_accumulator_out        (mul_accumulator_out[h]), // Accumulators Out
            .col_IDs_BVB_out            (col_IDs_BVB_out[h]), // Column IDs to the Vector File
            .acc_new                    (channel_acc_done[h]), // New values available from the accumulator 

            .pipe_fetch_bubble          (pipe_fetch_bubble[h]) // Can have separate channel bubbles at end of computation
        );
    end
endgenerate


///////////////////////////////////////////////////////////////////////////////////////////////////////
// Vector Prefetching Control 
///////////////////////////////////////////////////////////////////////////////////////////////////////

wire vec_mem_req_rdy, vec_mem_req_val, vec_mem_resp_val;
wire [5:0] vec_mem_req_transid, vec_mem_resp_transid;
wire [`DCP_PADDR_MASK       ] vec_mem_req_addr;
wire [`DCP_NOC_RES_DATA_SIZE-1:0] vec_mem_resp_data;

// Buffer Storing all of the Vector Elements
vec_file #(
    .VEC_W                  (DATA_W),
    .NUM_CH                 (NUM_CH)
) vec_file (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .spmv_init              (spmv_init), 
    .prefetch               (spmv_prefetch),
    .vec_pntr               (vec_pntr),
    .vec_len                (vec_len),

    .col_idx_in             (spm_col_idx), // Connected to Channel Pipeline Stage

    .mem_req_rdy            (vec_mem_req_rdy),
    .mem_req_val            (vec_mem_req_val),
    .mem_req_transid        (vec_mem_req_transid),
    .mem_req_addr           (vec_mem_req_addr),

    .mem_resp_val           (vec_mem_resp_val),
    .mem_resp_transid       (vec_mem_resp_transid),
    .mem_resp_data          (vec_mem_resp_data),

    .prefetch_done          (prefetch_done),
    .col_val_out            (vec_val_in)
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Sparse Matrix Element Access and Arbitration
///////////////////////////////////////////////////////////////////////////////////////////////////////

wire spm_mem_req_rdy, spm_mem_req_val, spm_mem_resp_val;
wire [5:0] spm_mem_req_transid, spm_mem_resp_transid;
wire [`DCP_PADDR_MASK       ] spm_mem_req_addr;
wire [`DCP_NOC_RES_DATA_SIZE-1:0] spm_mem_resp_data;

// Arbiter For Fetching the Sparse Matrix Values 
spm_arbiter #(
    .CHAN_NUM               (NUM_CH),
    .SPM_ELE_W              (DATA_W)
) spm_arbiter (
    .clk                        (clk),
    .rst_n                      (rst_n),

    // Sparse Matrix Data Inputs
    .spmv_init                  (spmv_init),
    .spm_fetch                  (spm_fetch),
    .spm_val_pntr               (spm_val_pntr),
    .spm_col_idx_pntr           (spm_col_idx_pntr),
    .spm_row_len_pntr           (spm_row_len_pntr),
    .spm_nnz                    (spm_nnz),
    .spm_nr                     (spm_nr),

    // NoC1 Interface
    .mem_req_rdy                (spm_mem_req_rdy),
    .mem_req_val                (spm_mem_req_val),
    .mem_req_transid            (spm_mem_req_transid),
    .mem_req_addr               (spm_mem_req_addr),

    // NoC2 Inputs
    .mem_resp_val               (spm_mem_resp_val),
    .mem_resp_transid           (spm_mem_resp_transid),
    .mem_resp_data              (spm_mem_resp_data),

    // Outputs
    .spm_val                    (spm_val),                     // All Channel Pipe Info
    .spm_row_len                (spm_row_len),
    .spm_col_idx                (spm_col_idx),
    .spm_fetch_stall            (spm_fetch_stall),                     // Insert Bubbles into Pipeline
    .last_not_valid             ()
    .spm_fetch_done             (),                      // Pass into Pipe to accumulator_done (Accumulator Tells When Finished)

)

///////////////////////////////////////////////////////////////////////////////////////////////////////
// NoC1 and NoC2 Multiplexing Between Sparse Matrix Element Access and Vector Prefetch
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Demux NoC1 Inputs
assign vec_mem_req_rdy = spmv_prefetch ? mem_req_rdy : 0;
assign spm_mem_req_rdy = spm_fetch ? mem_req_rdy : 0;

// Demux NoC2 Response
assign vec_mem_resp_val = spmv_prefetch ? mem_resp_val : 0;
assign spm_mem_resp_val = spm_fetch ? mem_resp_val : 0;
assign vec_mem_resp_transid = spmv_prefetch ? mem_resp_transid : 0;
assign spm_mem_resp_transid = spm_fetch ? mem_resp_transid : 0;
assign vec_mem_resp_data = spmv_prefetch ? mem_resp_data : 0;
assign spm_mem_resp_data = spm_fetch ? mem_resp_data : 0;

// Mux NoC1 Outputs 
assign mem_req_val = spmv_prefetch ? vec_mem_req_val :
                     spm_fetch     ? spm_mem_req_val : 0;
assign mem_req_transid = spmv_prefetch ? vec_mem_req_transid :
                         spm_fetch     ? spm_mem_req_transid : 0;
assign mem_req_addr = spmv_prefetch ? vec_mem_req_addr :
                      spm_fetch     ? spm_mem_req_addr : 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////
// CISR Decoder 
///////////////////////////////////////////////////////////////////////////////////////////////////////

cisr_decoder #(
    .NUM_CH                     (NUM_CH),
    .DATA_W                     (DATA_W)
) cisr_decoder ( 
    .clk                        (clk),
    .rst_n                      (rst_n),

    // Inputs
    .spmv_init                  (spmv_init),
    .row_len                    (decoder_row_lens),         // Row Buffer inputs
    .pipe_bubble                (pipe_fetch_bubble),        // Channel Pipe Bubble (For different channels)

    // Outputs
    .row_len_pop                (decoder_pop_len),         // Send to Pipe 
    .row_idx_out                (cisr_decoder_row_id)          // Send to Pipe 
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Output Buffer (Regular Vector RegFile) Max Output Vector Length (Row Number): 1024
///////////////////////////////////////////////////////////////////////////////////////////////////////

reg [DATA_W-1:0] output_vec [`MAX_DIM_LEN-1:0]; // Vector Buffer 
reg [`DIM_W:0] channel_row_idx [NUM_CH-1:0];
reg [`DIM_W-1:0] rows_calc;

assign output_full = (rows_calc >= spm_nr); // Full output vector if total rows

// Storing the Values in the Buffer  
always_ff @(posedge clk) begin
    if (!rst_n) begin
        output_vec <= '{default: '0};
        rows_calc <= 0;
    end
    else if (spmv_init) rows_calc <= 0;
    else begin
        for(i=0; i < NUM_CH; i=i+1) begin
            // If accumulator is pushing out value 
            if (channel_acc_done[i]) begin
                output_vec[channel_row_idx[i]] <= mul_accumulator_out[i]; // Outputs 
                rows_calc <= rows_calc + 1; // Counts total elements stored
            end
        end
    end
end

// Core Request Handshake for value retrieval
wire core_resp_hsk = resp_val && resp_rdy;
assign resp_val = retrieve_en;
assign result_returned = (rows_retrieved == (rows_calc-1)) && core_resp_hsk; // Next clock cycle reset

// Result Communication to the Core 
reg [`DIM_W-1:0] rows_retrieved;
always_ff @(posedge clk) begin
    if (!rst_n || spm_init) begin
        resp_data <= 0;
        retrieve_en <= 0;
        rows_retrieved <= 0;
    end
    else begin
        if (get_result) retrieve_en <= 1; // Activate the result retrieval
        else if (result_returned) begin 
            retrieve_en <= 1;
            rows_retrieved <= 0;
        end
        // If core asking for value push out to the core 
        if (core_resp_hsk) begin
            resp_data <= output_vec[rows_retrieved];
            rows_retrieved <= rows_retrieved + 1;
        end
    end
end

endmodule