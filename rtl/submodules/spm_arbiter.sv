`include "../dcp_mock.svh"

// Data Module to Handle Fetch the Sparse Matrix Values, Column Indexs, and Row Lengths from Memory

module spm_arbiter #(
    parameter NUM_CH = 16, 
    parameter DATA_W = 32
) (
    input  wire                             clk,
    input  wire                             rst_n,
    
    // Fetching Inputs and SPM data
    input  wire                              spmv_init,
    input  wire                              spm_fetch,
    input  wire [39:0]                       spm_val_pntr,
    input  wire [39:0]                       spm_col_idx_pntr,
    input  wire [39:0]                       spm_row_len_pntr,
    input  wire [`NNZ_W-1:0]                 spm_nnz, // Number of Non-Zero Elements
    input  wire [`DIM_W-1:0]                 spm_nr, // Number of Rows

    // Mem Req Signal Interface
    input  wire                              mem_req_rdy,
    output wire                              mem_req_val,
    output wire [5:0]                        mem_req_transid,
    output wire [`DCP_PADDR_MASK]            mem_req_addr, 

    // Mem Response Signal Interface
    input  wire                              mem_resp_val,
    input  wire [5:0]                        mem_resp_transid, // up to 64 outstanding requests 
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] mem_resp_data, //up to 64Bytes

    // Output to the Channels
    output reg  [DATA_W-1:0]                 spm_val        [NUM_CH-1:0],
    output reg  [DATA_W-1:0]                 spm_row_len    [NUM_CH-1:0],
    output reg  [DATA_W-1:0]                 spm_col_idx    [NUM_CH-1:0],
    output reg  [NUM_CH-1:0]                 last_not_valid,       // Only applies to the last values in a pipe
    output wire                              spm_fetch_stall, // Bubble signal if stall (don't push values),
    output reg                               spm_fetch_done
);

assign mem_req_transid[5:2] = 0;

// Type for Transaction ID's Improve Readability
typedef enum logic [1:0]{
    VALUE,
    COL_IDX,
    ROW_LEN
} trans_type;

// Align all of the Data Arrays to Cache
wire [`DCP_PADDR_MASK] first_val_addr = spm_val_pntr & ((~40'd0) << 6);
wire [`DCP_PADDR_MASK] first_col_addr = spm_col_idx_pntr & ((~40'd0) << 6);
wire [`DCP_PADDR_MASK] first_len_addr = spm_row_len_pntr & ((~40'd0) << 6);
wire [3:0] val_offset = spm_val_pntr[5:2]; 
wire [3:0] col_idx_offset = spm_col_idx_pntr[5:2];
wire [3:0] len_offset = spm_row_len_pntr[5:2];

// First Fetch important for initial data storage (loading the offset buffers first)
wire first_fetch = (iteration==0);
wire windup_stall = !(iteration > 1); // Windup allows the first fetch to fill the offset buffers and then on second fetch output data


// Stall when not all the Sparse Matrix Data for the 16 slots are ready
assign spm_fetch_stall = !fetch_finished || windup_stall || spm_fetch_done; 

///////////////////////////////////////////////////////////////////////////////
// Memory Requests
///////////////////////////////////////////////////////////////////////////////

// Transaction type for Requests
trans_type mem_req_trans;

assign mem_req_transid[1:0] = mem_req_trans;

localparam ALIGN = $clog2(`DCP_NOC_RES_DATA_SIZE/DATA_W);

// Done signal goes high on cycle after last cycle
reg spm_fetch_stop;

wire mem_req_hsk = mem_req_rdy && mem_req_val;

// Row Length Fetching will be finished before fetching all the non-zero values 
// Save time by not asking for row_lengths
wire row_fetch_done = (spm_nr <= iteration*NUM_CH);

// When done fetching row_lengths only increment transaction ID Twice 
wire [1:0] max_transid = !row_fetch_done ? 2'b11 : 2'b10;

// Wait to issue a new set of requests until get all three responses back
assign mem_req_val = !req_finished && spm_fetch && !spm_fetch_stop;

wire [`DCP_PADDR_MASK] val_addr = first_val_addr + {{(40-`DIM_W){'0}}, iteration}<<ALIGN;
wire [`DCP_PADDR_MASK] col_addr = first_col_addr + {{(40-`DIM_W){'0}}, iteration}<<ALIGN; 
wire [`DCP_PADDR_MASK] len_addr = first_len_addr + {{(40-`DIM_W){'0}}, iteration}<<ALIGN;

// Iteration Through which request address to issue 
assign mem_req_addr = (mem_req_trans == VALUE)   ? val_addr :
                      (mem_req_trans == COL_IDX) ? col_addr :
                      (mem_req_trans == ROW_LEN) ? len_addr : 40'b0;

// Iteration Counts the number of memory cache retrieval for all Sparse Matrix Values
reg [`DIM_W-1:0] iteration;
reg req_finished; 

// Issue three requests at a time corresponding to Values, Column Indices, and Row Lens (Before empty)
always_ff @(posedge clk) begin
    if(!rst_n || spmv_init) begin 
        mem_req_trans <= 0; 
        req_finished <= 0;
    end
    else if (mem_req_hsk) begin
        // Max transaction ID change depending on if there are no more row lengths to fetch
        if (mem_req_trans == (max_transid-1)) begin
            mem_req_trans <= 0;
            req_finished <= 1;
        end
        else begin 
            mem_req_trans <= mem_req_trans + 1;
        end
    end
    else if (!req_finished && fetch_finished) begin
        req_finished <= 0; // Reset Request Schedule 1 cycle after finishing (1 cycle delay)
    end
end

///////////////////////////////////////////////////////////////////////////////
// Memory Responses 
///////////////////////////////////////////////////////////////////////////////

// Response Transaction Identifier
trans_type mem_resp_trans = mem_resp_transid[1:0];

// Signals for Finished Fetches
reg [1:0] spm_response_sum; // Sum counts number of responses allows responses to be out of order
reg fetch_finished;

wire [DATA_W-1:0] mem_data [NUM_CH-1:0];

// Partition the Data from memory
// Divide into 16 slots
genvar j;
generate
    for (j = 0; j < NUM_CH; j = j + 1) begin: cache_split
        assign mem_data[j] = mem_resp_data[(j+1)*DATA_W-1:j*DATA_W];
    end
endgenerate

// Since array starting addresses can be misaligned to the cache line need offset registers
// To adjust the slot alignment to the cache alignment so that on every fetch we have all 16 slots from memory
reg [DATA_W-1:0] val_offset_queue [NUM_CH-1:0]; // Holds the Offset Values
reg [DATA_W-1:0] col_offset_queue [NUM_CH-1:0];
reg [DATA_W-1:0] len_offset_queue [NUM_CH-1:0];

wire [ALIGN-1:0] last_overflow = spm_nnz[ALIGN-1:0];

wire last_fetch = (spm_nnz >= {{(`NNZ_W-`DIM_W){'0}}, (iteration-10'd1)}<<ALIGN); // Same thing as multiplying by NUM_CH

always_ff @ (posedge clk) begin
    if(!rst_n || spmv_init) begin
        iteration <= 0;
        spm_response_sum <= 0;
        fetch_finished <= 0;
        spm_fetch_stop <= 0;
        spm_fetch_done <= 0;
        last_not_valid <= 0; // Last not valid is used for the last transaction and fetching overflow, sets invalid bits for output array for pipeline to insert bubbles
    end
    else begin
        // Last fetch occurs on the last fetch
        spm_fetch_stop <= last_fetch;
        spm_fetch_done <= spm_fetch_stop; // Asserted when nothing else is flowing
        if (spm_fetch && mem_resp_val && !spm_fetch_stop) begin
            // Assume we will be finished if we have a mem_resp_val and the sum is already 2
            if (spm_response_sum == (max_transid-1)) begin
                spm_response_sum <= 0;
                iteration <= iteration + 1; // Increment iteration count only after recieved three responses
                fetch_finished <= 1;
                last_not_valid <= 0;
            end 
            else begin
                // Add 1 to response sum
                spm_response_sum <= spm_response_sum + 1;
                fetch_finished <= 0;
            end
            // Same storage pattern for each 
            for (int i=0; i < NUM_CH; i = i + 1) begin
                case (mem_resp_trans)
                    VALUE : begin
                        // If first fetch only store values above the offset in offset array 
                        if (first_fetch && (i >= val_offset)) begin
                            val_offset_queue[i-val_offset] <= mem_data[i];
                        end
                        // Maintaining Data's slot alignment when fetching across cache lines
                        else begin
                            //  For values less then the offset, they get output to channel pipeline immediately
                            if (i < val_offset) spm_val[(NUM_CH - val_offset)+i] <= mem_data[i];
                            // For values above offset value, store in the offset array for output next iteration
                            else begin 
                                spm_val[i-val_offset] <= val_offset_queue[i-val_offset];
                                val_offset_queue[i-val_offset] <=  mem_data[i];
                            end
                        end
                    end
                    COL_IDX : begin
                        if (first_fetch && (i >= col_idx_offset)) begin
                            col_offset_queue[i-col_idx_offset] <= mem_data[i];
                        end
                        else begin
                            if (i < col_idx_offset) spm_col_idx[(NUM_CH - col_idx_offset)+i] <= mem_data[i];
                            else begin
                                spm_col_idx[i-col_idx_offset] <= col_offset_queue[i-col_idx_offset];
                                col_offset_queue[i-col_idx_offset] <=  mem_data[i];
                            end
                        end
                    end
                    ROW_LEN : begin
                        if (first_fetch && (i >= len_offset)) begin
                            len_offset_queue[i-len_offset] <= mem_data[i];
                        end
                        else begin
                            if (i < len_offset) spm_row_len[(NUM_CH - len_offset)+i] <= mem_data[i];
                            else begin
                                spm_row_len[i-len_offset] <= len_offset_queue[i-len_offset];
                                len_offset_queue[i-len_offset] <= mem_data[i];
                            end
                        end
                    end
                    default : begin
                    end
                endcase  
            end
        end
        // Iteration after last fetch where values still in offset buffer. Push into pipeline and invalidate other values
        else if (spm_fetch_stop && !spm_fetch_done) begin
            for (int i=0; i<NUM_CH; i = i + 1) begin
                 spm_val[i] <= val_offset_queue[i];
                 spm_col_idx[i] <= col_offset_queue[i];
                 spm_row_len[i] <= len_offset_queue[i];
                 if (i>= last_overflow) last_not_valid[i] <= 1;
            end
        end
    end
end 

endmodule 