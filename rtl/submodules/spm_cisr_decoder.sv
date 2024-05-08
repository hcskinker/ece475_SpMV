`include "../dcp_mock.svh"

/*
Decoder Module the Interleaved Sparse Representation (CISR)
*/

module cisr_decoder #(
    parameter NUM_CH = 16,
    parameter DATA_W    = 32
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // Input to Start SPMV Calculation
    input  wire                    spmv_init,

    // Inputs from the Row Len Buffer
    input  wire [DATA_W-1:0]       row_len        [NUM_CH-1:0], // Input from each channels row lenght fifo
    input  wire [NUM_CH-1:0]       pipe_bubble,    // Bubble in Channel's Fetch Pipeline
    
    // Output Row_ids
    output wire [NUM_CH-1:0]       row_len_pop,    // Indicates need another index
    output reg  [`DIM_W-1:0]       row_idx_out    [NUM_CH-1:0] // Output Indices to Tell Accumulator which row it is summing to
);

localparam CHAN_W = $clog2(NUM_CH); // Bits required to store the channel numbers

// Initial Signal to indicate each channel should be assigned its index at the start of computation
reg default_row_idx;

reg [DATA_W-1:0]  row_len_count [NUM_CH-1:0];
reg [`DIM_W-1:0]  row_idx  [NUM_CH-1:0];

// Checks for if Row Lenght Counter is Zero 
// When zero give it a new value 
genvar k;
generate 
    for (k=0; k < NUM_CH; k = k + 1) begin : row_len_pop_signal
        assign row_len_pop[k] = (row_len_count[k] == 0); // Still applies in default case as well
    end
endgenerate

// ROW ID Calculation
integer i;

reg [CHAN_W-1:0] idx_offset;
reg [`DIM_W-1:0] avail_idx; // Pointer to the next available row_id for all channels
// Edit so Pipe Bubble is Channel Dependent
always @(posedge clk) begin
    idx_offset = 0;
    if(!rst_n || spmv_init) begin
        default_row_idx <= 1;
        for (i=0; i < NUM_CH; i = i + 1) begin
            row_idx[i] = i;
        end
        avail_idx <= CHAN_NUM;
    end
    else begin
        for (i=0; i < NUM_CH; i = i + 1) begin 
            if (!pipe_bubble[i]) begin
                default_row_idx <= 0; // After the first non-bubble initialize all row_idx to channel num
                if (default_row_idx) begin 
                    avail_idx <= CHAN_NUM;
                    row_idx_out[i] <= row_idx[i]; // First Element load
                end
                else begin
                    if (row_len_pop[i]) begin 
                        // Use encoder to decide 
                        row_idx_out[i] <= priority_val[i] + avail_idx; // Don't update ID until pop
                        idx_offset = idx_offset + 1;    
                    end
                end
            end
        end
    avail_idx <= avail_idx + idx_offset; // Update available row ids, won't change if bubble
    end
end

reg [DATA_W-1:0] priority_val [NUM_CH-1:0];
reg [CHAN_W-1:0] priority_i;


// Priority Encoder for the New Row Idx assignment
// If several channels need new row ids use a priority encoder based on channel index to assign how large 
// of a new row id to allocate based of the current id available for next use
always @(*) begin
    priority_val = '{default: '0};
    priority_i = 0;
    for (i=0; i < NUM_CH; i = i + 1) begin
        if (!pipe_bubble[i]) begin
            if (row_len_pop[i]) begin
                priority_val[i] = priority_i;
                priority_i = priority_i + 1;
            end
        end
    end
end

// Channel Row Counters
always_ff @(posedge clk) begin
    if(!rst_n || spmv_init) begin
        row_len_count <= '{default :'0};
    end
    else begin
        for (i=0; i < NUM_CH; i = i + 1) begin 
            if (!pipe_bubble[i]) begin
                if (row_len_pop[i]) row_len_count[i] <= row_len[i]; // If at zero load a new value
                else row_len_count[i] <= row_len_count[i] - 1; //
            end
        end
    end
end



endmodule