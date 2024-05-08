`include "dcp_mock.svh"

module cisr_decoder #(
    parameter NUM_CH = 16,
    parameter DATA_W    = 32
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // Input to Start SPMV Calculation
    input  wire                    spmv_init,

    // Inputs from the Row Len Buffer
    input  wire [DATA_W-1:0]       row_len        [NUM_CH-1:0],
    input  wire [NUM_CH-1:0]       pipe_bubble,    // Bubble for all channels
    
    // Output Row_ids
    output wire [NUM_CH-1:0]       row_len_pop,         // Indicates need another index
    output wire [15:0]             row_idx_out    [NUM_CH-1:0]
);

reg default_row_idx;

reg [DATA_W-1:0] row_len_count [NUM_CH-1:0];
reg [`DIM_W-1:0]  row_idx  [NUM_CH-1:0];

genvar k;
generate 
    for (k=0; k < NUM_CH; k = k + 1) begin : row_len_pop_signal
        assign row_len_pop[k] = (row_len_count == 0); // Still applies in default case as well
    end
endgenerate

// ROW ID Calculation
integer i;
integer j;


// Edit so Pipe Bubble is Channel Dependent
always @(posedge clk) begin
    idx_offset = 0;
    if(!rst_n || spmv_init) begin
        default_row_idx <= 1;
        for (i=0; i < NUM_CH; i = i + 1) begin
            row_idx[i] = i;
        end
    end
    else begin
        for (i=0; i < NUM_CH; i = i + 1) begin 
            if (!pipe_bubble[i]) begin
                default_row_idx <= 0; // After the first non-bubble initialize all row_idx to channel num
                if (default_row_idx) row_idx_out[i] <= row_idx[i]; // First Element load
                else begin
                    if (row_len_pop[i]) begin 
                        row_idx_out[i] <= priority_val[i] + avail_idx; // Don't update ID until pop
                        idx_offset = idx_offset + 1;    
                    end
                end
            end
        end
    avail_idx <= avail_idx + idx_offset; // Update available row ids, won't change if bubble
    end
end

localparam CHAN_W = $clog2(NUM_CH);
reg [DATA_W-1:0] priority_val [NUM_CH-1:0];
reg [CHAN_W-1:0] priority_i;


// Priority Encoder for the New Row Idx assignment
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
                else row_len_count <= row_len_count - 1; //
            end
        end
    end
end



endmodule