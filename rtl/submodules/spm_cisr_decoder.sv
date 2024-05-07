"include dcp.h"

module cisr_decoder #(
    parameter CHAN_NUM = 16,
    parameter LEN_W    = 32
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // Input to Start SPMV Calculation
    input  wire                    spmv_init,

    // Inputs from the Row Len Buffer
    input  wire [LEN_W-1:0]        row_len        [CHAN_NUM-1:0],
    input  wire                    bubble,       // Bubble for all channels
    
    // Output Row_ids
    output wire [CHAN_NUM-1:0]     row_len_pop,         // Indicates need another index
    output wire [15:0]             row_idx_out    [CHAN_NUM-1:0]
);

reg default_row_idx;

reg [LEN_W-1:0] row_len_count [CHAN_NUM-1:0];
reg [15:0]  row_idx  [CHAN_NUM-1:0];

genvar k;
generate 
    for (k=0; k < CHAN_NUM; k = k + 1) begin : row_len_pop_signal
        assign row_len_pop[k] = (row_len_count == 0); // Still applies in default case as well
    end
endgenerate

// ROW ID Calculation
integer i;
integer j;
reg 

always @(posedge clk) begin
    idx_offset = 0;
    if(!rst_n || spmv_init) begin
        default_row_idx <= 1;
        for (i=0; i < CHAN_NUM; i = i + 1) begin
            row_idx[i] = i;
        end
    end
    else if (!bubble) begin
        default_row_idx <= 0; // First (Non-bubble) sets to zero
        for (i=0; i < CHAN_NUM; i = i + 1) begin 
            if (default_row_idx) row_idx_out[i] <= row_idx[i]; // First Element load
            else begin
                if (row_len_pop[i]) begin 
                    row_idx_out[i] <= priority_val[i] + avail_idx; // Don't update ID until pop
                    idx_offset = idx_offset + 1;    
                end
            end
        end
        avail_idx <= avail_idx + idx_offset;
    end
end

localparam CHAN_W = $clog2(CHAN_NUM);
reg [LEN_W-1:0] priority_val [CHAN_NUM-1:0];
reg [CHAN_W-1:0] priority_i;


always @(*) begin
    priority_val = '{default: '0};
    priority_i = 0;
    if (!bubble) begin
        for (i=0; i < CHAN_NUM; i = i + 1) begin
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
    else if (!bubble) begin
        for (i=0; i < CHAN_NUM; i = i + 1) begin 
            if (row_len_pop[i]) row_len_count[i] <= row_len[i]; // If at zero load a new value
            else row_len_count <= row_len_count - 1; //
        end
    end
end



endmodule