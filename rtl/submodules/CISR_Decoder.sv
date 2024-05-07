"include dcp.h"

module CISR_Decoder #(
    parameter VEC_W      = 32,
    parameter CHANNELS   = 16
) (
    input  wire                              clk,
    input  wire                              rst_n,
    // input from channel
    input wire [VEC_W-1:0]              row_lenths [CHANNELS-1:0],
    // output to channel
    output wire [VEC_W-1:0]             row_IDs [CHANNELS-1:0]
);

    module four_bit_counter (
        input wire clk,         // Clock signal
        input wire initialize,         // Signal to initialize the counter
        input wire decrement,      // Count decrement signal
        input wire [3:0] starting,  // Value to initialize the counter with
        output reg [3:0] count  // 4-bit counter output
    );

        // Counter operation
        always @(posedge clk or posedge rst) begin
            if (initialize) begin
                count <= starting;  // Reset the counter to starting value
            end else if (decrement) begin
                count <= count - 1;  // decrement the counter
            end
        end

    endmodule

integer i;

reg [VEC_W:0] row_ID_LIST [CHANNELS:0];
for (i=0; i<CHANNELS; i=i+1) begin
    row_ID_LIST[i] = i;
end

endmodule 