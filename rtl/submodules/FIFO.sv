module fifo #(
    parameter DATA_WIDTH = 32,      // Number of bits in each data entry
    parameter FIFO_DEPTH = 16,     // Number of entries in the FIFO
    parameter ADDR_WIDTH = 4       // Number of bits for the address (log2(FIFO_DEPTH))
)(
    input wire clk,                // Clock signal
    input wire rst,                // Reset signal
    input wire wr_en,              // Write enable
    input wire rd_en,              // Read enable
    input wire [DATA_WIDTH-1:0] data_in, // Data input
    output reg [DATA_WIDTH-1:0] data_out, // Data output
    output wire full,              // FIFO is full
    output wire empty              // FIFO is empty
);

    // Memory array to store FIFO data
    reg [DATA_WIDTH-1:0] fifo_mem[FIFO_DEPTH-1:0];
    
    // Read and write pointers
    reg [ADDR_WIDTH-1:0] rd_ptr = 0, wr_ptr = 0;
    
    // FIFO counter to keep track of the number of elements
    reg [ADDR_WIDTH:0] fifo_count = 0;

    // FIFO is considered full when the counter reaches the FIFO_DEPTH
    assign full = (fifo_count == FIFO_DEPTH);

    // FIFO is considered empty when the counter is zero
    assign empty = (fifo_count == 0);

    // Write operation
    always @(posedge clk) begin
        if (rst) begin
            wr_ptr <= 0;
            fifo_count <= 0;
        end else if (wr_en && !full) begin
            fifo_mem[wr_ptr] <= data_in;
            wr_ptr <= (wr_ptr + 1) % FIFO_DEPTH;
            fifo_count <= fifo_count + 1;
        end
    end

    // Read operation
    always @(posedge clk) begin
        if (rst) begin
            rd_ptr <= 0;
            data_out <= 0;
        end else if (rd_en && !empty) begin
            data_out <= fifo_mem[rd_ptr];
            rd_ptr <= (rd_ptr + 1) % FIFO_DEPTH;
            fifo_count <= fifo_count - 1;
        end
    end

endmodule