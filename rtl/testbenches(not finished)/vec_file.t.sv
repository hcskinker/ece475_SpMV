/*
Test bench was Chatgpt Created, minor edits have been made to adjust to specifications 
*/ 


`timescale 1ns / 1ps
`include "../dcp_mock.svh"
`include "vec_file.sv"

module tb_vec_file;

  // Parameters
  parameter DATA_W = 32; // Data width in bits
  parameter NUM_CH = 16; // Number of channels
  parameter ADDR_W = 32; // Address width in bits
  parameter CACHE_LINE_SIZE = 64; // Cache line size bytes
  parameter ELEMENTS_PER_LINE = CACHE_LINE_SIZE / (DATA_W / 8); // Elements per line
  parameter MAX_CACHE_LINES = 10; // Maximum number of cache lines

  // DUT Inputs
  reg clk, rst_n, spmv_init, prefetch, prefetch_done;
  reg [ADDR_W-1:0] vec_pntr; // Pointer to the vector in memory
  reg [9:0] vec_len = 48; // Length of the vector is fixed at 48 elements

  // Memory interfaces
  reg mem_req_rdy;
  wire mem_req_val;
  wire [5:0] mem_req_transid;
  wire [ADDR_W-1:0] mem_req_addr;
  reg [5:0] mem_resp_transid;
  reg [511:0] mem_resp_data;  // Memory response data width
  reg mem_resp_val;

  // DUT Outputs
  wire [DATA_W-1:0] col_val_out[NUM_CH-1:0];

  // Instantiate the DUT
  vec_file #(
    .DATA_W(DATA_W),
    .NUM_CH(NUM_CH)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .spmv_init(spmv_init),
    .prefetch(prefetch),
    .vec_pntr(vec_pntr),
    .vec_len(vec_len),
    .mem_req_rdy(mem_req_rdy),
    .mem_req_val(mem_req_val),
    .mem_req_transid(mem_req_transid),
    .mem_req_addr(mem_req_addr),
    .mem_resp_val(mem_resp_val),
    .mem_resp_transid(mem_resp_transid),
    .mem_resp_data(mem_resp_data),
    .col_val_out(col_val_out),
    .prefetch_done(prefetch_done)
  );

  // Clock generation
  always #5 clk = ~clk; // 100 MHz clock

  // Memory cache (simulated)
  reg [511:0] cache_memory[MAX_CACHE_LINES-1:0]; // Cache storage

  // Initialize test
  initial begin
    clk = 0;
    rst_n = 0;
    spmv_init = 0;
    prefetch = 0;
    vec_pntr = 512 + 4<<8; // Offset by 4 from the beginning of a cache line
    mem_req_rdy = 0;
    mem_resp_transid = 0;
    mem_resp_data = 0;
    mem_resp_val = 0;
    #10 rst_n = 1; // Reset release

    // Populate cache with some test data
    initialize_cache();

    // Start of tests
    test_prefetch();
  end

  // Populate cache with test data
    task initialize_cache();
    begin
        integer i, start_idx, idx;
        // Calculate the starting index in the cache memory for vec_pntr
        start_idx = (vec_pntr / CACHE_LINE_SIZE);
        idx = start_idx;
        // Fill the cache with elements 1 to 48 at the calculated index
        for (i = 0; i < 48; i = i + 1) begin
            cache_memory[idx][(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = i + 1; // Stores 1, 2, ..., 48 in the cache
            idx = start_idx + (i+1 / ELEMENTS_PER_LINE);
        end
    end
    endtask


  // Memory response simulation with random delay
  always @(posedge clk) begin
    if (mem_req_val && mem_req_rdy) begin
      // Emulate a random delay for the memory response, not exceeding 5 cycles
      repeat ($urandom_range(1, 5)) @(posedge clk);
      mem_resp_data <= cache_memory[mem_req_addr / CACHE_LINE_SIZE % MAX_CACHE_LINES];
      mem_resp_transid <= mem_req_transid;
      mem_resp_val <= 1'b1;
      @(posedge clk) mem_resp_val <= 1'b0; // Deassert next cycle
      mem_req_rdy <= 1'b0; // Deassert readiness until response handled
      @(posedge clk) mem_req_rdy <= 1'b1; // Reassert readiness
    end
  end

  // Test prefetch logic
task test_prefetch;
begin
    vec_pntr = 512 + 4<<8; // Set the vector pointer to a specific offset address
    prefetch = 1;  // Start prefetch by asserting the prefetch signal

    // Wait until prefetch_done is asserted by the vec_file module
    wait (dut.prefetch_done);

    prefetch = 0;  // Deassert prefetch once prefetch_done is asserted
    #200; // Allow some additional time for other operations and to observe outputs

    // Optionally, you can add checking mechanisms here to verify the correctness of the prefetch operation
    check_prefetch_completion();
end
endtask

// Check the results after prefetch completion
task check_prefetch_completion;
begin
    integer i;
    reg [DATA_W-1:0] expected_value;

    // Check outputs against the expected values based on column indices
    for (i = 0; i < NUM_CH; i++) begin
        expected_value = (i % 48) + 1;  // Calculate expected value assuming column IDs match indices
        if (col_val_out[i] !== expected_value) begin
            $display("Mismatch at channel %d: expected %d, got %d", i, expected_value, col_val_out[i]);
        end else begin
            $display("Data correct at channel %d: %d", i, col_val_out[i]);
        end
    end
end
endtask

endmodule