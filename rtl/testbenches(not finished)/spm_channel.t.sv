`timescale 1ns / 1ps

module tb_spm_channel;

    parameter SPM_ELE_W = 32;
    parameter CHAN_NUM = 16;

    reg                              clk;
    reg                              rst_n;
    reg [SPM_ELE_W-1:0]              row_IDs_decoder_in [CHAN_NUM-1:0];
    reg [SPM_ELE_W-1:0]              vector_values_BVB_in [CHAN_NUM-1:0];
    reg [SPM_ELE_W-1:0]              spm_val [CHAN_NUM-1:0];
    reg [SPM_ELE_W-1:0]              spm_row_len [CHAN_NUM-1:0];
    reg [SPM_ELE_W-1:0]              spm_col_idx [CHAN_NUM-1:0];
    reg                              spm_fetch_stall;

    wire [SPM_ELE_W-1:0]             row_lengths_decoder_out [CHAN_NUM-1:0];
    wire [SPM_ELE_W-1:0]             row_IDs_accumulator_out [CHAN_NUM-1:0];
    wire [SPM_ELE_W-1:0]             mul_accumulator_out [CHAN_NUM-1:0];
    wire [SPM_ELE_W-1:0]             col_IDs_BVB_out [CHAN_NUM-1:0];

    // Instantiate the module
    spm_channel #(
        .SPM_ELE_W(SPM_ELE_W),
        .CHAN_NUM(CHAN_NUM)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .row_IDs_decoder_in(row_IDs_decoder_in),
        .vector_values_BVB_in(vector_values_BVB_in),
        .spm_val(spm_val),
        .spm_row_len(spm_row_len),
        .spm_col_idx(spm_col_idx),
        .spm_fetch_stall(spm_fetch_stall),
        .row_lengths_decoder_out(row_lengths_decoder_out),
        .row_IDs_accumulator_out(row_IDs_accumulator_out),
        .mul_accumulator_out(mul_accumulator_out),
        .col_IDs_BVB_out(col_IDs_BVB_out)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Clock with a period of 10 ns
    end

    // Reset process
    initial begin
        rst_n = 1;
        #20 rst_n = 0;  // Release reset after 20 ns
    end

    // Test stimuli
    initial begin

        // Initialize inputs
        spm_fetch_stall = 0;
        @(posedge rst_n);  // Wait for reset release
        #10;               // Wait a bit after reset release

        // Apply some example stimuli
        for (int i = 0; i < CHAN_NUM; i++) begin
          row_IDs_decoder_in[i] = 1;
          vector_values_BVB_in[i] = 2;
          spm_val[i] =  3;
          spm_row_len[i] = 4;
          spm_col_idx[i] = 5;

        end

        // Toggle the stall signal
        #50 spm_fetch_stall = 1;
        #20 spm_fetch_stall = 0;

        // Modify inputs while running
        #100;
        for (int i = 0; i < CHAN_NUM; i++) begin
            vector_values_BVB_in[i] = i * 3;
            spm_val[i] = i + 10;
        end

        #50 $finish;  // End the simulation
    end

    // Generate waveform file
    initial begin
        $dumpfile("waveform.vcd");
      	$dumpvars;
    end
  initial #500ns $finish;

endmodule