`timescale 1ns/1ps

// =======================================================
// Testbench for Minilab1 (fixed top-level)
// - Stubs out mem_wrapper (and ROM contents) so you can
//   simulate without Altera altsyncram libraries.
// - Waits for DONE, then checks all 8 dot-product results.
// - Also prints LED state and HEX digits.
// =======================================================

module tb_minilab1;

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic CLOCK2_50, CLOCK3_50, CLOCK4_50, CLOCK_50;
  logic [3:0] KEY;
  logic [9:0] SW;

  logic [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
  logic [9:0] LEDR;

  // ----------------------------
  // Instantiate DUT
  // ----------------------------
  Minilab1 dut (
    .CLOCK2_50(CLOCK2_50),
    .CLOCK3_50(CLOCK3_50),
    .CLOCK4_50(CLOCK4_50),
    .CLOCK_50 (CLOCK_50),

    .HEX0(HEX0), .HEX1(HEX1), .HEX2(HEX2),
    .HEX3(HEX3), .HEX4(HEX4), .HEX5(HEX5),

    .LEDR(LEDR),
    .KEY(KEY),
    .SW(SW)
  );

  // ----------------------------
  // Clocks
  // ----------------------------
  initial begin
    CLOCK_50  = 1'b0;
    CLOCK2_50 = 1'b0;
    CLOCK3_50 = 1'b0;
    CLOCK4_50 = 1'b0;
  end

  // 50MHz => 20ns period
  always #10 CLOCK_50  = ~CLOCK_50;
  always #10 CLOCK2_50 = ~CLOCK2_50;
  always #10 CLOCK3_50 = ~CLOCK3_50;
  always #10 CLOCK4_50 = ~CLOCK4_50;

  // ----------------------------
  // Helper: 7-seg decode back to hex nibble (active-low)
  // Returns 4'hX if unknown/unmatched.
  // ----------------------------
  function automatic [3:0] seg7_to_nib(input logic [6:0] seg);
    case (seg)
      7'b1000000: seg7_to_nib = 4'h0;
      7'b1111001: seg7_to_nib = 4'h1;
      7'b0100100: seg7_to_nib = 4'h2;
      7'b0110000: seg7_to_nib = 4'h3;
      7'b0011001: seg7_to_nib = 4'h4;
      7'b0010010: seg7_to_nib = 4'h5;
      7'b0000010: seg7_to_nib = 4'h6;
      7'b1111000: seg7_to_nib = 4'h7;
      7'b0000000: seg7_to_nib = 4'h8;
      7'b0011000: seg7_to_nib = 4'h9;
      7'b0001000: seg7_to_nib = 4'hA;
      7'b0000011: seg7_to_nib = 4'hB;
      7'b1000110: seg7_to_nib = 4'hC;
      7'b0100001: seg7_to_nib = 4'hD;
      7'b0000110: seg7_to_nib = 4'hE;
      7'b0001110: seg7_to_nib = 4'hF;
      default:    seg7_to_nib = 4'hX;
    endcase
  endfunction

  function automatic [23:0] hex_from_hexes;
    // HEX0 is low nibble, HEX5 is high nibble
    hex_from_hexes = {
      seg7_to_nib(HEX5),
      seg7_to_nib(HEX4),
      seg7_to_nib(HEX3),
      seg7_to_nib(HEX2),
      seg7_to_nib(HEX1),
      seg7_to_nib(HEX0)
    };
  endfunction

  // ----------------------------
  // Expected ROM contents (from input_mem.mif)
  // - rows 0..7: matrix rows
  // - row 8: vector
  // ----------------------------
  logic [63:0] mem [0:8];
  initial begin
    mem[0] = 64'h0102030405060708;
    mem[1] = 64'h1112131415161718;
    mem[2] = 64'h2122232425262728;
    mem[3] = 64'h3132333435363738;
    mem[4] = 64'h4142434445464748;
    mem[5] = 64'h5152535455565758;
    mem[6] = 64'h6162636465666768;
    mem[7] = 64'h7172737475767778;
    mem[8] = 64'h8182838485868788;
  end

  // Extract byte k (0..7) MSB-first from a 64-bit word
  function automatic [7:0] byte_msb_first(input logic [63:0] w, input int k);
    // k=0 => [63:56], k=7 => [7:0]
    byte_msb_first = w[(7-k)*8 +: 8];
  endfunction

  function automatic [23:0] expected_lane(input int lane);
    int k;
    int unsigned sum;
    begin
      sum = 0;
      for (k = 0; k < 8; k++) begin
        sum += byte_msb_first(mem[lane], k) * byte_msb_first(mem[8], k);
      end
      expected_lane = sum[23:0];
    end
  endfunction

  // ----------------------------
  // Reset / stimulus
  // ----------------------------
  initial begin
    // defaults
    KEY = 4'hF;     // not pressed (KEY bits high)
    SW  = 10'h000;  // switches low

    // hold reset low (KEY0=0)
    KEY[0] = 1'b0;
    repeat (5) @(posedge CLOCK_50);
    KEY[0] = 1'b1;

    // enable HEX display
    SW[0] = 1'b1;

    // Wait for DONE = LEDR[2:0] == 3'b011 (state 3 = S_DONE)
    wait_for_done(200000); // timeout cycles

    $display("\n==== Reached DONE at time %0t ns ====", $time);

    // Check all 8 lanes by peeking internal c_out
    check_all_lanes();

    // Also show what HEX is currently showing for lane_sel=SW[3:1]
    $display("Current SW[3:1]=%0d, HEX shows 0x%06h",
             SW[3:1], hex_from_hexes());

    // Sweep lane select and print HEX values
    sweep_display();

    $display("\nTB PASSED.");
    $finish;
  end

  task automatic wait_for_done(input int max_cycles);
    int c;
    begin
      for (c = 0; c < max_cycles; c++) begin
        @(posedge CLOCK_50);
        // LEDR[2:0] is driven combinationally from internal state
        if (LEDR[2:0] == 3'b011) return;
      end
      $fatal(1, "TIMEOUT: never reached DONE (LEDR[2:0] != 011). Last LEDR=%b time=%0t", LEDR, $time);
    end
  endtask

  task automatic check_all_lanes;
    int i;
    logic [23:0] got, exp;
    begin
      for (i = 0; i < 8; i++) begin
        // Peek DUT internal output array (legal in SV TB)
        got = dut.c_out[i][23:0];
        exp = expected_lane(i);

        if (got !== exp) begin
          $display("FAIL lane %0d: got 0x%06h expected 0x%06h", i, got, exp);
          $fatal(1, "Mismatch on lane %0d", i);
        end else begin
          $display("OK   lane %0d: 0x%06h", i, got);
        end
      end

      // Bonus: explicitly show lane0 expected should be 12CC (0x0012CC)
      $display("Lane0 expected = 0x%06h (should end with 12CC)", expected_lane(0));
    end
  endtask

  task automatic sweep_display;
    int i;
    begin
      for (i = 0; i < 8; i++) begin
        SW[3:1] = i[2:0];
        // allow comb settle
        #1;
        $display("Display lane %0d -> HEX = 0x%06h (expected 0x%06h)",
                 i, hex_from_hexes(), expected_lane(i));
      end
    end
  endtask

  // Optional continuous debug: print key state transitions
  logic [2:0] last_state;
  initial last_state = 3'bxxx;
  always @(posedge CLOCK_50) begin
    if (LEDR[2:0] !== last_state) begin
      $display("t=%0t ns  state(LEDR[2:0]) changed: %b -> %b  LEDR=%b",
               $time, last_state, LEDR[2:0], LEDR);
      last_state <= LEDR[2:0];
    end
  end

endmodule


// =======================================================
// Behavioral stub for mem_wrapper used by Minilab1
// (so we don't need Altera ROM simulation libs)
// - Implements the same interface as your provided mem_wrapper
// - 10-cycle waitrequest delay then asserts readdatavalid for 1 cycle
// - Addresses 0..8 are valid
// =======================================================
module mem_wrapper (
    input  wire        clk,
    input  wire        reset_n,
    input  wire [31:0] address,
    input  wire        read,
    output reg  [63:0] readdata,
    output reg         readdatavalid,
    output reg         waitrequest
);
  // Local ROM contents
  reg [63:0] mem [0:8];
  initial begin
    mem[0] = 64'h0102030405060708;
    mem[1] = 64'h1112131415161718;
    mem[2] = 64'h2122232425262728;
    mem[3] = 64'h3132333435363738;
    mem[4] = 64'h4142434445464748;
    mem[5] = 64'h5152535455565758;
    mem[6] = 64'h6162636465666768;
    mem[7] = 64'h7172737475767778;
    mem[8] = 64'h8182838485868788;
  end

  // Simple delay FSM
  localparam IDLE    = 2'b00;
  localparam WAIT    = 2'b01;
  localparam RESPOND = 2'b10;

  reg [1:0] state;
  reg [3:0] delay_counter;
  reg [3:0] latched_addr;

  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      state         <= IDLE;
      readdata      <= 64'd0;
      readdatavalid <= 1'b0;
      waitrequest   <= 1'b0;
      delay_counter <= 4'd0;
      latched_addr  <= 4'd0;
    end else begin
      case (state)
        IDLE: begin
          readdatavalid <= 1'b0;
          waitrequest   <= 1'b0;
          if (read) begin
            latched_addr  <= address[3:0];
            delay_counter <= 4'd10;   // 10-cycle wait
            waitrequest   <= 1'b1;
            state         <= WAIT;
          end
        end

        WAIT: begin
          if (delay_counter != 0)
            delay_counter <= delay_counter - 1;
          else
            state <= RESPOND;
        end

        RESPOND: begin
          readdata      <= mem[latched_addr];
          readdatavalid <= 1'b1;
          waitrequest   <= 1'b0;
          state         <= IDLE;
        end

        default: state <= IDLE;
      endcase
    end
  end

endmodule
