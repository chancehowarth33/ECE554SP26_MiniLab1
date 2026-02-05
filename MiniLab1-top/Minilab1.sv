//=======================================================
// Minilab 1 Top-Level (FPGA)
// - Reads 8x 64-bit rows from mem_wrapper (addr 0..7) -> fills 8 A FIFOs (8 bytes each)
// - Reads 1x 64-bit vector from mem_wrapper (addr 8)   -> fills B FIFO (8 bytes)
// - After all full: runs MAC8
// - LEDs show state; HEX shows selected lane's 24-bit result
//=======================================================

module Minilab1
(
  //////////// CLOCK //////////
  input                   CLOCK2_50,
  input                   CLOCK3_50,
  input                   CLOCK4_50,
  input                   CLOCK_50,

  //////////// SEG7 //////////
  output  logic   [6:0]   HEX0,
  output  logic   [6:0]   HEX1,
  output  logic   [6:0]   HEX2,
  output  logic   [6:0]   HEX3,
  output  logic   [6:0]   HEX4,
  output  logic   [6:0]   HEX5,

  //////////// LED //////////
  output  logic   [9:0]   LEDR,

  //////////// KEY //////////
  input           [3:0]   KEY,

  //////////// SW //////////
  input           [9:0]   SW
);

  // ----------------------------
  // Parameters
  // ----------------------------
  localparam int DATA_WIDTH = 8;
  localparam int N          = 8;
  localparam int DEPTH      = 8;

  // mem layout (edit if your mif uses different placement)
  localparam int MAT_BASE_ADDR = 0; // rows 0..7
  localparam int VEC_ADDR      = 8; // vector word

  // ----------------------------
  // Reset
  // ----------------------------
  logic rst_n;
  assign rst_n = KEY[0]; // KEY0 low resets (same convention as your Minilab0)

  // ----------------------------
  // FIFO signals
  // ----------------------------
  logic [DATA_WIDTH-1:0]   a_wdata [0:N-1];
  logic                   a_wren  [0:N-1];
  logic                   a_rden  [0:N-1];
  logic [DATA_WIDTH-1:0]  a_rdata [0:N-1];
  logic                   a_full  [0:N-1];
  logic                   a_empty [0:N-1];

  logic [DATA_WIDTH-1:0]  b_wdata;
  logic                   b_wren;
  logic                   b_rden;
  logic [DATA_WIDTH-1:0]  b_rdata;
  logic                   b_full, b_empty;

  // ----------------------------
  // Instantiate 8 A FIFOs
  // ----------------------------
  genvar gi;
  generate
    for (gi = 0; gi < N; gi++) begin : GEN_A_FIFOS
      FIFO #(
        .DEPTH(DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
      ) u_fifo_a (
        .clk   (CLOCK_50),
        .rst_n (rst_n),
        .rden  (a_rden[gi]),
        .wren  (a_wren[gi]),
        .i_data(a_wdata[gi]),
        .o_data(a_rdata[gi]),
        .full  (a_full[gi]),
        .empty (a_empty[gi])
      );
    end
  endgenerate

  // ----------------------------
  // Instantiate B FIFO
  // ----------------------------
  FIFO #(
    .DEPTH(DEPTH),
    .DATA_WIDTH(DATA_WIDTH)
  ) u_fifo_b (
    .clk   (CLOCK_50),
    .rst_n (rst_n),
    .rden  (b_rden),
    .wren  (b_wren),
    .i_data(b_wdata),
    .o_data(b_rdata),
    .full  (b_full),
    .empty (b_empty)
  );

  // ----------------------------
  // Avalon-MM to mem_wrapper
  // ----------------------------
  logic [31:0] avm_address;
  logic        avm_read;
  logic [63:0] avm_readdata;
  logic        avm_readdatavalid;
  logic        avm_waitrequest;

  mem_wrapper u_mem (
    .clk          (CLOCK_50),
    .reset_n      (rst_n),
    .address      (avm_address),
    .read         (avm_read),
    .readdata     (avm_readdata),
    .readdatavalid(avm_readdatavalid),
    .waitrequest  (avm_waitrequest)
  );

  // ----------------------------
  // MAC8
  // ----------------------------
  logic                    En_in, Clr_in;
  logic [DATA_WIDTH*3-1:0] c_out [0:N-1];
  logic [N-1:0]            en_out, clr_out;
  logic [DATA_WIDTH-1:0]   b_pipe_dbg [0:N-1];

  MAC8 #(
    .DATA_WIDTH(DATA_WIDTH),
    .N(N)
  ) u_mac8 (
    .clk    (CLOCK_50),
    .rst_n  (rst_n),
    .En_in  (En_in),
    .Clr_in (Clr_in),
    .b_in   (b_rdata),
    .a_in   (a_rdata),
    .c_out  (c_out),
    .en_out (en_out),
    .clr_out(clr_out),
    .b_out  (b_pipe_dbg)
  );

  // ----------------------------
  // Helpers: full/empty reduction
  // ----------------------------
  logic all_a_full, all_a_empty;
  logic all_full;

  integer ii;
  always_comb begin
    all_a_full  = 1'b1;
    all_a_empty = 1'b1;
    for (ii = 0; ii < N; ii++) begin
      all_a_full  &= a_full[ii];
      all_a_empty &= a_empty[ii];
    end
  end

  assign all_full = all_a_full & b_full;

  // ----------------------------
  // Top FSM
  // ----------------------------
  typedef enum logic [2:0] {
    S_MEM_REQ   = 3'd0, // issue read for current word
    S_MEM_WAIT  = 3'd1, // wait for readdatavalid
    S_UNPACK    = 3'd2, // write 8 bytes into the right FIFO
    S_EXEC      = 3'd3, // run MAC8 streaming B
    S_DRAIN     = 3'd4, // allow pipeline to finish
    S_DONE      = 3'd5  // display
  } state_t;

  state_t state, state_n;

  // Which word are we reading? 0..7 rows, then vector at 8
  logic [3:0] word_index;     // 0..8
  logic       is_vector_word; // 0 => matrix row, 1 => vector

  // Latched 64-bit word and byte index for UNPACK
  logic [63:0] word_data;
  logic [2:0]  byte_index; // 0..7

  // Exec control
  logic exec_started;
  logic [$clog2(N+2)-1:0] drain_cnt;

  // Default controls
  always_comb begin
    // defaults for FIFO write/read
    for (ii = 0; ii < N; ii++) begin
      a_wren[ii]  = 1'b0;
      a_wdata[ii] = '0;
      a_rden[ii]  = 1'b0;
    end
    b_wren  = 1'b0;
    b_wdata = '0;
    b_rden  = 1'b0;

    // defaults for MAC8
    En_in  = 1'b0;
    Clr_in = 1'b0;

    // defaults for Avalon read
    avm_address = 32'd0;
    avm_read    = 1'b0;

    state_n = state;

    case (state)
      // --------------------
      // Request next 64-bit word from mem_wrapper
      // --------------------
      S_MEM_REQ: begin
        // Determine address
        is_vector_word = (word_index == VEC_ADDR[3:0]);
        avm_address    = {28'd0, word_index}; // small address, mem_wrapper truncates anyway

        // Only assert read if wrapper not busy
        if (!avm_waitrequest) begin
          avm_read  = 1'b1;     // one-cycle pulse
          state_n   = S_MEM_WAIT;
        end
      end

      // --------------------
      // Wait for response (readdatavalid)
      // --------------------
      S_MEM_WAIT: begin
        if (avm_readdatavalid) begin
          state_n = S_UNPACK;
        end
      end

      // --------------------
      // Unpack 64-bit word into 8 bytes, write one byte per cycle
      // - For matrix rows: write into a_fifo[row]
      // - For vector word: write into b_fifo
      // --------------------
      S_UNPACK: begin
        if (word_index < 8) begin
          // matrix row -> A FIFO[word_index]
          if (!a_full[word_index]) begin
            a_wren[word_index]  = 1'b1;
            a_wdata[word_index] = word_data[(7-byte_index)*8 +: 8];
          end
        end else begin
          // vector word -> B FIFO
          if (!b_full) begin
            b_wren  = 1'b1;
            b_wdata = word_data[(7-byte_index)*8 +: 8];
          end
        end

        // Next state handled in sequential block when byte_index reaches 7
      end

      // --------------------
      // EXEC: stream B into stage0; pop A lanes aligned to en_out
      // --------------------
      S_EXEC: begin
        if (!exec_started) begin
          Clr_in = 1'b1; // clear accumulators once at start
        end

        // While B has data, issue En and pop B each cycle
        if (!b_empty) begin
          En_in  = 1'b1;
          b_rden = 1'b1;
        end else begin
          state_n = S_DRAIN;
        end

        // Pop each A lane aligned to its pipelined enable
        for (ii = 0; ii < N; ii++) begin
          if (en_out[ii] && !a_empty[ii]) begin
            a_rden[ii] = 1'b1;
          end
        end
      end

      // --------------------
      // DRAIN: wait ~N cycles for en/b pipeline to finish
      // --------------------
      S_DRAIN: begin
        for (ii = 0; ii < N; ii++) begin
          if (en_out[ii] && !a_empty[ii]) begin
            a_rden[ii] = 1'b1;
          end
        end

        if (drain_cnt == 0) begin
          state_n = S_DONE;
        end
      end

      // --------------------
      // DONE: hold results for display
      // --------------------
      S_DONE: begin
        // no-op
      end

      default: state_n = S_MEM_REQ;
    endcase
  end

  // Sequential: manage state/counters/word capture
  always_ff @(posedge CLOCK_50 or negedge rst_n) begin
    if (!rst_n) begin
      state        <= S_MEM_REQ;
      word_index   <= 4'd0;
      word_data    <= 64'd0;
      byte_index   <= 3'd0;
      exec_started <= 1'b0;
      drain_cnt    <= '0;
    end else begin
      state <= state_n;

      // Capture the 64-bit word on valid
      if (state == S_MEM_WAIT && avm_readdatavalid) begin
        word_data  <= avm_readdata;
        byte_index <= 3'd0;
      end

      // Advance byte_index during UNPACK (one byte written per cycle)
      if (state == S_UNPACK) begin
        if (byte_index == 3'd7) begin
          // finished this 64-bit word
          if (word_index == VEC_ADDR[3:0]) begin
            // finished vector -> move to EXEC
            state <= S_EXEC;
          end else begin
            // next word
            word_index <= word_index + 1;
            state      <= S_MEM_REQ;
          end
          byte_index <= 3'd0;
        end else begin
          byte_index <= byte_index + 1;
        end
      end

      // If we've finished row7, next should be vector word at addr 8
      if (state == S_MEM_REQ) begin
        // after word_index reaches 8 naturally, we read vector
        // nothing extra needed
      end

      // Track EXEC entry
      if (state != S_EXEC && state_n == S_EXEC) begin
        exec_started <= 1'b0;
      end else if (state == S_EXEC) begin
        exec_started <= 1'b1;
      end

      // Setup drain counter on DRAIN entry
      if (state != S_DRAIN && state_n == S_DRAIN) begin
        drain_cnt <= N[$clog2(N+2)-1:0];
      end else if (state == S_DRAIN && drain_cnt != 0) begin
        drain_cnt <= drain_cnt - 1;
      end
    end
  end

  // ----------------------------
  // LEDs: show state + a few useful status bits
  // LEDR[2:0] = state
  // LEDR[9]   = all_full
  // LEDR[8]   = b_empty
  // LEDR[7]   = pipeline busy (any en_out)
  // ----------------------------
  logic any_en;
  always_comb begin
    any_en = 1'b0;
    for (ii = 0; ii < N; ii++) any_en |= en_out[ii];
  end

  always_comb begin
    LEDR      = '0;
    LEDR[2:0] = state;
    LEDR[9]   = all_full;
    LEDR[8]   = b_empty;
    LEDR[7]   = any_en;
  end

  // ----------------------------
  // 7-seg display
  // - SW[0] enables display
  // - SW[3:1] selects lane 0..7
  // - Displays 24-bit c_out[lane] across HEX5..HEX0
  // ----------------------------
  function automatic logic [6:0] hex7(input logic [3:0] nib);
    case (nib)
      4'h0: hex7 = 7'b1000000;
      4'h1: hex7 = 7'b1111001;
      4'h2: hex7 = 7'b0100100;
      4'h3: hex7 = 7'b0110000;
      4'h4: hex7 = 7'b0011001;
      4'h5: hex7 = 7'b0010010;
      4'h6: hex7 = 7'b0000010;
      4'h7: hex7 = 7'b1111000;
      4'h8: hex7 = 7'b0000000;
      4'h9: hex7 = 7'b0011000;
      4'hA: hex7 = 7'b0001000;
      4'hB: hex7 = 7'b0000011;
      4'hC: hex7 = 7'b1000110;
      4'hD: hex7 = 7'b0100001;
      4'hE: hex7 = 7'b0000110;
      4'hF: hex7 = 7'b0001110;
      default: hex7 = 7'b1111111;
    endcase
  endfunction

  logic [2:0]  lane_sel;
  logic [23:0] lane_c;

  assign lane_sel = SW[3:1];

  always_comb begin
    lane_c = 24'h0;
    lane_c = c_out[lane_sel];
  end

  always_comb begin
    if (SW[0] && (state == S_DONE)) begin
      HEX0 = hex7(lane_c[ 3: 0]);
      HEX1 = hex7(lane_c[ 7: 4]);
      HEX2 = hex7(lane_c[11: 8]);
      HEX3 = hex7(lane_c[15:12]);
      HEX4 = hex7(lane_c[19:16]);
      HEX5 = hex7(lane_c[23:20]);
    end else begin
      HEX0 = 7'b1111111;
      HEX1 = 7'b1111111;
      HEX2 = 7'b1111111;
      HEX3 = 7'b1111111;
      HEX4 = 7'b1111111;
      HEX5 = 7'b1111111;
    end
  end

endmodule
