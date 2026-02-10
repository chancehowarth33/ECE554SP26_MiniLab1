//=======================================================
// Minilab 1 Top-Level (FPGA)
// - Uses controller module to fetch 8x8 matrix + vector from mem_wrapper
// - Controller fills 8 A FIFOs (one per row) and 1 B FIFO
// - After load completes: runs MAC8 pipeline
// - LEDs show FSM state; HEX shows selected lane's 24-bit result
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

  // ----------------------------
  // Reset (active-low, KEY0)
  // ----------------------------
  logic rst_n;
  assign rst_n = KEY[0];

  // ----------------------------
  // FIFO signals
  // ----------------------------
  // Write side (driven by controller)
  logic [7:0]  a_fifo_wren_vec;         // packed bit-vector from controller
  logic [7:0]  a_fifo_wdata [0:N-1];    // byte per row from controller
  logic        b_fifo_wren;
  logic [7:0]  b_fifo_wdata;

  // Read side (driven by top FSM during EXEC/DRAIN)
  logic                   a_rden  [0:N-1];
  logic [DATA_WIDTH-1:0]  a_rdata [0:N-1];
  logic                   a_full  [0:N-1];
  logic                   a_empty [0:N-1];

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
        .wren  (a_fifo_wren_vec[gi]),       // from controller
        .i_data(a_fifo_wdata[gi]),          // from controller
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
    .wren  (b_fifo_wren),                   // from controller
    .i_data(b_fifo_wdata),                  // from controller
    .o_data(b_rdata),
    .full  (b_full),
    .empty (b_empty)
  );

  // ----------------------------
  // Avalon-MM signals (controller <-> mem_wrapper)
  // ----------------------------
  logic [31:0] avm_address;
  logic        avm_read;
  logic [63:0] avm_readdata;
  logic        avm_readdatavalid;
  logic        avm_waitrequest;

  // ----------------------------
  // Controller (fetches matrix + vector, fills FIFOs)
  // ----------------------------
  logic ctrl_start, ctrl_done;
  logic [3:0] ctrl_dbg_state;
  logic [3:0] ctrl_dbg_addr;
  logic [2:0] ctrl_dbg_row;
  logic [2:0] ctrl_dbg_byte;

  controller u_ctrl (
    .clk              (CLOCK_50),
    .rst_n            (rst_n),
    .start            (ctrl_start),
    .done             (ctrl_done),

    // Avalon-MM master
    .avm_address      (avm_address),
    .avm_read         (avm_read),
    .avm_readdata     (avm_readdata),
    .avm_readdatavalid(avm_readdatavalid),
    .avm_waitrequest  (avm_waitrequest),

    // FIFO fill outputs
    .a_fifo_wren      (a_fifo_wren_vec),
    .a_fifo_wdata     (a_fifo_wdata),
    .b_fifo_wren      (b_fifo_wren),
    .b_fifo_wdata     (b_fifo_wdata),

    // Debug
    .dbg_state        (ctrl_dbg_state),
    .dbg_addr         (ctrl_dbg_addr),
    .dbg_row          (ctrl_dbg_row),
    .dbg_byte         (ctrl_dbg_byte)
  );

  // ----------------------------
  // Memory wrapper (Avalon-MM slave)
  // ----------------------------
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
  // Top-level FSM
  // ----------------------------
  typedef enum logic [2:0] {
    S_LOAD  = 3'd0,   // controller fetching data into FIFOs
    S_EXEC  = 3'd1,   // streaming B through MAC8 pipeline
    S_DRAIN = 3'd2,   // wait for pipeline to flush
    S_DONE  = 3'd3    // display results
  } state_t;

  state_t state, state_n;

  logic exec_started;
  logic last_b_sent;
  logic [$clog2(N+2)-1:0] drain_cnt;

  // ----------------------------
  // Combinational: next-state + datapath control
  // ----------------------------
  always_comb begin
    // Defaults
    for (int ii = 0; ii < N; ii++) begin
      a_rden[ii] = 1'b0;
    end
    b_rden     = 1'b0;
    En_in      = 1'b0;
    Clr_in     = 1'b0;
    ctrl_start = 1'b0;
    state_n    = state;

    case (state)
      // --------------------
      // LOAD: tell controller to fetch, wait for done
      // --------------------
      S_LOAD: begin
        ctrl_start = 1'b1;
        if (ctrl_done) begin
          state_n = S_EXEC;
        end
      end

      // --------------------
      // EXEC: stream B into MAC8; pop A lanes via pipelined enables
      // --------------------
      // --------------------
      // EXEC: stream B into MAC8; pop A lanes via pipelined enables
      // --------------------
      S_EXEC: begin
        if (!exec_started) begin
          Clr_in = 1'b1;   // clear accumulators once at start
          // FIX: Prefetch first B value during Clr cycle (FIFO latency = 1)
          if (!b_empty) b_rden = 1'b1;
        end else if (!b_empty) begin
          En_in  = 1'b1;
          b_rden = 1'b1;
        end else if (!last_b_sent) begin
          // One extra En cycle: last B value is in b_rdata but
          // hasn't been latched into b_pipe[0] yet
          En_in = 1'b1;
        end else begin
          state_n = S_DRAIN;
        end

        // Pop each A lane aligned to its pipelined enable
        // FIX: Reads must happen 1 cycle *before* enabled use in MAC
        // Lane 0 needs read when En_in is high (enters pipe)
        // Lane k needs read when en_out[k-1] (stage k-1) is high
        for (int ii = 0; ii < N; ii++) begin
          if (ii == 0) begin
             if (En_in && !a_empty[ii]) a_rden[ii] = 1'b1;
          end else begin
             if (en_out[ii-1] && !a_empty[ii]) a_rden[ii] = 1'b1;
          end
        end
      end

      // --------------------
      // DRAIN: wait for pipeline to finish
      // --------------------
      S_DRAIN: begin
        // Keep draining A FIFOs if needed (pipeline tail)
        for (int ii = 0; ii < N; ii++) begin
          if (ii == 0) begin
             // No new En_in during DRAIN, so lane 0 doesn't read
          end else begin
             if (en_out[ii-1] && !a_empty[ii]) a_rden[ii] = 1'b1;
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
        // no-op, results held in MAC accumulators
      end

      default: state_n = S_LOAD;
    endcase
  end

  // ----------------------------
  // Sequential: state register + counters
  // ----------------------------
  always_ff @(posedge CLOCK_50 or negedge rst_n) begin
    if (!rst_n) begin
      state        <= S_LOAD;
      exec_started <= 1'b0;
      last_b_sent  <= 1'b0;
      drain_cnt    <= '0;
    end else begin
      state <= state_n;

      // Track EXEC entry for one-cycle Clr pulse
      if (state != S_EXEC && state_n == S_EXEC) begin
        exec_started <= 1'b0;
        last_b_sent  <= 1'b0;
      end else if (state == S_EXEC) begin
        exec_started <= 1'b1;
        if (b_empty && !last_b_sent)
          last_b_sent <= 1'b1;
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
  // LEDs: show state + status
  // LEDR[2:0] = top-level FSM state
  // LEDR[5:3] = controller sub-state (debug)
  // LEDR[7]   = MAC pipeline active (any en_out)
  // LEDR[8]   = b_empty
  // LEDR[9]   = controller done
  // ----------------------------
  logic any_en;
  always_comb begin
    any_en = 1'b0;
    for (int ii = 0; ii < N; ii++) any_en |= en_out[ii];
  end

  always_comb begin
    LEDR      = '0;
    LEDR[2:0] = state;
    LEDR[5:3] = ctrl_dbg_state[2:0];
    LEDR[7]   = any_en;
    LEDR[8]   = b_empty;
    LEDR[9]   = ctrl_done;
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