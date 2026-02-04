//=======================================================
// controller.sv
//
// Part 1a "fetcher" / controller:
// - Avalon-MM MASTER talking to mem_wrapper (SLAVE)
// - Reads addr 0..7 (8 rows of A) and addr 8 (vector B)
// - Unpacks 64-bit read into 8 bytes
// - Fills FIFOs:
//     A FIFO per row: a_fifo_wren[row], a_fifo_wdata[row]
//     B FIFO:         b_fifo_wren,     b_fifo_wdata
//
// Notes:
// - mem_wrapper latches address when read is seen in IDLE.
// - mem_wrapper raises waitrequest during its delay, then pulses readdatavalid.
//=======================================================

module controller (
  input  logic clk,
  input  logic rst_n,

  // simple control
  input  logic start,
  output logic done,

  // -----------------------------
  // Avalon-MM master -> mem_wrapper slave
  // -----------------------------
  output logic [31:0] avm_address,
  output logic        avm_read,
  input  logic [63:0] avm_readdata,
  input  logic        avm_readdatavalid,
  input  logic        avm_waitrequest,

  // -----------------------------
  // FIFO fill outputs (to external FIFOs)
  // 8 A-fifos (one per row), 1 B-fifo
  // -----------------------------
  output logic [7:0]  a_fifo_wren,           // one-hot-ish (only one row writes at a time)
  output logic [7:0]  a_fifo_wdata [0:7],    // a_fifo_wdata[row] is the byte for that row

  output logic        b_fifo_wren,
  output logic [7:0]  b_fifo_wdata,

  // -----------------------------
  // Debug outputs (so TB can print)
  // -----------------------------
  output logic [3:0]  dbg_state,
  output logic [3:0]  dbg_addr,      // current ROM address being requested (0..8)
  output logic [2:0]  dbg_row,       // 0..7
  output logic [2:0]  dbg_byte       // 0..7
);

  // ----------------------------------------------------
  // State machine encoding (simple and TB-friendly)
  // ----------------------------------------------------
  localparam logic [3:0]
    S_IDLE       = 4'd0,
    S_REQ        = 4'd1,
    S_WAIT_VALID = 4'd2,
    S_UNPACK     = 4'd3,
    S_NEXT       = 4'd4,
    S_DONE       = 4'd5;

  logic [3:0] state, state_n;

  // what we are currently fetching:
  // 0 = matrix rows (0..7), 1 = vector (addr 8)
  logic fetching_vec;

  logic [2:0] row_idx;     // 0..7
  logic [2:0] byte_idx;    // 0..7
  logic [3:0] rom_addr;    // 0..8 (fits in 4 bits)
  logic [63:0] word64;     // latched read data

  // ----------------------------------------------------
  // Helper: extract byte i from 64-bit word
  // ----------------------------------------------------
  function automatic logic [7:0] get_byte(input logic [63:0] w, input logic [2:0] i);
    // part-select: start bit = 8*i, width 8
    get_byte = w[8*i +: 8];
  endfunction

  // ----------------------------------------------------
  // Default outputs
  // ----------------------------------------------------
  always_comb begin
    // Avalon defaults
    avm_address = 32'd0;
    avm_read    = 1'b0;

    // FIFO defaults
    a_fifo_wren = 8'b0;
    for (int r = 0; r < 8; r++) begin
      a_fifo_wdata[r] = 8'h00;
    end
    b_fifo_wren  = 1'b0;
    b_fifo_wdata = 8'h00;

    // done default
    done = 1'b0;

    // debug
    dbg_state = state;
    dbg_addr  = rom_addr;
    dbg_row   = row_idx;
    dbg_byte  = byte_idx;

    // drive Avalon during request state
    if (state == S_REQ) begin
      avm_address = {28'd0, rom_addr}; // lower bits used by mem_wrapper
      // Only assert read if slave isn't stalling right now
      // (per Avalon, master must wait for waitrequest to drop)
      avm_read    = ~avm_waitrequest;
    end

    // During UNPACK: push one byte per cycle into the correct FIFO
    if (state == S_UNPACK) begin
      if (!fetching_vec) begin
        // writing matrix row 'row_idx' into its FIFO
        a_fifo_wren[row_idx]   = 1'b1;
        a_fifo_wdata[row_idx]  = get_byte(word64, byte_idx);
      end else begin
        // writing vector bytes into B FIFO
        b_fifo_wren  = 1'b1;
        b_fifo_wdata = get_byte(word64, byte_idx);
      end
    end

    if (state == S_DONE) begin
      done = 1'b1;
    end
  end

  // ----------------------------------------------------
  // Next-state logic
  // ----------------------------------------------------
  always_comb begin
    state_n = state;

    unique case (state)
      S_IDLE: begin
        if (start) state_n = S_REQ;
      end

      // Issue a read. When avm_waitrequest is low, we assert avm_read (in comb),
      // then move on (mem_wrapper will latch address and raise waitrequest internally).
      S_REQ: begin
        if (!avm_waitrequest) begin
          // we are presenting a valid read this cycle
          state_n = S_WAIT_VALID;
        end
      end

      // Wait until slave says data is valid
      S_WAIT_VALID: begin
        if (avm_readdatavalid) begin
          state_n = S_UNPACK;
        end
      end

      // Unpack 8 bytes across 8 cycles into FIFOs
      S_UNPACK: begin
        if (byte_idx == 3'd7) begin
          state_n = S_NEXT;
        end
      end

      // Advance to next address/phase
      S_NEXT: begin
        // If we just finished matrix row 7, go fetch vector (addr 8)
        // If we just finished vector, we're done.
        if (!fetching_vec) begin
          if (row_idx == 3'd7) begin
            state_n = S_REQ;
          end else begin
            state_n = S_REQ;
          end
        end else begin
          state_n = S_DONE;
        end
      end

      S_DONE: begin
        // hold done high until start deasserts (simple handshake)
        if (!start) state_n = S_IDLE;
      end

      default: state_n = S_IDLE;
    endcase
  end

  // ----------------------------------------------------
  // State registers + counters + address selection
  // ----------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= S_IDLE;
      fetching_vec <= 1'b0;
      row_idx      <= 3'd0;
      byte_idx     <= 3'd0;
      rom_addr     <= 4'd0;
      word64       <= 64'd0;
    end else begin
      state <= state_n;

      // latch read data when valid
      if (state == S_WAIT_VALID && avm_readdatavalid) begin
        word64 <= avm_readdata;
        byte_idx <= 3'd0; // start unpack at byte 0
      end

      // During unpack, advance byte index each cycle
      if (state == S_UNPACK) begin
        byte_idx <= byte_idx + 3'd1;
      end

      // Address/phase management in S_NEXT
      if (state == S_NEXT) begin
        byte_idx <= 3'd0;

        if (!fetching_vec) begin
          // We were fetching matrix rows
          if (row_idx == 3'd7) begin
            // finished last row -> next is vector at addr 8
            fetching_vec <= 1'b1;
            rom_addr     <= 4'd8;
          end else begin
            // next matrix row
            row_idx  <= row_idx + 3'd1;
            rom_addr <= rom_addr + 4'd1;
          end
        end else begin
          // finished vector -> nothing else
        end
      end

      // Initialize when start triggers from IDLE
      if (state == S_IDLE && start) begin
        fetching_vec <= 1'b0;
        row_idx      <= 3'd0;
        byte_idx     <= 3'd0;
        rom_addr     <= 4'd0;
      end
    end
  end

endmodule
