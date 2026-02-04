`timescale 1ns/1ps

module tb_part1a;

  localparam int DATA_WIDTH = 8;
  localparam int DEPTH      = 8;

  // -----------------------------
  // clock / reset / start
  // -----------------------------
  logic clk;
  logic rst_n;
  logic start;

  // -----------------------------
  // Avalon-MM wires (controller <-> mem_wrapper)
  // -----------------------------
  logic [31:0] avm_address;
  logic        avm_read;
  logic [63:0] avm_readdata;
  logic        avm_readdatavalid;
  logic        avm_waitrequest;

  // -----------------------------
  // FIFO bank signals
  // -----------------------------
  logic [7:0] a_full, a_empty;
  logic [7:0] a_wren;
  logic [DATA_WIDTH-1:0] a_wdata [0:7];
  logic [7:0] a_rden;
  logic [DATA_WIDTH-1:0] a_rdata [0:7];

  logic       b_full, b_empty;
  logic       b_wren;
  logic [DATA_WIDTH-1:0] b_wdata;
  logic       b_rden;
  logic [DATA_WIDTH-1:0] b_rdata;

  // -----------------------------
  // controller status/debug
  // -----------------------------
  logic       load_done;
  logic [3:0] dbg_state, dbg_addr;
  logic [2:0] dbg_row, dbg_byte;

  // -----------------------------
  // MAC8 interface
  // -----------------------------
  logic En_in, Clr_in;
  logic [DATA_WIDTH-1:0] b_in;
  logic [DATA_WIDTH-1:0] a_in [0:7];
  logic [DATA_WIDTH*3-1:0] c_out [0:7];

  // =====================================================
  // Clock generation: 50 MHz -> 20 ns period
  // =====================================================
  initial clk = 1'b0;
  always #10 clk = ~clk;

  // =====================================================
  // DUT: mem_wrapper (slave)
  // =====================================================
  mem_wrapper u_mem (
    .clk          (clk),
    .reset_n      (rst_n),
    .address      (avm_address),
    .read         (avm_read),
    .readdata     (avm_readdata),
    .readdatavalid(avm_readdatavalid),
    .waitrequest  (avm_waitrequest)
  );

  // =====================================================
  // DUT: controller (master, fills FIFOs)
  // =====================================================
  controller u_ctrl (
    .clk              (clk),
    .rst_n            (rst_n),
    .start            (start),
    .done             (load_done),

    .avm_address      (avm_address),
    .avm_read         (avm_read),
    .avm_readdata     (avm_readdata),
    .avm_readdatavalid(avm_readdatavalid),
    .avm_waitrequest  (avm_waitrequest),

    .a_fifo_wren      (a_wren),
    .a_fifo_wdata     (a_wdata),

    .b_fifo_wren      (b_wren),
    .b_fifo_wdata     (b_wdata),

    .dbg_state        (dbg_state),
    .dbg_addr         (dbg_addr),
    .dbg_row          (dbg_row),
    .dbg_byte         (dbg_byte)
  );

  // =====================================================
  // 8x A FIFOs
  // =====================================================
  genvar r;
  generate
    for (r = 0; r < 8; r = r + 1) begin : A_FIFOS
      FIFO #(
        .DEPTH(DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
      ) u_fifo_a (
        .clk   (clk),
        .rst_n (rst_n),
        .rden  (a_rden[r]),
        .wren  (a_wren[r]),
        .i_data(a_wdata[r]),
        .o_data(a_rdata[r]),
        .full  (a_full[r]),
        .empty (a_empty[r])
      );
    end
  endgenerate

  // =====================================================
  // 1x B FIFO
  // =====================================================
  FIFO #(
    .DEPTH(DEPTH),
    .DATA_WIDTH(DATA_WIDTH)
  ) u_fifo_b (
    .clk   (clk),
    .rst_n (rst_n),
    .rden  (b_rden),
    .wren  (b_wren),
    .i_data(b_wdata),
    .o_data(b_rdata),
    .full  (b_full),
    .empty (b_empty)
  );

  // =====================================================
  // MAC8 wiring (a_in <= a_rdata)
  // =====================================================
  always_comb begin
    for (int i = 0; i < 8; i++) begin
      a_in[i] = a_rdata[i];
    end
  end

  assign b_in = b_rdata;

  MAC8 #(
    .DATA_WIDTH(DATA_WIDTH)
  ) u_mac8 (
    .clk   (clk),
    .rst_n (rst_n),
    .En_in (En_in),
    .Clr_in(Clr_in),
    .b_in  (b_in),
    .a_in  (a_in),
    .c_out (c_out)
  );

  // =====================================================
  // Pretty printing helpers
  // =====================================================
  task automatic print_status;
    $display("[%0t] start=%0b done=%0b  AV: addr=%0d read=%0b wait=%0b rvalid=%0b rdata=%h  dbg:st=%0d addr=%0d row=%0d byte=%0d  Afull=%b Bfull=%0b Aempty=%b Bempty=%0b  Arden=%b Brden=%0b En=%0b Clr=%0b",
      $time, start, load_done,
      avm_address[3:0], avm_read, avm_waitrequest, avm_readdatavalid, avm_readdata,
      dbg_state, dbg_addr, dbg_row, dbg_byte,
      a_full, b_full, a_empty, b_empty,
      a_rden, b_rden, En_in, Clr_in
    );
  endtask

  // Print every cycle (comment out if too chatty)
  always_ff @(posedge clk) begin
    if (rst_n) print_status();
  end

  // =====================================================
  // Timeout helper (prevents hanging forever)
  // =====================================================
  task automatic wait_load_done_or_timeout;
    fork
      begin
        wait (load_done == 1'b1);
      end
      begin
        repeat (5000) @(posedge clk);
        $fatal(1, "TIMEOUT: controller never asserted load_done");
      end
    join_any
    disable fork;
  endtask

  // =====================================================
  // Test sequence
  // =====================================================
  initial begin
    // init
    rst_n = 1'b0;
    start = 1'b0;

    // FIFO reads disabled initially
    a_rden = 8'h00;
    b_rden = 1'b0;

    // MAC8 controls
    En_in  = 1'b0;
    Clr_in = 1'b0;

    // reset for a few cycles
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);

    // start controller
    start = 1'b1;

    // wait for controller to finish filling FIFOs (with timeout)
    $display("---- Waiting for controller to fill FIFOs ----");
    wait_load_done_or_timeout();
    $display("---- Controller done: FIFOs filled ----");

    // -------------------------------------------------
    // EXEC phase (compile smoke test, not final-correct math):
    // - FIFO o_data updates on posedge when rden is asserted.
    // - do 1-cycle preload with En=0, then 8 cycles En=1.
    // -------------------------------------------------
    $display("---- EXEC: preload read (En=0) ----");
    Clr_in = 1'b1;
    a_rden = 8'hFF;
    b_rden = 1'b1;
    En_in  = 1'b0;
    @(posedge clk);

    Clr_in = 1'b0;
    $display("---- EXEC: 8 MAC cycles (En=1) ----");
    repeat (8) begin
      a_rden = 8'hFF;
      b_rden = 1'b1;
      En_in  = 1'b1;
      @(posedge clk);
    end

    // stop reading/enabling
    a_rden = 8'h00;
    b_rden = 1'b0;
    En_in  = 1'b0;

    // let pipeline settle a couple cycles
    repeat (3) @(posedge clk);

    // Print final results
    $display("---- FINAL c_out results ----");
    for (int i = 0; i < 8; i++) begin
      $display("c_out[%0d] = 0x%0h", i, c_out[i]);
    end

    $display("---- TEST DONE ----");
    $finish;
  end

endmodule
