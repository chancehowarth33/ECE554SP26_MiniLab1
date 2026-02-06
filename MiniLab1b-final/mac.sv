module MAC #
(
  parameter DATA_WIDTH = 8
)
(
  input  logic clk,
  input  logic rst_n,
  input  logic En,
  input  logic Clr,
  input  logic [DATA_WIDTH-1:0] Ain,
  input  logic [DATA_WIDTH-1:0] Bin,
  output logic [DATA_WIDTH*3-1:0] Cout
);

  // accumulator (24-bit when DATA_WIDTH=8)
  logic [DATA_WIDTH*3-1:0] accumulator;

  // Stage 1: registered multiplier output (16-bit when DATA_WIDTH=8)
  logic [DATA_WIDTH*2-1:0] mult_reg;

  // Pipeline control to align with mult_reg (1-cycle delay)
  logic En_d1, Clr_d1;

  // -------------------------
  // Stage 1: multiply + pipe control
  // -------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mult_reg <= '0;
      En_d1    <= 1'b0;
      Clr_d1   <= 1'b0;
    end else begin
      mult_reg <= Ain * Bin;  // multiplication isolated in its own stage
      En_d1    <= En;
      Clr_d1   <= Clr;
    end
  end

  // -------------------------
  // Stage 2: accumulate (uses registered product)
  // -------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      accumulator <= '0;
    end
    else if (Clr_d1) begin
      accumulator <= '0;
    end
    else if (En_d1) begin
      // zero-extend mult_reg to accumulator width before add
      accumulator <= accumulator + {{(DATA_WIDTH){1'b0}}, mult_reg};
    end
  end

  assign Cout = accumulator;

endmodule
