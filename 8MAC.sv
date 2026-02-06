module MAC8 #(
  parameter int DATA_WIDTH = 8,
  parameter int N          = 8
)(
  input  logic clk,
  input  logic rst_n,

  input  logic En_in,
  input  logic Clr_in,
  input  logic [DATA_WIDTH-1:0] b_in,

  input  logic [DATA_WIDTH-1:0] a_in [0:N-1],

  output logic [DATA_WIDTH*3-1:0] c_out [0:N-1],

  // Keep these ports for compatibility/debug
  output logic [N-1:0] en_out,
  output logic [N-1:0] clr_out,
  output logic [DATA_WIDTH-1:0] b_out [0:N-1]
);

  // For compatibility: replicate controls/data to all lanes
  always_comb begin
    en_out  = {N{En_in}};
    clr_out = {N{Clr_in}};
    for (int i = 0; i < N; i++) begin
      b_out[i] = b_in;
    end
  end

  genvar i;
  generate
    for (i = 0; i < N; i++) begin : MACS
      MAC #(.DATA_WIDTH(DATA_WIDTH)) u_mac (
        .clk  (clk),
        .rst_n(rst_n),
        .En   (En_in),     // broadcast
        .Clr  (Clr_in),    // broadcast
        .Ain  (a_in[i]),
        .Bin  (b_in),      // broadcast
        .Cout (c_out[i])
      );
    end
  endgenerate

endmodule
