module MAC8 #(
  parameter int DATA_WIDTH = 8,
  parameter int N          = 8
)(
  input  logic clk,
  input  logic rst_n,

  // control + B stream (into stage 0)
  input  logic En_in,
  input  logic Clr_in,
  input  logic [DATA_WIDTH-1:0] b_in,

  // A inputs (parallel)
  input  logic [DATA_WIDTH-1:0] a_in [0:N-1],

  // outputs (vector of results)
  output logic [DATA_WIDTH*3-1:0] c_out [0:N-1],

  // export pipelined control so top-level can align FIFO reads
  output logic [N-1:0] en_out,
  output logic [N-1:0] clr_out,

  // optional debug: pipelined B
  output logic [DATA_WIDTH-1:0] b_out [0:N-1]
);

  // pipeline registers
  logic [DATA_WIDTH-1:0] b_pipe [0:N-1];
  logic [N-1:0]          en_pipe;
  logic [N-1:0]          clr_pipe;

  integer k;

  // shift B, En, Clr across pipeline
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (k = 0; k < N; k++) begin
        b_pipe[k] <= '0;
      end
      en_pipe  <= '0;
      clr_pipe <= '0;
    end else begin
      // stage 0
      b_pipe[0]   <= b_in;
      en_pipe[0]  <= En_in;
      clr_pipe[0] <= Clr_in;

      // stages 1..N-1
      for (k = 1; k < N; k++) begin
        b_pipe[k]   <= b_pipe[k-1];
        en_pipe[k]  <= en_pipe[k-1];
        clr_pipe[k] <= clr_pipe[k-1];
      end
    end
  end

  // exports
  assign en_out  = en_pipe;
  assign clr_out = clr_pipe;

  genvar i;
  generate
    for (i = 0; i < N; i++) begin : MACS
      MAC #(.DATA_WIDTH(DATA_WIDTH)) u_mac (
        .clk  (clk),
        .rst_n(rst_n),
        .En   (en_pipe[i]),
        .Clr  (clr_pipe[i]),
        .Ain  (a_in[i]),
        .Bin  (b_pipe[i]),
        .Cout (c_out[i])
      );

      assign b_out[i] = b_pipe[i];
    end
  endgenerate

endmodule