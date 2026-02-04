module MAC8 #(
  parameter int DATA_WIDTH = 8
)(
  input  logic clk,
  input  logic rst_n,

  // control + B stream
  input  logic En_in,
  input  logic Clr_in,
  input  logic [DATA_WIDTH-1:0] b_in,

  // A inputs (parallel)
  // [0:7] is used since we loop from 0 to 7
  // normaly we would write [7:0] and loop from 7 down to 0
  input  logic [DATA_WIDTH-1:0] a_in [0:7],

  // outputs (vector of results)
  // [0:7] is used since we loop from 0 to 7
  // normaly we would write [7:0] and loop from 7 down to 0
  output logic [DATA_WIDTH*3-1:0] c_out [0:7]
);

  // pipeline registers // 
  logic [DATA_WIDTH-1:0] b_pipe   [0:7];
  logic [7:0]            en_pipe;
  logic [7:0]            clr_pipe;

  integer k;

  // shift B, En, Clr across pipeline
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (k = 0; k < 8; k++) begin
        // use '0 since it dynamicaly matches the width of the LHS
        b_pipe[k] <= '0;
      end
      en_pipe  <= '0;
      clr_pipe <= '0;
    end else begin
      // stage 0
      b_pipe[0]  <= b_in;
      en_pipe[0] <= En_in;
      clr_pipe[0]<= Clr_in;

      // stages 1..7
      for (k = 1; k < 8; k++) begin
        b_pipe[k]   <= b_pipe[k-1];
        en_pipe[k]  <= en_pipe[k-1];
        clr_pipe[k] <= clr_pipe[k-1];
      end
    end
  end

  // MAC instances
  // instatiate N MAC modules in this case N = 8
  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : MACS
      MAC #(.DATA_WIDTH(DATA_WIDTH)) u_mac (
        .clk  (clk),
        .rst_n(rst_n),
        .En   (en_pipe[i]),
        .Clr  (clr_pipe[i]),
        .Ain  (a_in[i]),
        .Bin  (b_pipe[i]),
        .Cout (c_out[i])
      );
    end
  endgenerate

endmodule
