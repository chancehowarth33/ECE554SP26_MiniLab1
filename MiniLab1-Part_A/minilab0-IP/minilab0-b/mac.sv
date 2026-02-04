module MAC
#(
  parameter DATA_WIDTH = 8
)
(
  input clk,
  input rst_n,
  input En,
  input Clr,
  input [DATA_WIDTH-1:0] Ain,
  input [DATA_WIDTH-1:0] Bin,
  output reg [DATA_WIDTH*3-1:0] Cout
);

// Internal wires
wire [15:0] mult_result;
wire [23:0] add_result;


// Instantiate multiplier IP
LPMMULT mult_inst (
  .dataa(Ain),
  .datab(Bin),
  .result(mult_result)
);

// Instantiate adder IP
LPMADDSUB adder_inst (
  .dataa(Cout),
  .datab(mult_result),
  .result(add_result)
);

// Accumulator register
always @(posedge clk or negedge rst_n) begin
  if (~rst_n) begin
    Cout <= {(DATA_WIDTH*3){1'b0}};
  end
  else if (Clr) begin
    Cout <= {(DATA_WIDTH*3){1'b0}};
  end
  else if (En) begin
    Cout <= add_result;
  end
end

endmodule