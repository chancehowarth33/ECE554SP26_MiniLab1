module MAC #
(
parameter DATA_WIDTH = 8
)
(
input logic clk,
input logic rst_n,
input logic En,
input logic Clr,
input logic [DATA_WIDTH-1:0] Ain,
input logic [DATA_WIDTH-1:0] Bin,
output logic [DATA_WIDTH*3-1:0] Cout
);

logic [DATA_WIDTH*3-1:0] accumulator;
logic [DATA_WIDTH*2-1:0] mult;

// multiply
assign mult = Ain * Bin;

always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    accumulator <= 0;
  end
  else if (Clr) begin
    accumulator <= 0;
  end
  else if (En) begin
    accumulator <= accumulator + mult;
  end
end

assign Cout = accumulator;

endmodule
