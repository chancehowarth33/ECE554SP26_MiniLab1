module FIFO
#(
  parameter DEPTH = 8,
  parameter DATA_WIDTH = 8
)
(
  input  logic clk,
  input  logic rst_n,
  input  logic rden,
  input  logic wren,
  input  logic [DATA_WIDTH-1:0] i_data,
  output logic [DATA_WIDTH-1:0] o_data,
  output logic full,
  output logic empty
);

logic [DATA_WIDTH-1:0] mem [DEPTH-1:0];
logic [2:0] wr_ptr, rd_ptr;
logic [3:0] count;

// status
assign full  = (count == DEPTH);
assign empty = (count == 0);

always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wr_ptr <= 0;
    rd_ptr <= 0;
    count  <= 0;
    o_data <= 0;
  end
  else begin

    // write
    if (wren && !full) begin
      mem[wr_ptr] <= i_data;
      wr_ptr <= wr_ptr + 1;
    end

    // read
    if (rden && !empty) begin
      o_data <= mem[rd_ptr];
      rd_ptr <= rd_ptr + 1;
    end

    // update count
    case ({wren && !full, rden && !empty})
      2'b10: count <= count + 1;
      2'b01: count <= count - 1;
      default: count <= count;
    endcase

  end
end

endmodule
