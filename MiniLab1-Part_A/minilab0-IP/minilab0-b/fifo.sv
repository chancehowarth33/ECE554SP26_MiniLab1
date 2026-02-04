`timescale 1 ps / 1 ps

module FIFO
#(
  parameter DEPTH = 8,        // Accept but don't use
  parameter DATA_WIDTH = 8    // Accept but don't use
)
(
  input clk,
  input rst_n,
  input rden,
  input wren,
  input [DATA_WIDTH-1:0] i_data,
  output [DATA_WIDTH-1:0] o_data,
  output full,
  output empty
);

// Internal wires
wire [7:0] usedw; 

// Instantiate the IP-generated FIFO (renamed from FIFO to FIFO_IP)
FIFO_IP fifo_inst (
  .clock(clk),
  .data(i_data),
  .rdreq(rden),
  .wrreq(wren),
  .empty(empty),
  .full(full),
  .q(o_data),
  .usedw(usedw)
);

endmodule