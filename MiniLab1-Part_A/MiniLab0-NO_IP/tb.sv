`timescale 1ns/1ps

module tb;

parameter DATA_WIDTH = 8;

logic clk;
logic rst_n;
logic En, Clr;
logic [DATA_WIDTH-1:0] Ain, Bin;
logic [DATA_WIDTH*3-1:0] Cout;

// instantiate MAC
MAC #(DATA_WIDTH) mac (
    .clk(clk),
    .rst_n(rst_n),
    .En(En),
    .Clr(Clr),
    .Ain(Ain),
    .Bin(Bin),
    .Cout(Cout)
);

// clock
always #5 clk = ~clk;

initial begin

clk = 0;
rst_n = 0;
En = 0;
Clr = 0;

#20;
rst_n = 1;

/////////////////////
// clear accumulator
/////////////////////
Clr = 1;
@(posedge clk);
Clr = 0;

/////////////////////
// i0
/////////////////////
@(posedge clk);
Ain = 8'd1;
Bin = 8'd6;
En = 1;
$display("i0 -> %0d * %0d = %0d", Ain, Bin, Ain*Bin);

/////////////////////
// i1
/////////////////////
@(posedge clk);
Ain = 8'd2;
Bin = 8'd7;
$display("i1 -> %0d * %0d = %0d", Ain, Bin, Ain*Bin);

/////////////////////
// i2
/////////////////////
@(posedge clk);
Ain = 8'd3;
Bin = 8'd8;
$display("i2 -> %0d * %0d = %0d", Ain, Bin, Ain*Bin);

/////////////////////
// i3
/////////////////////
@(posedge clk);
Ain = 8'd4;
Bin = 8'd9;
$display("i3 -> %0d * %0d = %0d", Ain, Bin, Ain*Bin);

/////////////////////
// i4
/////////////////////
@(posedge clk);
Ain = 8'd5;
Bin = 8'd10;
$display("i4 -> %0d * %0d = %0d", Ain, Bin, Ain*Bin);

@(posedge clk);
En = 0;

#20;

$display("Expected DOT PRODUCT = 130");
$display("MAC Output = %0d", Cout);

if (Cout == 130)
    $display("PASS");
else
    $display("FAIL");

$stop();

end

endmodule
