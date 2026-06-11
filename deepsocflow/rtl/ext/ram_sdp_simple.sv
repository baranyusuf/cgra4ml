`timescale 1ns/1ps
module ram_sdp_simple #(
  parameter WIDTH = 32,
  parameter DEPTH = 6272,
  parameter ADDRW = 13
)(
  input  logic              clkA, clkB, weA, enaA, enaB,
  input  logic [ADDRW-1:0]  addrA, addrB,
  input  logic [WIDTH-1:0]  diA,
  output logic [WIDTH-1:0]  doB
);
  (* ram_style = "block" *) logic [WIDTH-1:0] RAM [0:DEPTH-1];
  logic [WIDTH-1:0] readB;
  always_ff @(posedge clkA) if (enaA && weA) RAM[addrA] <= diA;
  always_ff @(posedge clkB) if (enaB)        readB      <= RAM[addrB];
  assign doB = readB;
endmodule