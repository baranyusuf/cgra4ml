// Asymmetric port RAM
// Write Wider than Read. Write Statement in loop
// asym_ram_sdp_write_wider.sv
`timescale 1ns / 1ps

module asym_ram_sdp_write_wider (
    clkA,
    clkB,
    enaA,
    weA,
    enaB,
    addrA,
    addrB,
    diA,
    doB
);
  parameter WIDTHA = 128;   
  parameter SIZEA  = 256;
  parameter ADDRWIDTHA = 8;

  parameter WIDTHB = 32;    
  parameter SIZEB  = 1024;
  parameter ADDRWIDTHB = 10;

  input  clkA;
  input  clkB;
  input  weA;
  input  enaA, enaB;
  input  [ADDRWIDTHA-1:0] addrA;
  input  [ADDRWIDTHB-1:0] addrB;
  input  [WIDTHA-1:0]     diA;
  output [WIDTHB-1:0]     doB;

  `define max(a, b) ((a) > (b) ? (a) : (b))
  `define min(a, b) ((a) < (b) ? (a) : (b))

  localparam maxSIZE   = `max(SIZEA, SIZEB);
  localparam maxWIDTH  = `max(WIDTHA, WIDTHB);
  localparam minWIDTH  = `min(WIDTHA, WIDTHB);

  localparam RATIO     = maxWIDTH / minWIDTH;   // 128/32 = 4
  localparam log2RATIO = $clog2(RATIO);

  reg [minWIDTH-1:0] RAM [0:maxSIZE-1];
  reg [WIDTHB-1:0]   readB;

  always @(posedge clkA) begin : ramwrite
    integer i;
    reg [log2RATIO-1:0] lsbaddr;
    if (enaA) begin
      if (weA) begin
        for (i = 0; i < RATIO; i = i + 1) begin
          lsbaddr = log2RATIO'(i);
          RAM[{addrA, lsbaddr}] <= diA[(i+1)*minWIDTH-1 -: minWIDTH];
        end
      end
    end
  end

  always @(posedge clkB) begin : ramread
    if (enaB)
      readB <= RAM[addrB];
  end
  assign doB = readB;

endmodule