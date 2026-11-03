// quant_lrelu() from runtime.h as a 4-stage register pipeline (latency = 4 clocks).
//   1) leaky : x<0 ? (nzero ? x : 0) : x << pl_scale
//   2) round : t = x + (shift>0 ? (1<<(shift-1)) - (x>>shift is even) : 0)
//   3) shift : t >> shift          (steps 2+3 == round-half-to-even of x / 2^shift)
//   4) clip  : clamp to [clip_min, clip_max]
module wb_quant_lrelu #(
  parameter WIDTH = 32
)(
  input  logic                     clk,
  input  logic                     rstn,
  input  logic signed [WIDTH-1:0]  x,
  input  logic                     nzero,
  input  logic        [4:0]        shift,
  input  logic        [4:0]        pl_scale,
  input  logic signed [WIDTH-1:0]  clip_min,
  input  logic signed [WIDTH-1:0]  clip_max,
  output logic signed [WIDTH-1:0]  y
);

  logic signed [WIDTH-1:0] lk_q, sum_q, sh_q;

  wire signed [WIDTH-1:0] lk_d  = (x < 0) ? (nzero ? x : '0) : (x <<< pl_scale);

  wire signed [WIDTH-1:0] q     = lk_q >>> shift;
  wire signed [WIDTH-1:0] half  = (WIDTH'(1) <<< (shift - 5'd1));
  wire signed [WIDTH-1:0] adj   = (shift != 0) ? (half - (q[0] ? WIDTH'(0) : WIDTH'(1))) : '0;

  wire signed [WIDTH-1:0] clipd = (sh_q < clip_min) ? clip_min :
                                  (sh_q > clip_max) ? clip_max : sh_q;

  always_ff @(posedge clk)
    if (!rstn) begin
      lk_q <= '0; sum_q <= '0; sh_q <= '0; y <= '0;
    end else begin
      lk_q  <= lk_d;
      sum_q <= lk_q + adj;
      sh_q  <= sum_q >>> shift;
      y     <= clipd;
    end

endmodule
