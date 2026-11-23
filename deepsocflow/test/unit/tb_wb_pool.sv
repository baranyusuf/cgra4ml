// Unit test of rtl/wb_pool.sv against a behavioural copy of the pooling sweep of the C firmware (max / avg, strides, 'same' shifts, activation).
// Run (Verilator >= 5):
//   verilator --binary --timing --top tb_pool deepsocflow/test/unit/tb_wb_pool.sv deepsocflow/rtl/wb_pool.sv deepsocflow/rtl/wb_quant_lrelu.sv deepsocflow/rtl/ext/ram_sdp_simple.sv --Mdir obj && ./obj/Vtb_pool
// Prints POOL_TB PASS when every emitted pooled pixel (position, channel, value) matches.
`timescale 1ns/1ps
// Unit test of wb_pool against a behavioural copy of the C sweep in runtime.h
module tb_one #(
  parameter YH=14, YW=10, CO=3, PKH=3, PKW=3, PSH=2, PSW=3, PH=7, PW=4, SHH=0, SHW=1,
  parameter AVG=0, PA_SHIFT=0, PA_NZ=1, PA_PL=0, NPIX=0
)(input logic clk, input logic rstn, output logic fin, output int errors);
  localparam WORDS = YH*YW*CO;
  localparam AW = $clog2(WORDS);
  logic start; logic [AW-1:0] pix_addr; logic signed [7:0] pix_val;
  logic [9:0] pix_h, pix_w; logic [21:0] pix_c; logic [4:0] pix_n;
  logic em_valid; logic [31:0] em_data; logic [9:0] em_h, em_w; logic [21:0] em_c; logic [4:0] em_n; logic em_ack;
  logic idle, done;
  localparam signed [31:0] CLIP_MAX = 7;
  wire signed [31:0] clip_min = -((CLIP_MAX+1) >>> PA_PL);
  wb_pool #(.WIDTH(32), .WORDS(WORDS)) dut (
    .clk(clk), .rstn(rstn), .start(start), .pix_addr(pix_addr), .pix_val(pix_val),
    .pix_h(pix_h), .pix_w(pix_w), .pix_c(pix_c), .pix_n(pix_n),
    .mode_avg(AVG != 0), .yh(32'(YH)), .yw(32'(YW)), .step_c(32'(CO)), .step_h(32'(YW*CO)),
    .pkh(8'(PKH)), .pkw(8'(PKW)), .psh(8'(PSH)), .psw(8'(PSW)), .psh_shift(8'(SHH)), .psw_shift(8'(SHW)),
    .ph(10'(PH)), .pw(10'(PW)), .pa_nzero(PA_NZ != 0), .pa_shift(5'(PA_SHIFT)), .pa_pl_scale(5'(PA_PL)),
    .pa_clip_min(clip_min), .pa_clip_max(CLIP_MAX),
    .em_valid(em_valid), .em_data(em_data), .em_h(em_h), .em_w(em_w), .em_c(em_c), .em_n(em_n), .em_ack(em_ack),
    .idle(idle), .done(done));

  int img [0:YH-1][0:YW-1][0:CO-1];
  // expected emissions
  int e_h[$], e_w[$], e_c[$], e_v[$];

  function automatic int shift_round(int n, int s);
    int t;
    if (s > 0) t = n + ((1<<(s-1)) - ((~(n>>>s)) & 1)); else t = n;
    return t >>> s;
  endfunction
  function automatic int quant_lrelu(int x, int nz, int sh, int pl);
    int y;
    y = (x < 0) ? (nz ? x : 0) : (x <<< pl);
    y = shift_round(y, sh);
    if (y < -(8 >>> pl)) y = -(8 >>> pl);
    if (y > 7) y = 7;
    return y;
  endfunction
  function automatic int div_round(int a, int b);
    int q1;
    q1 = a / b;
    return (a + (b/2) - ((~(b | q1)) & 1)) / b;
  endfunction

  task automatic model(input int i_yh, input int i_yw, input int c);
    int ph_end, pw_end, ph_beg_const, pw_beg_const, ixh_beg, ixw_beg, xh_sweep, xw_sweep;
    int nh, nw, qh, qw, rh, rw, ph_beg, pw_beg, result, count;
    nh = i_yh + SHH - PKH + 1; nw = i_yw + SHW - PKW + 1;
    qh = nh / PSH; rh = nh % PSH; qw = nw / PSW; rw = nw % PSW;
    ixh_beg = qh; ixw_beg = qw;
    if (ixh_beg < 0 || ixw_beg < 0) return;
    if (rh != 0) begin if (i_yh == YH-1) ixh_beg += 1; else return; end
    if (rw != 0) begin if (i_yw == YW-1) ixw_beg += 1; else return; end
    ph_end = i_yh; pw_end = i_yw;
    ph_beg_const = ((PSH*ixh_beg-SHH) > 0 ? (PSH*ixh_beg-SHH) : 0) - 1;
    pw_beg_const = ((PSW*ixw_beg-SHW) > 0 ? (PSW*ixw_beg-SHW) : 0) - 1;
    xh_sweep = (i_yh == YH-1) ? PH : ixh_beg+1;
    xw_sweep = (i_yw == YW-1) ? PW : ixw_beg+1;
    ph_beg = ph_beg_const;
    for (int ixh = ixh_beg; ixh < xh_sweep; ixh++) begin
      pw_beg = pw_beg_const;
      for (int ixw = ixw_beg; ixw < xw_sweep; ixw++) begin
        result = AVG ? 0 : -(1<<30);
        for (int ph = ph_end; ph > ph_beg; ph--)
          for (int pw = pw_end; pw > pw_beg; pw--)
            result = AVG ? result + img[ph][pw][c] : ((img[ph][pw][c] > result) ? img[ph][pw][c] : result);
        count = (ph_end-ph_beg)*(pw_end-pw_beg);
        if (AVG) result = div_round(result, count);
        result = quant_lrelu(result, PA_NZ, PA_SHIFT, PA_PL);
        e_h.push_back(ixh); e_w.push_back(ixw); e_c.push_back(c); e_v.push_back(result);
        pw_beg += PSW;
      end
      ph_beg += PSH;
    end
  endtask

  int seed;
  int nexp, ngot;
  initial begin
    fin = 0; errors = 0; start = 0; em_ack = 0; pix_h = 0; pix_w = 0; pix_c = 0; pix_n = 0; pix_addr = 0; pix_val = 0;
    seed = YH*1000 + YW*100 + CO*10 + PKH + AVG;
    for (int h=0; h<YH; h++) for (int w=0; w<YW; w++) for (int c=0; c<CO; c++) img[h][w][c] = $urandom_range(0,15) - 8;
    wait (rstn);
    repeat (5) @(posedge clk);
    for (int c=0; c<CO; c++)
      for (int h=0; h<YH; h++)
        for (int w=0; w<YW; w++) begin
          // present the pixel; dividers need time to settle
          @(posedge clk);
          pix_h <= h[9:0]; pix_w <= w[9:0]; pix_c <= c[21:0]; pix_n <= 0;
          pix_addr <= AW'((h*YW + w)*CO + c); pix_val <= 8'(img[h][w][c]);
          repeat (20) @(posedge clk);
          model(h, w, c);
          wait (idle);
          @(posedge clk); start <= 1; @(posedge clk); start <= 0;
          // PS role: take emitted pixels, ack them
          forever begin
            @(posedge clk);
            if (done) break;
            if (em_valid) begin
              if (e_h.size() == 0) begin errors++; $display("[%m] unexpected emission h=%0d w=%0d c=%0d v=%0d (pixel %0d,%0d,%0d)", em_h, em_w, em_c, $signed(em_data), h, w, c); end
              else begin
                if (em_h != e_h[0] || em_w != e_w[0] || em_c != e_c[0] || $signed(em_data) != e_v[0]) begin
                  errors++;
                  if (errors < 10) $display("[%m] MISMATCH got (%0d,%0d,c%0d)=%0d exp (%0d,%0d,c%0d)=%0d at pixel (%0d,%0d,%0d)", em_h, em_w, em_c, $signed(em_data), e_h[0], e_w[0], e_c[0], e_v[0], h, w, c);
                end
                void'(e_h.pop_front()); void'(e_w.pop_front()); void'(e_c.pop_front()); void'(e_v.pop_front());
              end
              ngot++;
              em_ack <= 1; @(posedge clk); em_ack <= 0;
            end
          end
          if (e_h.size() != 0) begin errors++; $display("[%m] missing %0d emissions after pixel (%0d,%0d,%0d)", e_h.size(), h, w, c); e_h.delete(); e_w.delete(); e_c.delete(); e_v.delete(); end
        end
    $display("[%m] done: emissions=%0d errors=%0d", ngot, errors);
    fin = 1;
  end
endmodule

module tb_pool;
  logic clk = 0, rstn = 0;
  always #5 clk = ~clk;
  initial begin repeat (10) @(posedge clk); rstn = 1; end
  logic [5:0] fin; int err [6];
  // max, stride 2x3 same-like (shift w 1)
  tb_one #(.YH(14),.YW(10),.CO(3),.PKH(3),.PKW(3),.PSH(2),.PSW(3),.PH(7),.PW(4),.SHH(0),.SHW(1),.AVG(0)) t0 (clk,rstn,fin[0],err[0]);
  // avg, 3x3 stride 2 same
  tb_one #(.YH(14),.YW(10),.CO(3),.PKH(3),.PKW(3),.PSH(2),.PSW(2),.PH(7),.PW(5),.SHH(0),.SHW(0),.AVG(1)) t1 (clk,rstn,fin[1],err[1]);
  // max 2x2 stride 2 valid
  tb_one #(.YH(7),.YW(5),.CO(4),.PKH(2),.PKW(2),.PSH(2),.PSW(2),.PH(3),.PW(2),.SHH(0),.SHW(0),.AVG(0)) t2 (clk,rstn,fin[2],err[2]);
  // max 7x7 stride 7 (like the failing test)
  tb_one #(.YH(28),.YW(28),.CO(2),.PKH(7),.PKW(7),.PSH(7),.PSW(7),.PH(4),.PW(4),.SHH(0),.SHW(0),.AVG(0)) t3 (clk,rstn,fin[3],err[3]);
  // avg 2x2 with activation shift
  tb_one #(.YH(8),.YW(8),.CO(2),.PKH(2),.PKW(2),.PSH(2),.PSW(2),.PH(4),.PW(4),.SHH(0),.SHW(0),.AVG(1),.PA_SHIFT(1),.PA_NZ(0)) t4 (clk,rstn,fin[4],err[4]);
  // avg 3x3 stride 1 same-like shift 1
  tb_one #(.YH(6),.YW(7),.CO(2),.PKH(3),.PKW(3),.PSH(1),.PSW(1),.PH(6),.PW(7),.SHH(1),.SHW(1),.AVG(1)) t5 (clk,rstn,fin[5],err[5]);
  initial begin
    wait (&fin);
    $display("ALL DONE errors: %0d %0d %0d %0d %0d %0d", err[0], err[1], err[2], err[3], err[4], err[5]);
    if (err[0]+err[1]+err[2]+err[3]+err[4]+err[5] == 0) $display("POOL_TB PASS"); else $display("POOL_TB FAIL");
    $finish;
  end
  initial begin #50_000_000; $display("POOL_TB TIMEOUT"); $finish; end
endmodule
