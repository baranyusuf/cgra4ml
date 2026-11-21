`timescale 1ns/1ps
// Softmax in the PL, replaces the float exp / sum / divide that the PS did.
//
//  The logits arriving here are the (already clipped) outputs of the last activation, so they fit
//  in X_BITS <= 8 bits and there are at most 256 different values. For one output pixel with n classes:
//    1) every logit is written into a small RAM when the PS accepts it (cap_*), the last one starts the FSM
//    2) max = largest logit of the pixel
//    3) e_k = tab[max - logit_k]  (tab[d] = exp(-d / 2^frac) in Q24, filled by the PS; e_max = 1.0)
//       sum = e_0 + e_1 + ...      (sum >= 1.0, never zero)
//    4) p_k = floor(e_k * 2^24 / sum)   (Q24 probability, shift-subtract divider)
//  The exp table has the finest resolution (tab[j] = exp(-j/2^TAB_F)). A bundle whose logits have f fractional
//  bits reads tab[d << (TAB_F - f)], so one table serves every softmax bundle (cap_shift = TAB_F - f); indices past the
//  end are clamped to the last entry (exp(-17.3) rounds to 0 in Q24).
//  5) emit: p_k is handed out one class at a time (em_valid / em_data / em_k), the consumer answers each
//     with em_ack. em_data is the raw Q24 probability, or - when cap_quant is set - p_k rounded
//     (half to even) to cap_ofrac fractional bits and clipped to em_qmax (input of the next bundle).
//  Result is mathematically softmax(logits) (shift by max does not change it), no dependence on any
//  calibration constant.
module wb_softmax #(
  parameter WIDTH  = 32,
  parameter WORDS  = 16,                                  // >= number of output words of the softmax bundle
  parameter ADDR_W = (WORDS > 1) ? $clog2(WORDS) : 1,
  parameter QP     = 24,                                  // fractional bits of probabilities (and of the table)
  parameter SUM_W  = 48,
  parameter TAB_N  = 2220,                                // exp table entries (last one is 0)
  parameter TAB_AW = $clog2(TAB_N)
)(
  input  logic                 clk,
  input  logic                 rstn,

  // one logit (one class of one output pixel), valid for 1 clock
  input  logic                 cap_valid,
  input  logic [ADDR_W-1:0]    cap_addr,    // NHWC index of this class
  input  logic [7:0]           cap_logit,   // signed
  input  logic                 cap_last,    // last class of the pixel
  input  logic [WIDTH-1:0]     cap_ncls,    // number of classes of the pixel
  input  logic [3:0]           cap_shift,   // TAB_F - logit fraction bits of this bundle (0..7)
  input  logic                 cap_quant,   // 1: emit probabilities rounded to cap_ofrac bits, 0: raw Q24
  input  logic [3:0]           cap_ofrac,   // fractional bits of the quantized output (<= 7)
  input  logic [WIDTH-1:0]     em_qmax,     // largest quantized output value, (1 << (X_BITS-1)) - 1

  // exp table write (AXI-Lite window)
  input  logic                 tab_we,
  input  logic [TAB_AW-1:0]    tab_addr,
  input  logic [WIDTH-1:0]     tab_data,

  // result stream: one class at a time, em_valid stays high until em_ack
  output logic                 em_valid,
  output logic [WIDTH-1:0]     em_data,
  output logic [WIDTH-1:0]     em_k,        // class index of em_data
  input  logic                 em_ack,

  output logic                 idle         // no pixel is being processed or emitted
);

  typedef enum logic [3:0] {S_IDLE, S_MAX, S_EXP, S_RD, S_LOAD, S_INT, S_FRAC, S_WR, S_ERD, S_EQ, S_EOUT} state_t;
  state_t state;

  logic [ADDR_W-1:0]  base;
  logic [WIDTH-1:0]   n, k;
  logic signed [7:0]  max_l;
  logic [SUM_W-1:0]   sum;

  // ---------------- RAMs ----------------
  logic [ADDR_W-1:0]  lg_rd_addr;
  logic [7:0]         lg_q;
  ram_sdp_simple #(.WIDTH(8), .DEPTH(WORDS), .ADDRW(ADDR_W)) LOGIT_RAM (
    .clkA(clk), .clkB(clk),
    .weA(cap_valid), .enaA(cap_valid), .addrA(cap_addr), .diA(cap_logit),
    .enaB(1'b1), .addrB(lg_rd_addr), .doB(lg_q)
  );

  logic [TAB_AW-1:0]  tab_rd_addr;
  logic [WIDTH-1:0]   tab_q;
  ram_sdp_simple #(.WIDTH(WIDTH), .DEPTH(TAB_N), .ADDRW(TAB_AW)) EXP_TABLE (
    .clkA(clk), .clkB(clk),
    .weA(tab_we), .enaA(tab_we), .addrA(tab_addr), .diA(tab_data),
    .enaB(1'b1), .addrB(tab_rd_addr), .doB(tab_q)
  );

  logic               pr_we;
  logic [ADDR_W-1:0]  pr_wr_addr, fsm_pr_rd_addr;
  logic [WIDTH-1:0]   pr_wr_data, pr_q;
  ram_sdp_simple #(.WIDTH(WIDTH), .DEPTH(WORDS), .ADDRW(ADDR_W)) PROB_RAM (
    .clkA(clk), .clkB(clk),
    .weA(pr_we), .enaA(pr_we), .addrA(pr_wr_addr), .diA(pr_wr_data),
    .enaB(1'b1), .addrB(fsm_pr_rd_addr), .doB(pr_q)
  );

  assign idle = (state == S_IDLE);

  // ---------------- pipeline helpers (S_MAX / S_EXP) ----------------
  // pipeline of pass 2:  lg_q (v1) -> dd_q = max - logit (v2) -> idx_q = clamp(dd_q << sh) (v3) -> tab_q (v4)
  logic               v1, v2, v3, v4;        // data of that stage is valid in this clock
  logic [WIDTH-1:0]   a1, a2, a3, a4;        // class index belonging to that data
  logic [3:0]         sh_q;                  // latched cap_shift
  logic               quant_q;               // latched cap_quant
  logic [3:0]         of_q;                  // latched cap_ofrac
  logic [7:0]         dd_q;
  logic [TAB_AW-1:0]  idx_q;
  wire  signed [8:0]  dd   = $signed({max_l[7], max_l}) - $signed({lg_q[7], lg_q});   // 0..255
  wire  [14:0]        dsh  = {7'b0, dd_q} << sh_q;
  assign tab_rd_addr = idx_q;

  // ---------------- divider ----------------
  logic [SUM_W:0]     r;
  logic [QP:0]        q;
  logic [WIDTH-1:0]   cnt;
  wire  [SUM_W:0]     r2 = {r[SUM_W-1:0], 1'b0};

  // quantize p (Q24) to of_q fractional bits: round half to even, clip to em_qmax  (same as shift_round in python)
  wire [4:0]          qs   = 5'(QP) - 5'(of_q);
  wire [WIDTH-1:0]    qp   = pr_q;
  wire [WIDTH-1:0]    qhalf = WIDTH'(1) << (qs - 5'd1);
  wire [WIDTH-1:0]    qsum = qp + qhalf - WIDTH'(~(qp >> qs) & WIDTH'(1));
  wire [WIDTH-1:0]    qv   = qsum >> qs;
  wire [WIDTH-1:0]    emq  = (qv > em_qmax) ? em_qmax : qv;
  assign em_k = k;

  always_comb begin
    lg_rd_addr     = base + ADDR_W'(k);
    fsm_pr_rd_addr = base + ADDR_W'(k);
    pr_we          = 1'b0;
    pr_wr_addr     = '0;
    pr_wr_data     = '0;
    if (state == S_EXP && v4) begin
      pr_we      = 1'b1;
      pr_wr_addr = base + ADDR_W'(a4);
      pr_wr_data = tab_q;
    end
    if (state == S_WR) begin
      pr_we      = 1'b1;
      pr_wr_addr = base + ADDR_W'(k);
      pr_wr_data = WIDTH'(q);
    end
  end

  always_ff @(posedge clk)
    if (!rstn) begin
      state <= S_IDLE;
      base <= '0; n <= '0; k <= '0; max_l <= -8'sd128; sum <= '0;
      v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0; a1 <= '0; a2 <= '0; a3 <= '0; a4 <= '0;
      sh_q <= '0; dd_q <= '0; idx_q <= '0; quant_q <= 1'b0; of_q <= '0;
      em_valid <= 1'b0; em_data <= '0;
      r <= '0; q <= '0; cnt <= '0;
    end else begin
      case (state)

        S_IDLE: begin
          v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0;
          if (cap_valid && cap_last) begin
            sh_q  <= cap_shift;
            quant_q <= cap_quant;
            of_q    <= cap_ofrac;
            base  <= cap_addr - ADDR_W'(cap_ncls - 1);
            n     <= cap_ncls;
            k     <= '0;
            max_l <= -8'sd128;
            sum   <= '0;
            state <= S_MAX;
          end
        end

        // ---- pass 1: largest logit ----
        S_MAX: begin
          v1 <= (k < n);
          if (k < n) k <= k + 1;
          if (v1 && ($signed(lg_q) > max_l)) max_l <= $signed(lg_q);
          if (k >= n && !v1) begin
            k     <= '0;
            state <= S_EXP;
          end
        end

        // ---- pass 2: e_k = tab[max - logit_k], sum, store e_k ----
        S_EXP: begin
          v1 <= (k < n);
          a1 <= k;
          if (k < n) k <= k + 1;
          v2 <= v1;  a2 <= a1;  dd_q  <= dd[7:0];
          v3 <= v2;  a3 <= a2;  idx_q <= (dsh > 15'(TAB_N-1)) ? TAB_AW'(TAB_N-1) : dsh[TAB_AW-1:0];
          v4 <= v3;  a4 <= a3;
          if (v4) sum <= sum + SUM_W'(tab_q);
          if (k >= n && !v1 && !v2 && !v3 && !v4) begin
            k     <= '0;
            state <= S_RD;
          end
        end

        // ---- pass 3: p_k = e_k * 2^QP / sum ----
        S_RD: state <= S_LOAD;                          // address k is presented, data arrives next clock

        S_LOAD: begin
          r     <= {{(SUM_W+1-WIDTH){1'b0}}, pr_q};
          q     <= '0;
          cnt   <= WIDTH'(QP);
          state <= S_INT;
        end

        S_INT: begin                                    // integer bit (only 1 when e == sum)
          if (r >= {1'b0, sum}) begin
            q[QP] <= 1'b1;
            r     <= r - {1'b0, sum};
          end
          state <= S_FRAC;
        end

        S_FRAC: begin                                   // one fractional bit per clock
          if (r2 >= {1'b0, sum}) begin
            q[cnt-1] <= 1'b1;
            r        <= r2 - {1'b0, sum};
          end else begin
            r        <= r2;
          end
          cnt <= cnt - 1;
          if (cnt == 1) state <= S_WR;
        end

        S_WR: begin
          if (k + 1 >= n) begin k <= '0; state <= S_ERD; end
          else            begin k <= k + 1; state <= S_RD; end
        end

        // ---- pass 4: hand the probabilities out, one class per em_ack ----
        S_ERD: state <= S_EQ;                           // address k is presented, data arrives next clock

        S_EQ: begin
          em_data  <= quant_q ? emq : pr_q;
          em_valid <= 1'b1;
          state    <= S_EOUT;
        end

        S_EOUT: if (em_ack) begin
          em_valid <= 1'b0;
          if (k + 1 >= n) state <= S_IDLE;
          else            state <= S_ERD;
          k <= k + 1;
        end

        default: state <= S_IDLE;
      endcase
    end

endmodule
