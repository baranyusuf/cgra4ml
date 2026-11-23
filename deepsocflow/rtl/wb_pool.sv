`timescale 1ns/1ps
// Max / average pooling in the PL, replaces the pooling sweep that the PS did in run().
//
//  Every output pixel of the bundle (conv-output coordinates h, w, c) is handed in once (start). The pixel is stored in the pool RAM
//  (NHWC address, 8 bit); then the unit does exactly what the PS did:
//    1) window index   ixh_beg = trunc((h + psh_shift - pkh + 1) / psh),  same for w   (wb_sdiv_pipe, free running, ready long before start)
//       the pixel is skipped when ix < 0, or when the division has a remainder and the pixel is not on the last row / column;
//       on the last row / column with a remainder the sweep starts one window later
//    2) sweep: ixh = ixh_beg .. xh_sweep-1, ixw = ixw_beg .. xw_sweep-1  (xh_sweep = last row ? ph : ixh_beg+1, same for w)
//       window of (ixh, ixw): rows (ph_beg, h], columns (pw_beg, w]  with  ph_beg = max(psh*ixh - psh_shift, 0) - 1  (moves by psh per ixh)
//    3) window: one pool RAM read per clock (address steps: -co per column, -cw*co per row), max or sum
//    4) avg:    result = div_round(sum, count)   (bit exact copy of the C macro, two sequential divisions)
//    5) pool activation quant_lrelu(pa_*) on the result (max and avg, as in python: pool.act is applied after the pool)
//    6) emit:   one pooled pixel (ixh, ixw, c, value) at a time, em_valid stays high until em_ack
//  done pulses for one clock when the pixel is finished (all windows emitted and acknowledged).
//  Needs no multiplier except psh*ixh_beg / psw*ixw_beg once per pixel (registered).
module wb_pool #(
  parameter WIDTH  = 32,
  parameter WORDS  = 1,                                   // >= number of conv-output words of the largest pooled bundle
  parameter ADDR_W = (WORDS > 1) ? $clog2(WORDS) : 1,
  parameter DIM_W  = 10,                                  // bits of h / w (i_yh / i_yw of PIXEL_PACK)
  parameter PAR_W  = 8,                                   // bits of pool size, stride and shift
  parameter IDX_W  = DIM_W + 2,                           // signed window index
  parameter ACC_W  = 24                                   // signed accumulator (sum of the window)
)(
  input  logic                 clk,
  input  logic                 rstn,

  // pixel that is accepted (start valid for 1 clock; h, w, c, n, addr, val stable from the clocks before start on)
  input  logic                 start,
  input  logic [ADDR_W-1:0]    pix_addr,    // NHWC address of the pixel (conv-output coordinates)
  input  logic signed [7:0]    pix_val,
  input  logic [DIM_W-1:0]     pix_h,
  input  logic [DIM_W-1:0]     pix_w,
  input  logic [21:0]          pix_c,
  input  logic [4:0]           pix_n,

  // bundle parameters (stable)
  input  logic                 mode_avg,    // 0: max, 1: avg
  input  logic [WIDTH-1:0]     yh,          // conv-output height / width (ram_ch / ram_cw)
  input  logic [WIDTH-1:0]     yw,
  input  logic [WIDTH-1:0]     step_c,      // address step between columns  (= channels)
  input  logic [WIDTH-1:0]     step_h,      // address step between rows     (= width * channels)
  input  logic [PAR_W-1:0]     pkh,
  input  logic [PAR_W-1:0]     pkw,
  input  logic [PAR_W-1:0]     psh,
  input  logic [PAR_W-1:0]     psw,
  input  logic [PAR_W-1:0]     psh_shift,
  input  logic [PAR_W-1:0]     psw_shift,
  input  logic [DIM_W-1:0]     ph,          // pooled height / width
  input  logic [DIM_W-1:0]     pw,
  input  logic                 pa_nzero,
  input  logic [4:0]           pa_shift,
  input  logic [4:0]           pa_pl_scale,
  input  logic signed [WIDTH-1:0] pa_clip_min,
  input  logic signed [WIDTH-1:0] pa_clip_max,

  // result stream: one pooled pixel at a time, em_valid stays high until em_ack
  output logic                 em_valid,
  output logic [WIDTH-1:0]     em_data,
  output logic [DIM_W-1:0]     em_h,
  output logic [DIM_W-1:0]     em_w,
  output logic [21:0]          em_c,
  output logic [4:0]           em_n,
  input  logic                 em_ack,

  output logic                 idle,        // waiting for a pixel
  output logic                 done         // 1 clock: the pixel is finished
);

  localparam LAT_QL = 4;                                  // wb_quant_lrelu latency

  // ---------------- free running window index dividers ----------------
  wire signed [IDX_W-1:0] num_h = $signed({{(IDX_W-DIM_W){1'b0}}, pix_h}) + $signed({{(IDX_W-PAR_W){1'b0}}, psh_shift}) - $signed({{(IDX_W-PAR_W){1'b0}}, pkh}) + IDX_W'(1);
  wire signed [IDX_W-1:0] num_w = $signed({{(IDX_W-DIM_W){1'b0}}, pix_w}) + $signed({{(IDX_W-PAR_W){1'b0}}, psw_shift}) - $signed({{(IDX_W-PAR_W){1'b0}}, pkw}) + IDX_W'(1);
  logic signed [IDX_W-1:0] q_h, q_w;
  logic                    rnz_h, rnz_w;
  wb_sdiv_pipe #(.NW(IDX_W), .DW(PAR_W)) DIV_H (.clk(clk), .rstn(rstn), .num(num_h), .den(psh), .quot(q_h), .rem_nz(rnz_h));
  wb_sdiv_pipe #(.NW(IDX_W), .DW(PAR_W)) DIV_W (.clk(clk), .rstn(rstn), .num(num_w), .den(psw), .quot(q_w), .rem_nz(rnz_w));

  // ---------------- pool RAM ----------------
  logic [ADDR_W-1:0]  rd_addr;
  logic signed [7:0]  rd_q;
  ram_sdp_simple #(.WIDTH(8), .DEPTH(WORDS), .ADDRW(ADDR_W)) POOL_RAM (
    .clkA(clk), .clkB(clk),
    .weA(start), .enaA(start), .addrA(pix_addr), .diA(pix_val),
    .enaB(1'b1), .addrB(rd_addr), .doB(rd_q)
  );

  // ---------------- pool activation (free running pipeline, LAT_QL clocks) ----------------
  logic signed [WIDTH-1:0] act_x, act_y;
  wb_quant_lrelu #(.WIDTH(WIDTH)) POOL_ACT (
    .clk(clk), .rstn(rstn),
    .x(act_x), .nzero(pa_nzero), .shift(pa_shift), .pl_scale(pa_pl_scale),
    .clip_min(pa_clip_min), .clip_max(pa_clip_max), .y(act_y)
  );

  // ---------------- sequential divider for the average ----------------
  logic               d_start, d_busy, d_done;
  logic [ACC_W-1:0]   d_num, d_den, d_q;
  wb_udiv_seq #(.NW(ACC_W)) DIV_AVG (.clk(clk), .rstn(rstn), .start(d_start), .num(d_num), .den(d_den), .busy(d_busy), .done(d_done), .q(d_q), .rem());

  // ---------------- FSM ----------------
  typedef enum logic [4:0] {
    S_IDLE, S_DEC, S_MUL, S_PB, S_HCHK, S_WCHK, S_WINIT, S_RINIT, S_RD, S_DRAIN, S_RES,
    S_AVG1, S_AVG1W, S_AVG2, S_AVG2W, S_ACT, S_ACTW, S_EM, S_NEXT, S_DONE
  } state_t;
  state_t state;

  logic [DIM_W-1:0]        l_h, l_w;                  // latched pixel (bottom right corner of every window of this pixel)
  logic [21:0]             l_c;
  logic [4:0]              l_n;
  logic [ADDR_W-1:0]       l_addr;
  logic signed [IDX_W-1:0] ixh_beg, ixw_beg, xh_sweep, xw_sweep, ixh, ixw;
  logic signed [IDX_W-1:0] ph_beg, pw_beg, pw_beg_const;
  logic signed [IDX_W-1:0] qh_l, qw_l;
  logic                    rnz_h_l, rnz_w_l;
  logic signed [IDX_W+PAR_W:0] mh_q, mw_q;
  logic signed [IDX_W-1:0] r_left, c_total, c_left;
  logic [ADDR_W-1:0]       row_addr, addr;
  logic                    v1;                        // pool RAM data of the previous clock's read is on rd_q
  logic signed [ACC_W-1:0] acc;
  logic [ACC_W-1:0]        cnt;
  logic signed [ACC_W-1:0] a_l;                       // sum, kept for the second division
  logic signed [ACC_W-1:0] res;
  logic [2:0]              act_cnt;

  wire  last_h = (WIDTH'(l_h) + WIDTH'(1) == yh);
  wire  last_w = (WIDTH'(l_w) + WIDTH'(1) == yw);
  wire  skip   = (qh_l < 0) || (qw_l < 0) || (rnz_h_l && !last_h) || (rnz_w_l && !last_w);
  wire signed [IDX_W-1:0] ixh_b = qh_l + IDX_W'(rnz_h_l);       // remainder != 0 here only on the last row / column
  wire signed [IDX_W-1:0] ixw_b = qw_l + IDX_W'(rnz_w_l);

  wire signed [IDX_W:0] ph_t = $signed({1'b0, mh_q[IDX_W-1:0]}) - $signed({{(IDX_W+1-PAR_W){1'b0}}, psh_shift});
  wire signed [IDX_W:0] pw_t = $signed({1'b0, mw_q[IDX_W-1:0]}) - $signed({{(IDX_W+1-PAR_W){1'b0}}, psw_shift});
  wire signed [IDX_W:0]   ph_cl = (ph_t < 0) ? '0 : ph_t;                       // max(psh*ixh - psh_shift, 0)
  wire signed [IDX_W-1:0] ph_c  = IDX_W'(ph_cl) - IDX_W'(1);
  wire signed [IDX_W:0]   pw_cl = (pw_t < 0) ? '0 : pw_t;
  wire signed [IDX_W-1:0] pw_c  = IDX_W'(pw_cl) - IDX_W'(1);

  wire signed [IDX_W-1:0] rows_w = $signed({{(IDX_W-DIM_W){1'b0}}, l_h}) - ph_beg;   // rows of the window (ph_beg, h]
  wire signed [IDX_W-1:0] cols_w = $signed({{(IDX_W-DIM_W){1'b0}}, l_w}) - pw_beg;

  // average: div_round(a, b) = (a + b/2 - (~(b | (a/b)) & 1)) / b, C integer division (truncation); b = cnt
  wire                    a_neg  = a_l[ACC_W-1];
  wire [ACC_W-1:0]        a_mag  = a_neg ? ACC_W'(-a_l) : ACC_W'(a_l);
  logic signed [ACC_W-1:0] q1;                        // a / b
  wire                    flag   = !cnt[0] && !q1[0];
  wire signed [ACC_W-1:0] num2   = a_l + $signed(ACC_W'(cnt >> 1)) - $signed(ACC_W'(flag));
  wire                    n2_neg = num2[ACC_W-1];
  wire [ACC_W-1:0]        n2_mag = n2_neg ? ACC_W'(-num2) : ACC_W'(num2);

  assign idle = (state == S_IDLE);
  assign done = (state == S_DONE);
  assign rd_addr = addr;

  always_ff @(posedge clk)
    if (!rstn) begin
      state <= S_IDLE;
      l_h <= '0; l_w <= '0; l_c <= '0; l_n <= '0; l_addr <= '0;
      ixh_beg <= '0; ixw_beg <= '0; xh_sweep <= '0; xw_sweep <= '0; ixh <= '0; ixw <= '0;
      ph_beg <= '0; pw_beg <= '0; pw_beg_const <= '0; qh_l <= '0; qw_l <= '0; rnz_h_l <= 1'b0; rnz_w_l <= 1'b0;
      mh_q <= '0; mw_q <= '0; r_left <= '0; c_total <= '0; c_left <= '0; row_addr <= '0; addr <= '0;
      v1 <= 1'b0; acc <= '0; cnt <= '0; a_l <= '0; res <= '0; q1 <= '0; act_cnt <= '0; act_x <= '0;
      em_valid <= 1'b0; em_data <= '0; em_h <= '0; em_w <= '0; em_c <= '0; em_n <= '0;
      d_start <= 1'b0; d_num <= '0; d_den <= '0;
    end else begin
      v1      <= (state == S_RD);
      d_start <= 1'b0;

      // window accumulation: data of the read issued in the previous clock
      if (v1) begin
        cnt <= cnt + ACC_W'(1);
        if (mode_avg)                                       acc <= acc + ACC_W'(rd_q);
        else if (ACC_W'(rd_q) > acc)                        acc <= ACC_W'(rd_q);
      end

      case (state)

        S_IDLE: if (start) begin
          l_h <= pix_h; l_w <= pix_w; l_c <= pix_c; l_n <= pix_n; l_addr <= pix_addr;
          qh_l <= q_h; qw_l <= q_w; rnz_h_l <= rnz_h; rnz_w_l <= rnz_w;
          state <= S_DEC;
        end

        S_DEC: begin
          ixh_beg  <= ixh_b;
          ixw_beg  <= ixw_b;
          xh_sweep <= last_h ? $signed({{(IDX_W-DIM_W){1'b0}}, ph}) : ixh_b + IDX_W'(1);
          xw_sweep <= last_w ? $signed({{(IDX_W-DIM_W){1'b0}}, pw}) : ixw_b + IDX_W'(1);
          state    <= skip ? S_DONE : S_MUL;
        end

        S_MUL: begin
          mh_q  <= $signed({1'b0, psh}) * ixh_beg;
          mw_q  <= $signed({1'b0, psw}) * ixw_beg;
          state <= S_PB;
        end

        S_PB: begin
          ixh          <= ixh_beg;
          ph_beg       <= ph_c;
          pw_beg_const <= pw_c;
          state        <= S_HCHK;
        end

        S_HCHK: begin
          if (ixh < xh_sweep) begin
            ixw    <= ixw_beg;
            pw_beg <= pw_beg_const;
            state  <= S_WCHK;
          end else
            state  <= S_DONE;
        end

        S_WCHK: begin
          if (ixw < xw_sweep) state <= S_WINIT;
          else begin
            ixh    <= ixh + IDX_W'(1);
            ph_beg <= ph_beg + $signed({{(IDX_W-PAR_W){1'b0}}, psh});
            state  <= S_HCHK;
          end
        end

        S_WINIT: begin
          row_addr <= l_addr;
          r_left   <= rows_w;
          c_total  <= cols_w;
          acc      <= mode_avg ? '0 : {1'b1, {(ACC_W-1){1'b0}}};
          cnt      <= '0;
          state    <= (rows_w <= 0 || cols_w <= 0) ? S_DRAIN : S_RINIT;
        end

        S_RINIT: begin
          addr   <= row_addr;
          c_left <= c_total;
          state  <= S_RD;
        end

        S_RD: begin                                         // address 'addr' is read; data is on rd_q next clock
          addr   <= addr - ADDR_W'(step_c);
          c_left <= c_left - IDX_W'(1);
          if (c_left == IDX_W'(1)) begin
            row_addr <= row_addr - ADDR_W'(step_h);
            r_left   <= r_left - IDX_W'(1);
            state    <= (r_left == IDX_W'(1)) ? S_DRAIN : S_RINIT;
          end
        end

        S_DRAIN: state <= S_RES;                            // the last read is accumulated in this clock

        S_RES: begin
          if (mode_avg) begin
            a_l   <= acc;
            state <= S_AVG1;
          end else begin
            res   <= acc;
            state <= S_ACT;
          end
        end

        S_AVG1: begin                                       // q1 = a / cnt
          d_num   <= a_mag;
          d_den   <= (cnt == '0) ? ACC_W'(1) : cnt;
          d_start <= 1'b1;
          state   <= S_AVG1W;
        end

        S_AVG1W: if (d_done) begin
          q1    <= a_neg ? -$signed(d_q) : $signed(d_q);
          state <= S_AVG2;
        end

        S_AVG2: begin                                       // res = (a + cnt/2 - flag) / cnt
          d_num   <= n2_mag;
          d_den   <= (cnt == '0) ? ACC_W'(1) : cnt;
          d_start <= 1'b1;
          state   <= S_AVG2W;
        end

        S_AVG2W: if (d_done) begin
          res   <= n2_neg ? -$signed(d_q) : $signed(d_q);
          state <= S_ACT;
        end

        S_ACT: begin
          act_x   <= WIDTH'(res);
          act_cnt <= 3'(LAT_QL);
          state   <= S_ACTW;
        end

        S_ACTW: begin
          if (act_cnt == 0) begin
            em_data  <= WIDTH'(act_y);
            em_h     <= DIM_W'(ixh);
            em_w     <= DIM_W'(ixw);
            em_c     <= l_c;
            em_n     <= l_n;
            em_valid <= 1'b1;
            state    <= S_EM;
          end else
            act_cnt <= act_cnt - 3'd1;
        end

        S_EM: if (em_ack) begin
          em_valid <= 1'b0;
          state    <= S_NEXT;
        end

        S_NEXT: begin
          ixw    <= ixw + IDX_W'(1);
          pw_beg <= pw_beg + $signed({{(IDX_W-PAR_W){1'b0}}, psw});
          state  <= S_WCHK;
        end

        S_DONE: state <= S_IDLE;

        default: state <= S_IDLE;
      endcase
    end

  // synthesis translate_off
  // Simulation check: the free running dividers must have settled when a pixel is accepted
  always @(posedge clk) if (rstn && start) begin
    if (q_h != num_h / $signed({{(IDX_W-PAR_W){1'b0}}, psh}) || (rnz_h != ((num_h % $signed({{(IDX_W-PAR_W){1'b0}}, psh})) != 0)) ||
        q_w != num_w / $signed({{(IDX_W-PAR_W){1'b0}}, psw}) || (rnz_w != ((num_w % $signed({{(IDX_W-PAR_W){1'b0}}, psw})) != 0)))
      $fatal(1, "[POOL] window index divider not settled at pixel start (h=%0d w=%0d)", pix_h, pix_w);
  end
  // synthesis translate_on

endmodule


// Signed division by an unsigned divisor, truncating like C (quot = num / den, rem != 0 <=> num % den != 0),
// as a pipeline with one result bit per stage. Latency = NW + 2 clocks. den must stay constant (it is a bundle parameter).
module wb_sdiv_pipe #(
  parameter NW = 12,
  parameter DW = 8
)(
  input  logic                     clk,
  input  logic                     rstn,
  input  logic signed [NW-1:0]     num,
  input  logic        [DW-1:0]     den,
  output logic signed [NW-1:0]     quot,
  output logic                     rem_nz
);
  logic [NW-1:0] n_p [0:NW];
  logic [NW-1:0] q_p [0:NW];
  logic [DW-1:0] r_p [0:NW];
  logic          neg_p [0:NW];

  wire           neg = num[NW-1];
  wire [NW-1:0]  mag = neg ? NW'(-num) : NW'(num);

  logic [DW:0] t_c  [0:NW-1];
  logic        ge_c [0:NW-1];
  always_comb
    for (int i = 0; i < NW; i++) begin
      t_c [i] = {r_p[i], n_p[i][NW-1-i]};
      ge_c[i] = (t_c[i] >= {1'b0, den});
    end

  always_ff @(posedge clk)
    if (!rstn) begin
      for (int i = 0; i <= NW; i++) begin
        n_p[i] <= '0; q_p[i] <= '0; r_p[i] <= '0; neg_p[i] <= 1'b0;
      end
    end else begin
      n_p[0] <= mag; q_p[0] <= '0; r_p[0] <= '0; neg_p[0] <= neg;      // stage 0: magnitude and sign
      for (int i = 0; i < NW; i++) begin
        n_p[i+1]   <= n_p[i];
        neg_p[i+1] <= neg_p[i];
        r_p[i+1]   <= ge_c[i] ? DW'(t_c[i] - {1'b0, den}) : DW'(t_c[i]);
        q_p[i+1]   <= q_p[i] | (NW'(ge_c[i]) << (NW-1-i));
      end
    end

  always_ff @(posedge clk)
    if (!rstn) begin
      quot <= '0; rem_nz <= 1'b0;
    end else begin
      quot   <= neg_p[NW] ? -$signed(q_p[NW]) : $signed(q_p[NW]);
      rem_nz <= (r_p[NW] != '0);
    end
endmodule


// Unsigned restoring divider, one quotient bit per clock. start pulses (while not busy), done pulses 1 clock when q is valid.
module wb_udiv_seq #(
  parameter NW = 24
)(
  input  logic          clk,
  input  logic          rstn,
  input  logic          start,
  input  logic [NW-1:0] num,
  input  logic [NW-1:0] den,
  output logic          busy,
  output logic          done,
  output logic [NW-1:0] q,
  output logic [NW-1:0] rem        // remainder, valid together with done
);
  localparam CW = $clog2(NW + 1);
  logic [NW-1:0] n_r, d_r;
  logic [NW:0]   r;
  logic [CW-1:0] i;
  wire  [NW:0]   t  = {r[NW-1:0], n_r[NW-1]};
  wire           ge = (t >= {1'b0, d_r});
  assign rem = r[NW-1:0];

  always_ff @(posedge clk)
    if (!rstn) begin
      busy <= 1'b0; done <= 1'b0; q <= '0; n_r <= '0; d_r <= '0; r <= '0; i <= '0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        n_r <= num; d_r <= den; r <= '0; q <= '0; i <= CW'(NW); busy <= 1'b1;
      end else if (busy) begin
        r   <= ge ? (t - {1'b0, d_r}) : t;
        q   <= {q[NW-2:0], ge};
        n_r <= {n_r[NW-2:0], 1'b0};
        i   <= i - CW'(1);
        if (i == CW'(1)) begin busy <= 1'b0; done <= 1'b1; end
      end
    end
endmodule
