`timescale 1ns/1ps

// Output writer of the writeback (PL replacement of the firmware's tile_write / write_x).
//
// One finished output pixel (value + n,h,w,c) is handed in at a time (it_valid / it_ready). The unit
//   1) [flatten bundles] folds (h,w,c) into the channel axis,
//   2) computes the NHWC index of the pixel,
//   3) residual: stores the value (8 bit) into the add RAM of the writeback when the bundle feeds a residual add,
//   4) last bundle: writes the 32 bit word (softmax: Q24 probability converted to float) straight to DDR  (y[idx])
//      other bundles: scatters the pixel into the tile RAM, in the layout the next bundle reads its pixels in
//         x[p, n, l, w, cm, r + pad]      (r = row inside the PE_ROWS block, pad = bottom x_pad rows of block l-1 copied in front)
//      The tile RAM holds the whole tiled output of a bundle (X_BITS per element). When the last pixel of a bundle is in, fl_start
//      streams the packed bytes (little endian, element 0 in the low bits, the same bytes the firmware used to build) to DDR
//      with AXI4 bursts (never crossing 4 KB, the last beat is masked to o_bytes).
//
// Address / index math (identical to the firmware code it replaces):
//   p_off   = (ip == 0) ? 0 : (cm_p0 + (ip-1)*cm) * xp_words
//   flat    = p_off + (((n*L + l)*W + w)*ycm + icm) * (PE_ROWS + x_pad) + r         ycm = (ip == 0) ? cm_p0 : cm
// All variable x constant products are written as  coordinate * pre-multiplied constant  (constants are registered, they settle a
// few clocks after a new bundle header is loaded, long before the first pixel of the bundle arrives).
module wb_tilewr #(
  parameter
    WIDTH          = 32  ,
    PE_ROWS        = 7   ,
    X_BITS         = 4   ,
    AXI_WIDTH      = 128 ,
    AXI_ADDR_WIDTH = 32  ,
    MAX_BURST      = 16  ,    // beats per AXI burst (1..256)
    TILE_WORDS     = 1   ,    // elements of the largest tiled output
    ADD_WORDS      = 1   ,    // bytes of the residual add RAM
    NHWC_WORDS     = 1   ,    // simulation dump only
    SM_FRAC_BITS   = 24  ,    // fractional bits of the softmax probabilities of the last bundle

  localparam
    BEAT_BYTES = AXI_WIDTH / 8,
    LOGB       = $clog2(BEAT_BYTES),
    EPB        = AXI_WIDTH / X_BITS,                                   // elements per AXI beat
    TILE_ELEMS = ((TILE_WORDS + EPB - 1) / EPB) * EPB,
    TILE_BEATS = TILE_ELEMS / EPB,
    TILE_AW    = (TILE_ELEMS > 1) ? $clog2(TILE_ELEMS) : 1,
    TILE_BW    = (TILE_BEATS > 1) ? $clog2(TILE_BEATS) : 1,
    ADD_AW     = (ADD_WORDS  > 1) ? $clog2(ADD_WORDS ) : 1,
    BURST_W    = $clog2(MAX_BURST + 1) + 1
)(
  input  logic clk,
  input  logic rstn,

  // ---- pixel in ----
  input  logic                     it_valid,
  output logic                     it_ready,
  input  logic signed [WIDTH-1:0]  it_val,
  input  logic [4:0]               it_n,
  input  logic [9:0]               it_h,
  input  logic [9:0]               it_w,
  input  logic [21:0]              it_c,

  // ---- parameters of the current bundle (stable while the bundle's pixels are written) ----
  input  logic                     p_is_last,       // last bundle: pixels go to y in DDR
  input  logic                     p_is_softmax,    // last bundle only: value is a Q(SM_FRAC_BITS) probability -> float
  input  logic                     p_is_flatten,
  input  logic                     p_has_next,      // pixels are tiled for the next bundle
  input  logic [WIDTH-1:0]         p_add_base,      // residual add RAM base of this bundle's output (bit 31 set: none)
  input  logic [WIDTH-1:0]         p_yn, p_yh, p_yw, p_yc,   // size of the produced output (after pooling): n, h, w, c
  input  logic [WIDTH-1:0]         p_oh, p_ow, p_oc,         // NHWC dims of the stored output
  input  logic [WIDTH-1:0]         p_nx_l, p_nx_w, p_nx_cm, p_nx_cm_p0, p_nx_xpw, p_x_pad,   // tiling of the next bundle's input
  input  logic [WIDTH-1:0]         p_out_base,      // DDR byte address of the output buffer (last bundle: y)
  input  logic [WIDTH-1:0]         p_out_bytes,     // bytes of the tiled output

  // ---- flush of the tile RAM to DDR ----
  input  logic                     fl_start,        // 1 clock: all pixels of the bundle are in; parameters are sampled now
  output logic                     fl_done,         // 1 clock: every byte is written and acknowledged

  output logic                     items_idle,      // no pixel in progress
  output logic                     quiet,           // nothing in progress, nothing flushing, no write waiting for its response

  // ---- residual add RAM write ----
  output logic                     add_we,
  output logic [ADD_AW-1:0]        add_addr,
  output logic [7:0]               add_data,

  // ---- AXI4 write master (static fields are set by the top: id 0, INCR, size = full beat) ----
  output logic                     m_aw_valid,
  input  logic                     m_aw_ready,
  output logic [AXI_ADDR_WIDTH-1:0] m_aw_addr,
  output logic [7:0]               m_aw_len,
  output logic                     m_w_valid,
  input  logic                     m_w_ready,
  output logic [AXI_WIDTH-1:0]     m_w_data,
  output logic [BEAT_BYTES-1:0]    m_w_strb,
  output logic                     m_w_last,
  input  logic                     m_b_valid,
  output logic                     m_b_ready,

  // ---- simulation dump trigger (unused in hardware) ----
  input  logic                     dbg_dump,        // 1 clock: bundle dbg_ib is complete
  input  logic [WIDTH-1:0]         dbg_ib
);

  if (AXI_WIDTH < 64) begin : g_bad_axi_width
    $error("wb_tilewr: AXI_WIDTH must be at least 64");
  end

  // ===================== constants of the bundle (registered) =====================
  logic [WIDTH-1:0] k_wc, s_h, s_n, rp_q;
  logic [WIDTH-1:0] c1 [2], c2 [2], c3 [2], c2l [2];     // [0]: first p pass (ycm = cm_p0)   [1]: later passes (ycm = cm)
  logic [WIDTH-1:0] kp0, kp;
  always_ff @(posedge clk) begin
    k_wc <= p_yw * p_yc;                     // flatten: h * (yw*yc)
    s_h  <= p_ow * p_oc;                     // NHWC strides
    s_n  <= p_oh * s_h;
    rp_q <= WIDTH'(PE_ROWS) + p_x_pad;       // elements per (cm) column of a block: rows + pad
    c1 [0] <= p_nx_cm_p0 * rp_q;   c1 [1] <= p_nx_cm * rp_q;
    c2 [0] <= p_nx_w * c1[0];      c2 [1] <= p_nx_w * c1[1];                 // step of l
    c3 [0] <= p_nx_l * c2[0];      c3 [1] <= p_nx_l * c2[1];                 // step of n
    c2l[0] <= (p_nx_l - WIDTH'(1)) * c2[0];   c2l[1] <= (p_nx_l - WIDTH'(1)) * c2[1];   // jump from block 0 to block l-1
    kp0 <= p_nx_cm_p0 * p_nx_xpw;            // start of pass 1
    kp  <= p_nx_cm    * p_nx_xpw;            // size of a later pass
  end

  // ===================== shared signals (declared first) =====================
  typedef enum logic [4:0] {
    T_IDLE, T_F1, T_F2, T_I1, T_I2, T_B, T_Y,
    T_T0, T_T1, T_DIV, T_P1, T_P2, T_P3, T_P4, T_WM, T_WP, T_DONE
  } t_state_e;
  t_state_e t_state;

  typedef enum logic [2:0] {F_IDLE, F_PLAN, F_AW, F_WT, F_W, F_FIN} f_state_e;
  f_state_e f_state;
  wire fl_busy = (f_state != F_IDLE);

  logic [AXI_ADDR_WIDTH-1:0] y_addr;
  logic [31:0]               y_data;
  logic                      y_aw_done, y_w_done;
  logic [WIDTH-1:0]          outst;

  wire y_active  = (t_state == T_Y);
  wire y_awvalid = y_active && !y_aw_done;
  wire y_wvalid  = y_active && !y_w_done;
  wire f_awvalid = (f_state == F_AW);
  wire f_wvalid  = (f_state == F_W);
  wire aw_fire   = m_aw_valid && m_aw_ready;
  wire w_fire_y  = y_wvalid && m_w_ready;
  wire b_fire    = m_b_valid && m_b_ready;

  // ===================== pixel pipeline =====================
  logic signed [WIDTH-1:0] q_val;
  logic [4:0]       q_n;
  logic [9:0]       q_h, q_w;
  logic [21:0]      q_c;
  logic [WIDTH-1:0] f_a, f_b;                         // flatten products
  logic [WIDTH-1:0] c2v, h2, w2, n2, yh2;             // coordinates after flatten
  logic [WIDTH-1:0] i_n, i_h, i_w, idx;               // NHWC index
  logic             lastrow;
  logic [WIDTH-1:0] hq, hr, dnum;                     // h / PE_ROWS, h % PE_ROWS, c - cm_p0
  logic             first;                            // channel is in the first p pass
  logic [WIDTH-1:0] ip, icm;
  logic [WIDTH-1:0] m_n, m_l, m_w, m_c, m_p, s_a, s_b, base0, pbase;
  logic [WIDTH-1:0] rd, rstop;

  assign it_ready   = (t_state == T_IDLE) && !fl_busy;
  assign items_idle = (t_state == T_IDLE);

  // divider for the p pass index: (c - cm_p0) / cm
  logic             d_start, d_busy, d_done;
  logic [23:0]      d_q, d_r;
  wb_udiv_seq #(.NW(24)) DIV_P (.clk(clk), .rstn(rstn), .start(d_start), .num(dnum[23:0]), .den((p_nx_cm[23:0] == 24'd0) ? 24'd1 : p_nx_cm[23:0]),
                                .busy(d_busy), .done(d_done), .q(d_q), .rem(d_r));
  assign d_start = (t_state == T_T1) && !first;

  wire [WIDTH-1:0] sel_c1  = first ? c1 [0] : c1 [1];
  wire [WIDTH-1:0] sel_c2  = first ? c2 [0] : c2 [1];
  wire [WIDTH-1:0] sel_c3  = first ? c3 [0] : c3 [1];
  wire [WIDTH-1:0] sel_c2l = first ? c2l[0] : c2l[1];

  always_ff @(posedge clk)
    if (!rstn) begin
      t_state <= T_IDLE;
      q_val <= '0; q_n <= '0; q_h <= '0; q_w <= '0; q_c <= '0;
      f_a <= '0; f_b <= '0; c2v <= '0; h2 <= '0; w2 <= '0; n2 <= '0; yh2 <= '0;
      i_n <= '0; i_h <= '0; i_w <= '0; idx <= '0; lastrow <= 1'b0;
      hq <= '0; hr <= '0; dnum <= '0; first <= 1'b0; ip <= '0; icm <= '0;
      m_n <= '0; m_l <= '0; m_w <= '0; m_c <= '0; m_p <= '0; s_a <= '0; s_b <= '0; base0 <= '0; pbase <= '0;
      rd <= '0; rstop <= '0;
    end else begin
      unique case (t_state)
        T_IDLE: if (it_valid && it_ready) begin
          q_val <= it_val; q_n <= it_n; q_h <= it_h; q_w <= it_w; q_c <= it_c;
          t_state <= T_F1;
        end

        // ---- flatten: (n,h,w,c) -> (0, n, 0, (h*yw + w)*yc + c) ----
        T_F1: begin
          f_a <= WIDTH'(q_h) * k_wc;
          f_b <= WIDTH'(q_w) * p_yc;
          t_state <= T_F2;
        end
        T_F2: begin
          if (p_is_flatten) begin
            c2v <= f_a + f_b + WIDTH'(q_c); h2 <= WIDTH'(q_n); w2 <= '0; n2 <= '0; yh2 <= p_yn;
          end else begin
            c2v <= WIDTH'(q_c); h2 <= WIDTH'(q_h); w2 <= WIDTH'(q_w); n2 <= WIDTH'(q_n); yh2 <= p_yh;
          end
          t_state <= T_I1;
        end

        // ---- NHWC index ----
        T_I1: begin
          i_n <= n2 * s_n;
          i_h <= h2 * s_h;
          i_w <= w2 * p_oc;
          lastrow <= (h2 == yh2 - WIDTH'(1));
          t_state <= T_I2;
        end
        T_I2: begin
          idx <= i_n + i_h + i_w + c2v;
          t_state <= T_B;
        end

        // ---- idx is valid: residual write happens in this state; then DDR (last bundle) or the tile RAM ----
        T_B: begin
          if (p_is_last)         t_state <= T_Y;
          else if (p_has_next)   t_state <= T_T0;
          else                   t_state <= T_DONE;
        end

        // ---- last bundle: y[idx] to DDR ----
        T_Y: if ((y_aw_done || aw_fire) && (y_w_done || w_fire_y)) t_state <= T_DONE;

        // ---- tiling for the next bundle ----
        T_T0: begin
          hq    <= h2 / WIDTH'(PE_ROWS);
          hr    <= h2 % WIDTH'(PE_ROWS);
          first <= (c2v < p_nx_cm_p0);
          dnum  <= c2v - p_nx_cm_p0;
          t_state <= T_T1;
        end
        T_T1: begin
          if (first) begin ip <= '0; icm <= c2v; t_state <= T_P1; end
          else       t_state <= T_DIV;           // the divider was started by this state
        end
        T_DIV: if (d_done) begin
          ip  <= WIDTH'(d_q) + WIDTH'(1);
          icm <= WIDTH'(d_r);
          t_state <= T_P1;
        end
        T_P1: begin
          m_n <= n2  * sel_c3;
          m_l <= hq  * sel_c2;
          m_w <= w2  * sel_c1;
          m_c <= icm * rp_q;
          m_p <= (ip == '0) ? '0 : ((ip - WIDTH'(1)) * kp);
          t_state <= T_P2;
        end
        T_P2: begin
          s_a <= m_n + m_l + m_w;
          s_b <= m_c + m_p + ((ip == '0) ? '0 : kp0);
          t_state <= T_P3;
        end
        T_P3: begin
          base0 <= s_a + s_b;
          t_state <= T_P4;
        end
        T_P4: begin
          pbase  <= (hq == '0) ? (base0 + sel_c2l) : (base0 - sel_c2);   // block l-1 (block L-1 for l = 0)
          rd     <= hr;
          rstop  <= lastrow ? WIDTH'(PE_ROWS) : (hr + WIDTH'(1));          // the last image row zero fills the rows below it
          t_state <= T_WM;
        end
        T_WM: begin                                                        // element (row rd) of block l
          if (rd < p_x_pad) t_state <= T_WP;
          else begin
            rd <= rd + WIDTH'(1);
            t_state <= ((rd + WIDTH'(1)) == rstop) ? T_DONE : T_WM;
          end
        end
        T_WP: begin                                                        // padding copy (row PE_ROWS + rd) of block l-1
          rd <= rd + WIDTH'(1);
          t_state <= ((rd + WIDTH'(1)) == rstop) ? T_DONE : T_WM;
        end

        T_DONE: t_state <= T_IDLE;
        default: t_state <= T_IDLE;
      endcase
    end

  // ===================== residual add RAM write =====================
  assign add_we   = (t_state == T_B) && !p_is_last && !p_add_base[WIDTH-1];
  assign add_addr = ADD_AW'(p_add_base + idx);
  assign add_data = q_val[7:0];

  // ===================== tile RAM =====================
  wire                  tile_we   = (t_state == T_WM) || (t_state == T_WP);
  wire                  zero_row  = (rd != hr);
  wire [WIDTH-1:0]      tile_addr = (t_state == T_WM) ? (base0 + rd) : (pbase + WIDTH'(PE_ROWS) + rd);
  wire [X_BITS-1:0]     main_data = zero_row ? '0 : q_val[X_BITS-1:0];
  wire [X_BITS-1:0]     pad_data  = (zero_row || hq == '0) ? '0 : q_val[X_BITS-1:0];
  wire [X_BITS-1:0]     tile_data = (t_state == T_WM) ? main_data : pad_data;

  logic [TILE_BW-1:0]   tile_rd_addr;
  logic [AXI_WIDTH-1:0] tile_rd;
  asym_ram_sdp_read_wider #(
    .WIDTHA (X_BITS    ), .SIZEA (TILE_ELEMS), .ADDRWIDTHA (TILE_AW),
    .WIDTHB (AXI_WIDTH ), .SIZEB (TILE_BEATS), .ADDRWIDTHB (TILE_BW)
  ) TILE_RAM (
    .clkA (clk), .clkB (clk),
    .enaA (tile_we), .weA (tile_we), .addrA (TILE_AW'(tile_addr)), .diA (tile_data),
    .enaB (1'b1), .addrB (tile_rd_addr), .doB (tile_rd)
  );

  // ===================== last bundle: single beat writes of y =====================
  // value as stored in y: int32, or float for a softmax output
  // softmax output: Q(SM_FRAC_BITS) probability v, 0 <= v <= 2^SM_FRAC_BITS (exactly representable) -> IEEE float v / 2^SM_FRAC_BITS
  wire [31:0] q_u = q_val;
  logic [4:0] q_msb;
  always_comb begin
    q_msb = '0;
    for (int b = 0; b < 32; b++) if (q_u[b]) q_msb = 5'(b);
  end
  wire [31:0] q_shl = (q_msb <= 5'd23) ? (q_u << (5'd23 - q_msb)) : (q_u >> (q_msb - 5'd23));
  wire [31:0] q_f32 = (q_u == 32'd0) ? 32'd0 : {1'b0, 8'(32'd127 + 32'(q_msb) - 32'(SM_FRAC_BITS)), q_shl[22:0]};

  always_ff @(posedge clk)
    if (!rstn) begin
      y_addr <= '0; y_data <= '0; y_aw_done <= 1'b0; y_w_done <= 1'b0;
    end else begin
      if (t_state == T_B) begin
        y_addr <= AXI_ADDR_WIDTH'(p_out_base + {idx[WIDTH-3:0], 2'b00});
        y_data <= p_is_softmax ? q_f32 : q_val;
        y_aw_done <= 1'b0; y_w_done <= 1'b0;
      end else if (t_state == T_Y) begin
        if (aw_fire)   y_aw_done <= 1'b1;
        if (w_fire_y)  y_w_done  <= 1'b1;
      end
    end

  wire [LOGB-3:0]            y_lane    = y_addr[LOGB-1:2];          // 32 bit lane inside the beat (AXI_WIDTH >= 64)
  wire [BEAT_BYTES-1:0]      y_strb    = BEAT_BYTES'(4'hF) << ({y_lane, 2'b00});

  // ===================== flush of the tile RAM: bursts to DDR =====================
  logic [AXI_ADDR_WIDTH-1:0] f_addr;
  logic [WIDTH-1:0]          f_beats_rem, f_beat_idx;
  logic [LOGB-1:0]           f_tail;                           // valid bytes of the last beat (0: all)
  logic [BURST_W-1:0]        f_nb, f_bcnt;

  localparam [WIDTH-1:0] BEATS_4K = WIDTH'(4096 / BEAT_BYTES);
  wire [WIDTH-1:0] to4k   = BEATS_4K - WIDTH'(f_addr[11:LOGB]);
  wire [WIDTH-1:0] nb_a   = (f_beats_rem < WIDTH'(MAX_BURST)) ? f_beats_rem : WIDTH'(MAX_BURST);
  wire [WIDTH-1:0] nb_b   = (nb_a < to4k) ? nb_a : to4k;

  wire f_w_last_beat = (f_bcnt == f_nb - BURST_W'(1));
  wire f_last_overall = (f_beats_rem == WIDTH'(1));
  wire [BEAT_BYTES-1:0] f_strb = (f_last_overall && f_tail != '0) ? ((BEAT_BYTES'(1) << f_tail) - BEAT_BYTES'(1)) : {BEAT_BYTES{1'b1}};

  assign tile_rd_addr = TILE_BW'(f_beat_idx);

  always_ff @(posedge clk)
    if (!rstn) begin
      f_state <= F_IDLE; f_addr <= '0; f_beats_rem <= '0; f_beat_idx <= '0; f_tail <= '0; f_nb <= '0; f_bcnt <= '0; fl_done <= 1'b0;
    end else begin
      fl_done <= 1'b0;
      unique case (f_state)
        F_IDLE: if (fl_start) begin
          f_addr      <= AXI_ADDR_WIDTH'(p_out_base);
          f_beats_rem <= (p_out_bytes + WIDTH'(BEAT_BYTES - 1)) >> LOGB;
          f_tail      <= p_out_bytes[LOGB-1:0];
          f_beat_idx  <= '0;
          f_state     <= F_PLAN;
        end
        F_PLAN: begin
          if (f_beats_rem == '0) f_state <= F_FIN;
          else begin
            f_nb    <= BURST_W'(nb_b);
            f_bcnt  <= '0;
            f_state <= F_AW;
          end
        end
        F_AW: if (m_aw_ready) f_state <= F_WT;
        F_WT: f_state <= F_W;                                       // the tile RAM needs one clock for the read
        F_W: if (m_w_ready) begin
          f_bcnt      <= f_bcnt + BURST_W'(1);
          f_beat_idx  <= f_beat_idx + WIDTH'(1);
          f_beats_rem <= f_beats_rem - WIDTH'(1);
          if (f_w_last_beat) begin
            f_addr  <= f_addr + AXI_ADDR_WIDTH'(f_nb) * AXI_ADDR_WIDTH'(BEAT_BYTES);
            f_state <= f_last_overall ? F_FIN : F_PLAN;
          end else f_state <= F_WT;
        end
        F_FIN: if (outst == '0) begin
          fl_done <= 1'b1;
          f_state <= F_IDLE;
        end
        default: f_state <= F_IDLE;
      endcase
    end

  // ---- AXI channel mux (the pixel path and the flush never run at the same time) ----
  assign m_aw_valid = y_awvalid | f_awvalid;
  assign m_aw_addr  = y_active ? {y_addr[AXI_ADDR_WIDTH-1:LOGB], {LOGB{1'b0}}} : f_addr;
  wire [BURST_W-1:0] f_len = f_nb - BURST_W'(1);
  assign m_aw_len   = y_active ? 8'd0 : 8'(f_len);
  assign m_w_valid  = y_wvalid | f_wvalid;
  assign m_w_data   = y_active ? {(AXI_WIDTH/32){y_data}} : tile_rd;
  assign m_w_strb   = y_active ? y_strb : f_strb;
  assign m_w_last   = y_active ? 1'b1 : f_w_last_beat;
  assign m_b_ready  = 1'b1;

  always_ff @(posedge clk)
    if (!rstn) outst <= '0;
    else       outst <= outst + WIDTH'(aw_fire) - WIDTH'(b_fire);

  assign quiet = items_idle && !fl_busy && (outst == '0);

  // synthesis translate_off
  // ===================== simulation only: checks and dumps =====================
  // The files {ib}_y_nhwc_sim.txt (value per NHWC index) and {ib}_y_tiled_sim.txt (tiled elements) are checked by the python
  // flow, like the arrays the firmware used to keep. DATA_DIR comes from the plusarg +DATA_DIR=<dir>.
  string dd;
  bit    have_dd;
  initial have_dd = $value$plusargs("DATA_DIR=%s", dd);

  int dbg_nhwc [NHWC_WORDS];
  int dbg_tile [TILE_ELEMS];
  int dbg_nhwc_n, dbg_tile_n;
  initial begin dbg_nhwc_n = 0; dbg_tile_n = 0; end

  always @(posedge clk) if (rstn) begin
    if (t_state == T_B) begin
      if (idx >= NHWC_WORDS) $fatal(1, "[TILEWR] NHWC index %0d out of range (NHWC_WORDS=%0d)", idx, NHWC_WORDS);
      dbg_nhwc[idx] = q_val;
      if (int'(idx) + 1 > dbg_nhwc_n) dbg_nhwc_n = int'(idx) + 1;
      if (!p_add_base[WIDTH-1] && !p_is_last && (p_add_base + idx >= ADD_WORDS)) $fatal(1, "[TILEWR] add RAM address %0d out of range (ADD_WORDS=%0d)", p_add_base + idx, ADD_WORDS);
      if (p_is_last && p_is_softmax && q_val > (1 << SM_FRAC_BITS)) $fatal(1, "[TILEWR] softmax value %0d does not fit the exact float conversion", q_val);
    end
    if (tile_we) begin
      if (tile_addr >= TILE_ELEMS) $fatal(1, "[TILEWR] tile address %0d out of range (TILE_ELEMS=%0d) rd=%0d hr=%0d base0=%0d", tile_addr, TILE_ELEMS, rd, hr, base0);
      dbg_tile[tile_addr] = int'($signed(tile_data));
      if (int'(tile_addr) + 1 > dbg_tile_n) dbg_tile_n = int'(tile_addr) + 1;
    end
    if (fl_start && (p_out_bytes + WIDTH'(BEAT_BYTES - 1)) / WIDTH'(BEAT_BYTES) > TILE_BEATS) $fatal(1, "[TILEWR] flush of %0d bytes does not fit the tile RAM", p_out_bytes);
    if (dbg_dump) begin
      if (have_dd) begin
        int fd;
        fd = $fopen($sformatf("%s/%0d_y_nhwc_sim.txt", dd, dbg_ib), "w");
        for (int i = 0; i < dbg_nhwc_n; i++) $fdisplay(fd, "%0d", dbg_nhwc[i]);
        $fclose(fd);
        if (dbg_tile_n > 0) begin
          fd = $fopen($sformatf("%s/%0d_y_tiled_sim.txt", dd, dbg_ib), "w");
          for (int i = 0; i < dbg_tile_n; i++) $fdisplay(fd, "%0d", dbg_tile[i]);
          $fclose(fd);
        end
      end
      dbg_nhwc_n = 0; dbg_tile_n = 0;
    end
  end
  // synthesis translate_on

endmodule
