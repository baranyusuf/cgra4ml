`timescale 1ns/1ps

module writeback #(
  parameter
    SRAM_RD_DEPTH      = 8   , // number of bundles
    AXI_ADDR_WIDTH     = 32  ,
    AXI_DATA_WIDTH     = 32  ,
    AXI_WIDTH          = 128 ,
    Y_OUT_BITS      = 32 ,
    PE_ROWS         = 7  ,
    PE_COLS         = 96 ,
    KW_MAX          = 13 ,
    NHWC_WORDS      = 6272 ,  
    BIAS_WORDS      = 1    ,  // size of local bias RAM (words)
    ADD_WORDS       = 1    ,  // size of local residual-add RAM (bytes)
    SM_WORDS        = 1    ,  // output words of the softmax bundle (logit / probability RAM)
    SM_TAB_N        = 2220 ,  // entries of the softmax exp table
    POOL_WORDS      = 1    ,  // conv-output words of the largest pooled bundle (pool RAM, bytes)

  localparam  
    WIDTH              = 32  ,
    VAR_PER_ROW        = 32   ,
    SRAM_RD_DATA_WIDTH = VAR_PER_ROW * WIDTH,
    SRAM_WR_DEPTH = SRAM_RD_DEPTH * SRAM_RD_DATA_WIDTH / WIDTH, // 2048
    SRAM_RD_ADDR_WIDTH  = $clog2(SRAM_RD_DEPTH), // 11
    SRAM_WR_ADDR_WIDTH  = $clog2(SRAM_WR_DEPTH),
    N_VALS_PER_BEAT    = AXI_WIDTH / Y_OUT_BITS,
    RAW_BUF_DEPTH      = PE_ROWS * PE_COLS * (KW_MAX/2 + 1), 
    RAW_BUF_ADDR_WIDTH = $clog2(RAW_BUF_DEPTH),
    NHWC_ADDR_W = $clog2(NHWC_WORDS),
    BIAS_ADDR_W = (BIAS_WORDS > 1) ? $clog2(BIAS_WORDS) : 1,
    SM_ADDR_W   = (SM_WORDS > 1) ? $clog2(SM_WORDS) : 1,
    POOL_ADDR_W = (POOL_WORDS > 1) ? $clog2(POOL_WORDS) : 1
)(
  input  logic clk,
  input  logic rstn,

  // SRAM port
  input  logic                      reg_wr_en  ,
  input  logic [AXI_ADDR_WIDTH-1:0] reg_wr_addr,
  input  logic [AXI_DATA_WIDTH-1:0] reg_wr_data,
  input  logic                      reg_rd_en  ,
  input  logic [AXI_ADDR_WIDTH-1:0] reg_rd_addr, 
  output logic [AXI_DATA_WIDTH-1:0] reg_rd_data,

  // This signal is coming from one module above
  input out_tready_effective                   ,


  // AXIS slave interface 
  input  logic [AXI_WIDTH-1:0]      out_tdata  ,
  input  logic [AXI_WIDTH/8-1:0]    out_tkeep  ,
  input  logic                      out_tvalid ,
  output logic                      out_tready ,
  input  logic                      out_tlast
);
  assign out_tready = out_tready_effective;

  localparam N_REG = 32;

  // Addresses for local memory 0:32 is registers, rest is SRAM
  typedef enum int {
  `include "config_reg.def"
    WB_A_NUM_REGS
  } wb_reg_e;

  logic [WIDTH-1:0] cfg [N_REG-1:0];
  wire en_count = 1'(cfg[WB_A_EN_COUNT]);
  wire start    = 1'(cfg[WB_A_START]);
  wire [WIDTH-1:0] n_bundles_1 = cfg[WB_A_N_BUNDLES_1];

  //------------------- BUNDLES SRAM  ---------------------------------------

  localparam BIAS_BASE = N_REG + SRAM_WR_DEPTH; // bias RAM window starts right after the bundle SRAM window
  localparam ADD_BASE  = BIAS_BASE + ((BIAS_WORDS > 0) ? BIAS_WORDS : 1); // residual-add RAM window starts right after the bias RAM
  localparam ADD_ADDR_W = (ADD_WORDS > 1) ? $clog2(ADD_WORDS) : 1;
  localparam SM_TAB_BASE  = ADD_BASE + ((ADD_WORDS > 0) ? ADD_WORDS : 1); // softmax exp table window (SM_TAB_N words)
  localparam SM_TAB_AW    = $clog2(SM_TAB_N);

  logic ram_rd_en, ram_wr_en, ram_rd_valid;
  logic [SRAM_RD_ADDR_WIDTH-1:0] ram_rd_addr;
  logic [SRAM_WR_ADDR_WIDTH-1:0] ram_wr_addr;
  logic [WIDTH             -1:0] ram_wr_data;
  logic [SRAM_RD_DATA_WIDTH-1:0] ram_rd_data;

  asym_ram_sdp_read_wider #(
    .WIDTHB     (SRAM_RD_DATA_WIDTH),
    .SIZEB      (SRAM_RD_DEPTH     ),
    .ADDRWIDTHB (SRAM_RD_ADDR_WIDTH),
    .WIDTHA     (WIDTH             ),
    .SIZEA      (SRAM_WR_DEPTH     ),
    .ADDRWIDTHA (SRAM_WR_ADDR_WIDTH)
  ) sdp_ram (
    .clkA  (clk        ), 
    .clkB  (clk        ), 
    .weA   (ram_wr_en  ), 
    .enaA  (ram_wr_en  ), 
    .addrA (ram_wr_addr), 
    .diA   (ram_wr_data), 
    .enaB  (ram_rd_en  ), 
    .addrB (ram_rd_addr), 
    .doB   (ram_rd_data)
  );

  always_comb begin
    ram_wr_en   = reg_wr_en && (reg_wr_addr >= N_REG) && (reg_wr_addr < BIAS_BASE);
    ram_wr_addr = SRAM_WR_ADDR_WIDTH'(reg_wr_addr - N_REG);
    ram_wr_data = WIDTH'(reg_wr_data);
    // ram_rd_en   = reg_rd_en && (reg_rd_addr >= N_REG); Cannot read from CPU
    reg_rd_data = AXI_DATA_WIDTH'(cfg[reg_rd_addr[$clog2(N_REG)-1:0]]);
  end


  wire bank_end = l_icoe & l_ir & l_iw_last;
  logic bank_end_increment;

  



  //------------------- RAW VALUE PING-PONG BUFFER (BRAM, wide-write) -----------

  localparam RAW_WIDE_DEPTH  = (RAW_BUF_DEPTH + N_VALS_PER_BEAT - 1) / N_VALS_PER_BEAT;
  localparam RAW_WIDE_ADDR_W = $clog2(RAW_WIDE_DEPTH);

  logic [RAW_WIDE_ADDR_W-1:0] raw_buf_wr_word;  
  logic                       raw_ocm_bank;     
  logic                       raw_ocm_bank_rd;  

  wire [RAW_BUF_ADDR_WIDTH-1:0] raw_buf_rd_addr = i_sram_addr[RAW_BUF_ADDR_WIDTH-1:0];
  wire beat_handshake = out_tvalid && out_tready_effective;

  always_ff @(posedge clk)
    if (!rstn)               raw_ocm_bank_rd <= 1'b0;
    else if (rst_inner_rest) raw_ocm_bank_rd <= ~raw_ocm_bank_rd;

  always_ff @(posedge clk)
    if (!rstn) begin
      raw_buf_wr_word <= '0;
      raw_ocm_bank    <= 1'b0;
    end else if (beat_handshake) begin
      if (out_tlast) begin
        raw_buf_wr_word <= '0;
        raw_ocm_bank    <= ~raw_ocm_bank;
      end else
        raw_buf_wr_word <= raw_buf_wr_word + RAW_WIDE_ADDR_W'(1);
    end

  wire wr_b0 = beat_handshake && (raw_ocm_bank == 1'b0);
  wire wr_b1 = beat_handshake && (raw_ocm_bank == 1'b1);
  wire [Y_OUT_BITS-1:0] raw_rd_b0, raw_rd_b1;

  asym_ram_sdp_write_wider #(
    .WIDTHA(AXI_WIDTH), .SIZEA(RAW_WIDE_DEPTH), .ADDRWIDTHA(RAW_WIDE_ADDR_W),
    .WIDTHB(Y_OUT_BITS), .SIZEB(RAW_BUF_DEPTH), .ADDRWIDTHB(RAW_BUF_ADDR_WIDTH)
  ) RAW_RAM_0 (
    .clkA(clk), .clkB(clk),
    .enaA(wr_b0), .weA(wr_b0), .addrA(raw_buf_wr_word), .diA(out_tdata),
    .enaB(1'b1), .addrB(raw_buf_rd_addr), .doB(raw_rd_b0)
  );

  asym_ram_sdp_write_wider #(
    .WIDTHA(AXI_WIDTH), .SIZEA(RAW_WIDE_DEPTH), .ADDRWIDTHA(RAW_WIDE_ADDR_W),
    .WIDTHB(Y_OUT_BITS), .SIZEB(RAW_BUF_DEPTH), .ADDRWIDTHB(RAW_BUF_ADDR_WIDTH)
  ) RAW_RAM_1 (
    .clkA(clk), .clkB(clk),
    .enaA(wr_b1), .weA(wr_b1), .addrA(raw_buf_wr_word), .diA(out_tdata),
    .enaB(1'b1), .addrB(raw_buf_rd_addr), .doB(raw_rd_b1)
  );

  wire [Y_OUT_BITS-1:0] raw_value_rd = raw_ocm_bank_rd ? raw_rd_b1 : raw_rd_b0;

  //------------------- END RAW VALUE BUFFER ------------------------------------



// ===================== DIVIDER-FREE STRIDED REDUCTION =======================

  wire signed [WIDTH-1:0] s_csh = $signed(ram_csh), s_csw = $signed(ram_csw);
  wire signed [WIDTH-1:0] s_csh_shift = $signed(ram_csh_shift);
  wire signed [WIDTH-1:0] s_csw_shift = $signed(ram_csw_shift);
  wire signed [WIDTH-1:0] s_pe_rows   = $signed(ram_PE_ROWS);
  localparam signed [WIDTH-1:0] ONE = $signed(WIDTH'(1));
  wire signed [WIDTH-1:0] s_pe_div = $signed(ram_pe_div);  
  wire signed [WIDTH-1:0] s_pe_mod = $signed(ram_pe_mod); 

  logic signed [WIDTH-1:0] bh_q, bh_r, bw_q, bw_r;
  logic signed [WIDTH-1:0] nbh_q, nbh_r, nbw_q, nbw_r;

  always_comb begin
    nbh_q = bh_q; nbh_r = bh_r;
    nbw_q = bw_q; nbw_r = bw_r;

    if (ram_rd_valid || (lc_wkw2 && l_l)) begin          
      nbh_q = (s_csh_shift == 0) ? '0 : -ONE;
      nbh_r = (s_csh_shift == 0) ? '0 : (s_csh - s_csh_shift);
    end else if (lc_wkw2) begin                         
      if (s_csh == ONE) begin
        nbh_q = bh_q + s_pe_rows;  nbh_r = '0;
      end else begin
        nbh_q = bh_q + s_pe_div;                         
        nbh_r = bh_r + s_pe_mod;
        if (nbh_r >= s_csh) begin nbh_r -= s_csh; nbh_q += 1; end
      end
    end

   
    if (ram_rd_valid || (en_count && l_wkw2)) begin
      nbw_q = (s_csw_shift == 0) ? '0 : -ONE;
      nbw_r = (s_csw_shift == 0) ? '0 : (s_csw - s_csw_shift);
    end else if (en_count) begin                       
      if (s_csw == ONE) begin
        nbw_q = bw_q + 1;  nbw_r = '0;
      end else if (bw_r + ONE == s_csw) begin
        nbw_r = '0;        nbw_q = bw_q + 1;
      end else begin
        nbw_r = bw_r + ONE; nbw_q = bw_q;
      end
    end
  end

  always_ff @(posedge clk)
    if (!rstn) begin bh_q<='0; bh_r<='0; bw_q<='0; bw_r<='0; end
    else      begin bh_q<=nbh_q; bh_r<=nbh_r; bw_q<=nbw_q; bw_r<=nbw_r; end

  logic signed [WIDTH-1:0] h_q, h_r, w_q, w_r;
  logic signed [WIDTH-1:0] nh_q, nh_r, nw_q, nw_r;

  always_comb begin
    nh_q = h_q; nh_r = h_r;
    nw_q = w_q; nw_r = w_r;

   
    if (rst_inner) begin                     
      nh_q = nbh_q;
      nh_r = (s_csh == ONE) ? '0 : nbh_r;
    end else if (lc_wkw2) begin               
      nh_q = nbh_q;
      nh_r = (s_csh == ONE) ? '0 : nbh_r;
    end else if (lc_ir) begin              
      nh_q = nbh_q;
      nh_r = (s_csh == ONE) ? '0 : nbh_r;
    end else if (en_counter_last) begin      
      if (s_csh == ONE)            begin nh_q = h_q + 1;   nh_r = '0;        end
      else if (h_r + ONE == s_csh) begin nh_q = h_q + 1;   nh_r = '0;        end
      else                         begin nh_q = h_q;       nh_r = h_r + ONE; end
    end

    if (rst_inner) begin
      nw_q = nbw_q;
      nw_r = (s_csw == ONE) ? '0 : nbw_r;
    end else if (lc_iw_last) begin           
      nw_q = bw_q;
      nw_r = (s_csw == ONE) ? '0 : bw_r;
    end else if (lc_ir) begin                
      if (s_csw == ONE)            begin nw_q = w_q + 1;   nw_r = '0;        end
      else if (w_r + ONE == s_csw) begin nw_q = w_q + 1;   nw_r = '0;        end
      else                         begin nw_q = w_q;       nw_r = w_r + ONE; end
    end
  end

  always_ff @(posedge clk)
    if (!rstn) begin h_q<='0; h_r<='0; w_q<='0; w_r<='0; end
    else      begin h_q<=nh_q; h_r<=nh_r; w_q<=nw_q; w_r<=nw_r; end

  wire signed [WIDTH-1:0] ch_quot = h_q;     
  wire signed [WIDTH-1:0] cw_quot = w_q;
  wire conv_stride_invalid = ((h_r != 0) || (w_r != 0)) && (!hold_invalid);
  // ============================================================================

  

  
  //------------------- NHWC ACCUMULATOR MEMORY (BRAM, proven SDP template) -----

    logic [NHWC_ADDR_W-1:0] nhwc_wr_addr;
    logic [WIDTH-1:0]       nhwc_wr_data;
    logic                   nhwc_wr_en;
    logic [NHWC_ADDR_W-1:0] nhwc_rd_addr;
    logic [WIDTH-1:0]       nhwc_rd_q;    

    ram_sdp_simple #(
      .WIDTH (WIDTH), .DEPTH (NHWC_WORDS), .ADDRW (NHWC_ADDR_W)
    ) NHWC_RAM (
      .clkA  (clk         ), .clkB  (clk         ),
      .weA   (nhwc_wr_en  ), .enaA  (nhwc_wr_en  ), .addrA (nhwc_wr_addr), .diA (nhwc_wr_data),
      .enaB  (1'b1        ), .addrB (nhwc_rd_addr), .doB (nhwc_rd_q)
    );

  //------------------- END NHWC ACCUMULATOR MEMORY ----------------------------


  localparam logic [1:0]
    PASS_ONLY   = 2'd0,   // p==1
    PASS_FIRST  = 2'd1,   // ip==0  (ve p!=1)
    PASS_MIDDLE = 2'd2,   // 0<ip<p-1
    PASS_LAST   = 2'd3;   // ip==p-1 (ve p!=1)

  wire only_p_now  = (ram_max_p == '0);          
  wire last_p_now  = l_p;                        
  wire first_p_now = (i_p == '0);               

  logic [1:0] pass_type;
  always_comb begin
    if      (only_p_now)  pass_type = PASS_ONLY;
    else if (last_p_now)  pass_type = PASS_LAST;
    else if (first_p_now) pass_type = PASS_FIRST;
    else                  pass_type = PASS_MIDDLE;
  end
  //------------------- END P-PASS TYPE ----------------------------------------


  logic acc_pass;
  wire  acc_pass_last = acc_pass && wb_a_done_write_any && (~pixel_invalid) && (!hold_invalid) && (!addr_en_counter_hold);


  always_ff @(posedge clk) begin
    if (!rstn)
      acc_pass <= 1'b0;
    else if (acc_pass_last && (!(en_counter_last)))
      acc_pass <= 1'b0;
    else if (!acc_pass)
      acc_pass <= start_increment_2 || en_counter_last;
  end

  assign nhwc_rd_addr = iy_nhwc_hw[NHWC_ADDR_W-1:0];

  // S0 -> S1 pipeline
  logic                   s1_valid;
  logic [1:0]             s1_type;
  logic [NHWC_ADDR_W-1:0] s1_idx;

  always_ff @(posedge clk)
    if (!rstn) begin
      s1_valid <= 1'b0; s1_type <= PASS_ONLY;  s1_idx <= '0;
    end else begin
      s1_valid <= acc_pass_last;
      s1_type  <= pass_type;
      s1_idx   <= nhwc_rd_addr;
    end

  // S1: case
  wire use_acc = (s1_type == PASS_MIDDLE) || (s1_type == PASS_LAST);
  wire [WIDTH-1:0] acc_sum = use_acc ? (nhwc_rd_q + WIDTH'(raw_value_rd)) : WIDTH'(raw_value_rd);

  always_comb begin
    nhwc_wr_en   = en_counter_acc;
    nhwc_wr_addr = s1_idx;
    nhwc_wr_data = acc_sum;
  end

  wire [WIDTH-1:0] sum_val     = acc_sum;

  wire             sum_proceed = (s1_type == PASS_ONLY) || (s1_type == PASS_LAST);

  wire             en_counter_acc = s1_valid && ((s1_type == PASS_FIRST) || (s1_type == PASS_MIDDLE));
//------------------- END NHWC P-PASS PROCEDURE ------------------------------


  // ram_ are combinational from ram
  logic [WIDTH-1:0] ram_max_wkw2, ram_max_l, ram_max_n, ram_max_t, ram_max_p, ram_b_offset      ;
  logic [WIDTH-1:0] ram_coe     , ram_PE_ROWS , ram_kw                                          ;
  logic [WIDTH-1:0] ram_h, ram_w, ram_co                                                        ;
  logic [WIDTH-1:0] ram_csh, ram_csh_shift, ram_csw, ram_csw_shift                              ;
  logic [WIDTH-1:0] ram_ch, ram_cw;


  logic [WIDTH-1:0] i_wkw2      , i_l      , i_n      , i_t      , i_p      , i_b ;
  logic             lc_wkw2     , lc_l     , lc_n     , lc_t     , lc_p     , lc_b;
  logic             l_wkw2      , l_l      , l_n      , l_t      , l_p      , l_b ;

  logic [WIDTH-1:0] ram_pe_div  , ram_pe_mod;
  logic [WIDTH-1:0] ram_is_bias , ram_b_val_shift, ram_b_bias_shift;
  logic [WIDTH-1:0] ram_ca_nzero, ram_ca_shift, ram_ca_pl_scale, ram_clip_min, ram_clip_max;
  logic [WIDTH-1:0] ram_add_base, ram_aa_pack, ram_aa_clip_min, ram_add_shifts;
  wire              ram_is_softmax = ram_add_shifts[16]; // bundle word 31 bit 16: this bundle's outputs go through the PL softmax
  // Pooling parameters sit in the unused upper bits of the header words 20..25 (the low byte of each word keeps its old field):
  //   word 20 [9:8] pool mode (0 none, 1 max, 2 avg)
  //   word 21 [15:8] pkh   [23:16] pkw   [31:24] psh_shift       (low bits: b_val_shift)
  //   word 22 [15:8] psh   [23:16] psw   [31:24] psw_shift       (low bits: b_bias_shift)
  //   word 23 [8] pa_nzero                                       (bit 0: ca_nzero)
  //   word 24 [12:8] pa_shift   [25:16] ph (pooled height)       (low bits: ca_shift)
  //   word 25 [12:8] pa_pl_scale [25:16] pw (pooled width)       (low bits: ca_pl_scale)
  wire [1:0]        ram_pool_mode   = ram_is_bias[9:8];
  wire              ram_is_pool     = (ram_pool_mode != 2'd0);
  wire [7:0]        ram_pkh = ram_b_val_shift [15:8], ram_pkw = ram_b_val_shift [23:16], ram_psh_shift = ram_b_val_shift [31:24];
  wire [7:0]        ram_psh = ram_b_bias_shift[15:8], ram_psw = ram_b_bias_shift[23:16], ram_psw_shift = ram_b_bias_shift[31:24];
  wire              ram_pa_nzero    = ram_ca_nzero[8];
  wire [4:0]        ram_pa_shift    = ram_ca_shift[12:8];
  wire [9:0]        ram_ph          = ram_ca_shift[25:16];
  wire [4:0]        ram_pa_pl_scale = ram_ca_pl_scale[12:8];
  wire [9:0]        ram_pw          = ram_ca_pl_scale[25:16];

  localparam ACTUAL_READ_WIDTH = 32 * WIDTH;
  assign ram_rd_addr = SRAM_RD_ADDR_WIDTH'(i_b);
  wire last_wkw2_now = (i_wkw2 == ram_max_wkw2);  
  wire [WIDTH-1:0] w_last_hw = last_wkw2_now ? ((ram_kw >> 1) + 32'd1) : 32'd1;

  logic [SRAM_RD_DATA_WIDTH-1:0] ram_rd_data_q;
  always_ff @(posedge clk)
    if (!rstn) ram_rd_data_q <= '0;
    else       ram_rd_data_q <= ram_rd_data; 

  assign {ram_add_shifts, ram_aa_clip_min, ram_aa_pack, ram_add_base,
          ram_clip_max, ram_clip_min, ram_ca_pl_scale, ram_ca_shift, ram_ca_nzero,
          ram_b_bias_shift, ram_b_val_shift, ram_is_bias,
          ram_pe_mod, ram_pe_div,
          ram_cw, ram_ch, ram_csw_shift, ram_csw, ram_csh_shift, ram_csh, ram_co,
          ram_w, ram_h, ram_b_offset, ram_kw, ram_PE_ROWS, ram_coe,
          ram_max_wkw2, ram_max_l, ram_max_n, ram_max_t, ram_max_p}
        = ram_rd_data_q[ACTUAL_READ_WIDTH-1:0];


  logic [WIDTH-1:0] hw_co_q;     
  logic [WIDTH-1:0] w_co_q; 
  logic [WIDTH-1:0] chw_co_q;   
  logic [WIDTH-1:0] cw_co_q;     
  logic [WIDTH-1:0] co_q;     
  logic [WIDTH-1:0] coe_t_q;


  logic [WIDTH-1:0] hw_q, chw_q;
  always_ff @(posedge clk)
    if (!rstn) begin
      hw_q <= '0; hw_co_q <= '0; chw_q <= '0; chw_co_q <= '0;
      w_co_q <= '0; cw_co_q <= '0; co_q <= '0; coe_t_q <= '0;
    end else begin
      hw_q     <= ram_h  * ram_w;       
      hw_co_q  <= hw_q   * ram_co;      
      chw_q    <= ram_ch * ram_cw;
      chw_co_q <= chw_q  * ram_co;
      w_co_q   <= ram_w  * ram_co;
      cw_co_q  <= ram_cw * ram_co;
      co_q     <= ram_co;
      coe_t_q  <= ram_coe * i_t;
    end     


  wire [WIDTH-1:0] iyn = i_n;
  wire [WIDTH-1:0] iyh = il_dot_pe + i_ir;
  wire [WIDTH-1:0] iyw = i_wkw2 + i_iw_last;
  wire [WIDTH-1:0] iyc = coe_t_q + i_icoe;
  logic [WIDTH-1:0] iyn_hwco_q;
  always_ff @(posedge clk)
    if (!rstn) iyn_hwco_q <= '0;
    else       iyn_hwco_q <= iyn * hw_co_q;

  logic [WIDTH-1:0] iyh_wco_q, iyw_co_q, iyc_sum1;
  always_ff @(posedge clk)
    if (!rstn) begin
      iyh_wco_q <= '0; iyw_co_q <= '0; iyc_sum1 <= '0;
    end else begin
      iyh_wco_q   <= iyh * w_co_q;    
      iyw_co_q    <= iyw * co_q;     
      iyc_sum1      <= iyc + iyn_hwco_q;             
    end

  logic [WIDTH-1:0] iy_nhwc_hw;
  always_ff @(posedge clk)
    if (!rstn) iy_nhwc_hw <= '0;
    else       iy_nhwc_hw <= iyc_sum1 + iyh_wco_q + iyw_co_q;

  logic [WIDTH-1:0] il_dot_pe;
  always_ff @(posedge clk)
    if (!rstn) il_dot_pe <= '0;
    else       il_dot_pe <= i_l * ram_PE_ROWS;

  logic signed [WIDTH-1:0] ch_quot_q, cw_quot_q;
  always_ff @(posedge clk)
    if (!rstn) begin
      ch_quot_q <= '0; cw_quot_q <= '0;
    end else begin
      ch_quot_q <= ch_quot;
      cw_quot_q <= cw_quot;
    end

  logic [WIDTH-1:0] cv_m0, cv_m1, cv_m2, cv_iyc0;
  always_ff @(posedge clk)
    if (!rstn) begin
      cv_m0 <= '0; cv_m1 <= '0; cv_m2 <= '0; cv_iyc0 <= '0;
    end else begin
      cv_m0   <= iyn        * chw_co_q;   
      cv_m1   <= ch_quot_q  * cw_co_q;    
      cv_m2   <= cw_quot_q  * co_q;       
      cv_iyc0 <= iyc;
    end


  logic [WIDTH-1:0] cv_s0, cv_s1;
  always_ff @(posedge clk)
    if (!rstn) begin
      cv_s0 <= '0; cv_s1 <= '0;
    end else begin
      cv_s0 <= cv_m0 + cv_m1;
      cv_s1 <= cv_m2 + cv_iyc0;
    end

  logic [WIDTH-1:0] iy_nhwc_conv;
  always_ff @(posedge clk)
    if (!rstn) iy_nhwc_conv <= '0;
    else       iy_nhwc_conv <= cv_s0 + cv_s1;         



  // ===================== PIXEL OUTPUT PIPELINE  ========
  logic [WIDTH-1:0] pack_p, pack2_p, outval_p;

  // ---- BIAS (PL): out = (sum << b_val_shift) + (bias << b_bias_shift), when ram_is_bias ----
  // bias RAM is filled by the PS over AXI-Lite (window starts at BIAS_BASE). Read address i_bias
  // is stable well before sum_val, so the RAM read data is ready in time. One extra register
  // stage (sum_q) keeps the shift+add off the RAM->adder critical path; addr_en_counter_delay
  // is one cycle longer to cover it.
  logic                   bias_wr_en;
  logic [BIAS_ADDR_W-1:0] bias_wr_addr, bias_rd_addr;
  logic [WIDTH-1:0]       bias_rd_q;
  always_comb begin
    bias_wr_en   = reg_wr_en && (reg_wr_addr >= BIAS_BASE) && (reg_wr_addr < ADD_BASE);
    bias_wr_addr = BIAS_ADDR_W'(reg_wr_addr - BIAS_BASE);
    bias_rd_addr = BIAS_ADDR_W'(i_bias);
  end

  ram_sdp_simple #(
    .WIDTH (WIDTH), .DEPTH ((BIAS_WORDS > 0) ? BIAS_WORDS : 1), .ADDRW (BIAS_ADDR_W)
  ) BIAS_RAM (
    .clkA (clk), .clkB (clk),
    .weA  (bias_wr_en), .enaA (bias_wr_en), .addrA (bias_wr_addr), .diA (WIDTH'(reg_wr_data)),
    .enaB (1'b1), .addrB (bias_rd_addr), .doB (bias_rd_q)
  );

  logic [WIDTH-1:0] sum_q;
  always_ff @(posedge clk)
    if (!rstn) sum_q <= '0;
    else       sum_q <= sum_val;

  wire signed [WIDTH-1:0] sum_sh  = $signed(sum_q    ) <<< ram_b_val_shift [4:0];
  wire signed [WIDTH-1:0] bias_sh = $signed(bias_rd_q) <<< ram_b_bias_shift[4:0];
  wire        [WIDTH-1:0] sum_biased = ram_is_bias[0] ? WIDTH'(sum_sh + bias_sh) : sum_q;

  // ---- CORE ACT (PL): quant_lrelu(ca_*) -- see wb_quant_lrelu.sv (4 register stages) ----
  logic signed [WIDTH-1:0] act1_y;
  wb_quant_lrelu #(.WIDTH(WIDTH)) CORE_ACT (
    .clk(clk), .rstn(rstn),
    .x        ($signed(WIDTH'(sum_biased))),
    .nzero    (ram_ca_nzero[0]),
    .shift    (ram_ca_shift   [4:0]),
    .pl_scale (ram_ca_pl_scale[4:0]),
    .clip_min ($signed(ram_clip_min)),
    .clip_max ($signed(ram_clip_max)),
    .y        (act1_y)
  );

  // ---- RESIDUAL ADD (PL): out = quant_lrelu(act1 + add_buffer[iy_nhwc], aa_*) ----
  // add RAM (bytes) is filled by the PS over AXI-Lite (window starts at ADD_BASE) with the
  // outputs of earlier bundles. ram_add_base = buffer_idx*NHWC_WORDS, or -1 when this bundle
  // has no residual add. Without residual add the value goes through a pass-through
  // quant_lrelu (nzero=1, shift=0, pl_scale=0, clip = full range), so latency stays constant.
  logic                   add_wr_en;
  logic [ADD_ADDR_W-1:0]  add_wr_addr, add_rd_addr;
  logic [7:0]             add_rd_q;
  always_comb begin
    add_wr_en   = reg_wr_en && (reg_wr_addr >= ADD_BASE) && (reg_wr_addr < SM_TAB_BASE);
    add_wr_addr = ADD_ADDR_W'(reg_wr_addr - ADD_BASE);
    add_rd_addr = ADD_ADDR_W'(ram_add_base + iy_nhwc_conv);
  end

  ram_sdp_simple #(
    .WIDTH (8), .DEPTH ((ADD_WORDS > 0) ? ADD_WORDS : 1), .ADDRW (ADD_ADDR_W)
  ) ADD_RAM (
    .clkA (clk), .clkB (clk),
    .weA  (add_wr_en), .enaA (add_wr_en), .addrA (add_wr_addr), .diA (reg_wr_data[7:0]),
    .enaB (1'b1), .addrB (add_rd_addr), .doB (add_rd_q)
  );

  wire                    has_add = !ram_add_base[WIDTH-1];
  // Align the two operands first (Python: x.add_val_shift(other)): main branch << add_val_shift,
  // stored branch << add_a_shift. Shift and add are separate register stages for timing.
  wire [4:0] add_vs = has_add ? ram_add_shifts[4:0]  : 5'd0;
  wire [4:0] add_as = has_add ? ram_add_shifts[12:8] : 5'd0;
  logic signed [WIDTH-1:0] add_x_q, add_a_q, add_q;
  always_ff @(posedge clk)
    if (!rstn) begin
      add_x_q <= '0; add_a_q <= '0; add_q <= '0;
    end else begin
      add_x_q <= act1_y <<< add_vs;
      add_a_q <= has_add ? (WIDTH'($signed(add_rd_q)) <<< add_as) : '0;
      add_q   <= add_x_q + add_a_q;
    end

  wb_quant_lrelu #(.WIDTH(WIDTH)) ADD_ACT (
    .clk(clk), .rstn(rstn),
    .x        (add_q),
    .nzero    (has_add ? ram_aa_pack[0]       : 1'b1),
    .shift    (has_add ? ram_aa_pack[12:8]    : 5'd0),
    .pl_scale (has_add ? ram_aa_pack[20:16]   : 5'd0),
    .clip_min (has_add ? $signed(ram_aa_clip_min) : 32'sh8000_0000),
    .clip_max (has_add ? $signed(ram_clip_max)    : 32'sh7FFF_FFFF),
    .y        (outval_p)
  );

  // ---- SOFTMAX (PL): see wb_softmax.sv ----
  // In a softmax bundle the PS never sees the logits. When a logit pixel becomes valid, the PL accepts it
  // itself (sm_auto_ack = the ack the PS would have written to WB_A_ADDR_EN_COUNTER) and stores it in the
  // softmax unit. The class with index co-1 is the last one of its pixel -> starts max / exp / sum / divide.
  // The result is then presented to the PS like any other output pixel (OUT_FLAGS / PIXEL_PACK / PIXEL_PACK2, one
  // class per pixel), already rounded to the input format of the next bundle, so the PS just runs tile_write.
  // New logit pixels are not accepted while the unit is working (the counters simply wait), and the
  // done flag is hidden until the last result has been taken by the PS.
  // ---- Pipeline latencies (clocks), counted from en_counter_last (the counters move to the next pixel) ----
  //  data path of the new pixel:
  //    raw RAM:   i_sram_addr counter register (1) -> RAW RAM read (1)                      = LAT_ADDR_RD   = 2
  //    NHWC RAM:  pixel step (i_ir / i_iw_last / i_icoe move):
  //                 counter register (1) -> iyh_wco_q/iyw_co_q/iyc_sum1 (1) -> iy_nhwc_hw (1) -> NHWC RAM read (1)  = LAT_NHWC_RD   = 4
  //               bank step (PS writes WB_A_EN_COUNT: i_wkw2 / i_l / i_n / i_t move, and the pixel counters restart):
  //                 counter register (1) -> il_dot_pe/coe_t_q/iyn_hwco_q (1) -> iyh_wco_q/iyc_sum1 (1) -> iy_nhwc_hw (1)
  //                 -> NHWC RAM read (1)                                                                          = LAT_NHWC_BANK = 5
  //               (the last pass of a p>1 layer adds the stored partial sum, so it needs the slower NHWC path)
  //    sum_val is valid LAT_SUM clocks after the counters moved
  //  sum_val -> outval_p:  sum_q (1) -> core act (4) -> add align + add (2) -> add act (4)  = LAT_PIPE      = 11
  //  flag path: the hold ends at +D, then hold visible (1) -> clear_invalid (1) -> cfg[ADDR_INVALID] (1)  = LAT_TAIL = 3
  //  The value is sampled in the same clock the flag falls, so  LAT_SUM + LAT_PIPE <= D + LAT_TAIL.
  localparam             LAT_BIAS_REG = 1;
  localparam             LAT_QL       = 4;                  // wb_quant_lrelu: lk_q, sum_q, sh_q, y
  localparam             LAT_ADD_REG  = 2;                  // add_x_q / add_a_q, add_q
  localparam             LAT_PIPE     = LAT_BIAS_REG + LAT_QL + LAT_ADD_REG + LAT_QL;   // = 11
  localparam             LAT_ADDR_RD  = 2;
  localparam             LAT_NHWC_RD  = 4;
  localparam             LAT_NHWC_BANK = 5;
  localparam             LAT_SUM      = (LAT_NHWC_RD > LAT_ADDR_RD) ? LAT_NHWC_RD : LAT_ADDR_RD;   // = 4
  localparam             LAT_TAIL     = 3;
  localparam             EN_CNT_DELAY = LAT_SUM + LAT_PIPE - LAT_TAIL;                  // = 12 (tightest value), after en_counter_last
  //  After a bank step the flag is released by hold_invalid (HOLD_INV_DELAY clocks after WB_A_EN_COUNT) instead, the same rule:
  localparam             LAT_SUM_BANK   = (LAT_NHWC_BANK > LAT_ADDR_RD) ? LAT_NHWC_BANK : LAT_ADDR_RD;   // = 5
  localparam             HOLD_INV_DELAY = LAT_SUM_BANK + LAT_PIPE - LAT_TAIL;       // = 13
  //  The PS used to wait for the raw bank to be written (BUNDLE_STATUS) before it looked at any pixel. The PL now does the
  //  same: a pixel is accepted only when the raw bank that is being read has been written completely (DONE_WRITE_x).
  //  The value is then ready LAT_RAM_WR_RD (RAM write -> read data) + LAT_PIPE clocks after the later of: flag low, bank written.
  localparam             LAT_RAM_WR_RD = 1;
  localparam             PX_SETTLE     = LAT_RAM_WR_RD + LAT_PIPE;                       // = 12
  localparam             PX_VCNT_W     = $clog2(PX_SETTLE + 1) + 1;
  wire                   px_bank_written = raw_ocm_bank_rd ? cfg[WB_A_DONE_WRITE_1][0] : cfg[WB_A_DONE_WRITE_0][0];
  wire                   px_not_ready  = cfg[WB_A_ADDR_INVALID][0] || !px_bank_written;
  logic [PX_VCNT_W-1:0]  px_vcnt;
  logic                  px_arm;
  logic                  sm_idle, sm_em_valid;
  logic                  pool_idle, pool_em_valid, pool_done;
  logic [WIDTH-1:0]      sm_em_data, sm_em_k;
  wire                   px_valid = !px_not_ready && (px_vcnt == PX_VCNT_W'(PX_SETTLE)) && !cfg[WB_A_ADDR_DONE][0];   // outval_p has left the pipeline
  wire                   sm_auto_ack  = ram_is_softmax && px_arm && px_valid && sm_idle && !sm_em_valid;
  // The PS acknowledges every emitted result (softmax class / pooled pixel) with a write to ADDR_EN_COUNTER, which must not move the counters.
  wire                   em_any_valid = sm_em_valid || pool_em_valid;
  wire                   em_ack_any   = reg_wr_en && (reg_wr_addr == WB_A_ADDR_EN_COUNTER) && reg_wr_data[0] && em_any_valid;
  wire                   sm_em_ack    = em_ack_any && sm_em_valid;
  wire                   pool_em_ack  = em_ack_any && pool_em_valid;
  // Pooling: the pixel is accepted like a softmax logit, but the counters are held (no ack) until the pool unit has swept all
  // windows of the pixel and the PS has taken every pooled pixel it emitted (pool_done), so the pixel's values stay stable meanwhile.
  wire                   pool_start   = ram_is_pool && px_arm && px_valid && pool_idle && !pool_em_valid;
  wire [21:0]            sm_cls       = pack2_p[31:10];                     // class index (iyc) of the pixel being accepted
  // The classes of a pixel arrive in ascending order (iyc = coe_t_q + i_icoe, both counters only count up, tiles t ascend),
  // so the logit of class co-1 is always the last one of its pixel and the pixel's logits end at its NHWC address.
  wire                   sm_last      = (WIDTH'(sm_cls) == ram_co - 1);
  logic [WIDTH-1:0]      sm_em_pack;
  logic [9:0]            sm_em_w;
  always_ff @(posedge clk)
    if (!rstn) begin
      px_vcnt <= '0; px_arm <= 1'b0; sm_em_pack <= '0; sm_em_w <= '0;
    end else begin
      if (px_not_ready)                        px_vcnt <= '0;
      else if (px_vcnt != PX_VCNT_W'(PX_SETTLE)) px_vcnt <= px_vcnt + PX_VCNT_W'(1);
      if (cfg[WB_A_ADDR_INVALID][0]) px_arm <= 1'b1;   // armed again once the counters have moved on
      else if (sm_auto_ack || pool_start) px_arm <= 1'b0;   // one pixel is accepted once
      if (sm_auto_ack && sm_last) begin                // coordinates of the pixel whose results will be emitted
        sm_em_pack <= pack_p;
        sm_em_w    <= pack2_p[9:0];
      end
    end


  wb_softmax #(.WIDTH(WIDTH), .WORDS(SM_WORDS), .TAB_N(SM_TAB_N)) SOFTMAX (
    .clk(clk), .rstn(rstn),
    .cap_valid(sm_auto_ack), .cap_addr(SM_ADDR_W'(iy_nhwc_conv)), .cap_logit(outval_p[7:0]),
    .cap_last(sm_last), .cap_ncls(ram_co), .cap_shift(ram_add_shifts[23:20]),
    .cap_quant(ram_add_shifts[17]), .cap_ofrac(ram_add_shifts[27:24]), .em_qmax(ram_clip_max),
    .tab_we(reg_wr_en && (reg_wr_addr >= SM_TAB_BASE) && (reg_wr_addr < SM_TAB_BASE + SM_TAB_N)),
    .tab_addr(SM_TAB_AW'(reg_wr_addr - SM_TAB_BASE)), .tab_data(WIDTH'(reg_wr_data)),
    .em_valid(sm_em_valid), .em_data(sm_em_data), .em_k(sm_em_k), .em_ack(sm_em_ack),
    .idle(sm_idle)
  );

  // ---- POOLING (PL): see wb_pool.sv ----
  logic [WIDTH-1:0] pool_em_data;
  logic [9:0]       pool_em_h, pool_em_w;
  logic [21:0]      pool_em_c;
  logic [4:0]       pool_em_n;
  wb_pool #(.WIDTH(WIDTH), .WORDS(POOL_WORDS)) POOL (
    .clk(clk), .rstn(rstn),
    .start(pool_start), .pix_addr(POOL_ADDR_W'(iy_nhwc_conv)), .pix_val(outval_p[7:0]),
    .pix_h(pack_p[14:5]), .pix_w(pack2_p[9:0]), .pix_c(pack2_p[31:10]), .pix_n(pack_p[4:0]),
    .mode_avg(ram_pool_mode == 2'd2), .yh(ram_ch), .yw(ram_cw), .step_c(ram_co), .step_h(cw_co_q),
    .pkh(ram_pkh), .pkw(ram_pkw), .psh(ram_psh), .psw(ram_psw), .psh_shift(ram_psh_shift), .psw_shift(ram_psw_shift),
    .ph(ram_ph), .pw(ram_pw),
    .pa_nzero(ram_pa_nzero), .pa_shift(ram_pa_shift), .pa_pl_scale(ram_pa_pl_scale),
    .pa_clip_min(-$signed((ram_clip_max + WIDTH'(1)) >> ram_pa_pl_scale)), .pa_clip_max($signed(ram_clip_max)),
    .em_valid(pool_em_valid), .em_data(pool_em_data), .em_h(pool_em_h), .em_w(pool_em_w), .em_c(pool_em_c), .em_n(pool_em_n),
    .em_ack(pool_em_ack),
    .idle(pool_idle), .done(pool_done)
  );

  always_ff @(posedge clk)
    if (!rstn) begin
      pack_p<='0; pack2_p<='0;
    end else begin
      pack_p   <= WIDTH'(WIDTH'(iyn[4:0]) | (WIDTH'(ch_quot[9:0]) << 5) | (WIDTH'(i_bias) << 15));
      pack2_p  <= WIDTH'(WIDTH'(cw_quot[9:0]) | (WIDTH'(iyc[21:0]) << 10));
    end
  // ============================================================================

  logic start_increment;
  logic start_increment_2;
  logic valid_increment;


  always_ff @(posedge clk)
    if (!rstn) begin
      ram_rd_en         <= 0;
      ram_rd_valid      <= 0;
      start_increment   <= 0;
      start_increment_2 <= 0;
      valid_increment   <= 0;
    end else begin
      ram_rd_en    <= lc_p || start; // ib gets updated at the cycle after lc_p
      start_increment <= start;
      start_increment_2 <= start_increment;
      valid_increment <= ram_rd_en;
      ram_rd_valid <= valid_increment;
    end

  up_counter #(.W(WIDTH)) C_B    (.clk(clk), .rstn_g(rstn), .rst_l(start           ), .en(lc_p    ), .max_in(n_bundles_1 ), .last_clk(lc_b   ), .last(l_b   ), .first(), .count(i_b   ));
  up_counter #(.W(WIDTH)) C_P    (.clk(clk), .rstn_g(rstn), .rst_l(ram_rd_valid    ), .en(lc_t    ), .max_in(ram_max_p   ), .last_clk(lc_p   ), .last(l_p   ), .first(), .count(i_p   ));
  up_counter #(.W(WIDTH)) C_T    (.clk(clk), .rstn_g(rstn), .rst_l(ram_rd_valid    ), .en(lc_n    ), .max_in(ram_max_t   ), .last_clk(lc_t   ), .last(l_t   ), .first(), .count(i_t   ));
  up_counter #(.W(WIDTH)) C_N    (.clk(clk), .rstn_g(rstn), .rst_l(ram_rd_valid    ), .en(lc_l    ), .max_in(ram_max_n   ), .last_clk(lc_n   ), .last(l_n   ), .first(), .count(i_n   ));
  up_counter #(.W(WIDTH)) C_L    (.clk(clk), .rstn_g(rstn), .rst_l(ram_rd_valid    ), .en(lc_wkw2 ), .max_in(ram_max_l   ), .last_clk(lc_l   ), .last(l_l   ), .first(), .count(i_l   ));
  up_counter #(.W(WIDTH)) C_WKW2 (.clk(clk), .rstn_g(rstn), .rst_l(ram_rd_valid    ), .en(en_count), .max_in(ram_max_wkw2), .last_clk(lc_wkw2), .last(l_wkw2), .first(), .count(i_wkw2));
  
  logic [WIDTH-1:0] max_ir                    ;     
  logic [WIDTH-1:0] max_iw_last               ;       
  logic [WIDTH-1:0] max_icoe                  ;     
  
  assign max_ir       = ram_PE_ROWS  - 1;   
  assign max_iw_last  = w_last_hw    - 1; 
  assign max_icoe     = ram_coe      - 1;

  logic lc_ir, lc_iw_last, lc_icoe;
  logic l_ir, l_iw_last, l_icoe;
  logic [WIDTH-1:0] i_ir, i_iw_last, i_icoe;

  logic hold_invalid;
  logic rst_inner_rest;
  wire rst_inner = (rst_inner_rest) || ram_rd_valid || en_counter_delay;
  wire addr_enable_counter = cfg[WB_A_ADDR_EN_COUNTER][0];
  wire wb_a_addr_invalid   = cfg[WB_A_ADDR_INVALID][0];
  wire wb_a_addr_done      = cfg[WB_A_ADDR_DONE][0];
  wire wb_a_done_write_any = cfg[WB_A_DONE_WRITE_ANY][0];

  wire en_counter_rest = pixel_invalid || addr_enable_counter || en_counter_acc || conv_stride_invalid;
  wire pixel_invalid = ((iyh >= ram_h) | (iyc >= ram_co)) && (!hold_invalid);
  wire en_counter_last = (!rst_inner_rest) && (en_counter_rest) && (!addr_en_counter_hold);


  up_counter #(.W(WIDTH)) C_IR      (.clk(clk), .rstn_g(rstn), .rst_l(rst_inner), .en(en_counter_last    ),      .max_in(max_ir       ),          .last_clk(lc_ir     ),      .last(l_ir     ),      .first(), .count(i_ir     ));
  up_counter #(.W(WIDTH)) C_IW_LAST (.clk(clk), .rstn_g(rstn), .rst_l(rst_inner), .en(lc_ir              ),      .max_in(max_iw_last  ),          .last_clk(lc_iw_last),      .last(l_iw_last),      .first(), .count(i_iw_last));
  up_counter #(.W(WIDTH)) C_ICOE    (.clk(clk), .rstn_g(rstn), .rst_l(rst_inner), .en(lc_iw_last         ),      .max_in(max_icoe     ),          .last_clk(lc_icoe   ),      .last(l_icoe   ),      .first(), .count(i_icoe   ));

  //------------------- SRAM ADDRESS COUNTER ----------------------------------

  logic [WIDTH-1:0] i_sram_addr;
  up_counter #(.W(WIDTH)) C_SRAM_ADDR (.clk(clk), .rstn_g(rstn), .rst_l(rst_inner), .en(en_counter_last), .max_in(32'hFFFF_FFFF), .last_clk(), .last(), .first(), .count(i_sram_addr));
 
  localparam I_DELAY = 3;
  logic i_valid_next;
  n_delay #(.W(1), .N(I_DELAY)) I_VALID_DELAY (.c(clk), .rng(rstn), .rnl(rstn), .e(1'b1), .i(en_count), .o(i_valid_next));


  logic en_counter_delay;
  n_delay #(.W(1), .N(1)) P_PASS_DELAY (.c(clk), .rng(rstn), .rnl(rstn), .e(1'b1), .i(en_count), .o(en_counter_delay));

  logic en_counter_hold_invalid;
  n_delay #(.W(1), .N(HOLD_INV_DELAY)) HOLD_INVALID_DELAY (.c(clk), .rng(rstn), .rnl(rstn), .e(1'b1), .i(en_count), .o(en_counter_hold_invalid));

  logic all_done_now; //all done signal for firmware
  assign all_done_now = l_b && l_p && l_t && l_n && l_l && l_wkw2;

  // The i_valid_next pulse (I_DELAY clocks after en_count) can come before the next bundle's header is in ram_* (ram_rd_en ->
  // valid_increment -> ram_rd_valid), so a last bundle whose limits differ from the previous bundle's was judged with the old limits.
  // The counters restart on ram_rd_valid; the clock after it, the flags are judged with the new limits.
  logic ram_rd_valid_d;
  always_ff @(posedge clk) ram_rd_valid_d <= rstn && ram_rd_valid;

  logic clear_invalid;

  wire [WIDTH-1:0] i_bias; 
  assign i_bias = coe_t_q + ram_b_offset + i_icoe;

  logic addr_en_counter_delay;
  n_delay #(.W(1), .N(EN_CNT_DELAY)) EN_COUNTER_DELAY (.c(clk), .rng(rstn), .rnl(rstn), .e(1'b1), .i(en_counter_last), .o(addr_en_counter_delay));
  logic addr_en_counter_hold;


  always_ff @(posedge clk) // All cfg written in this always block
      if (!rstn) begin 
        cfg                  <= '{default:WIDTH'(0)};
        hold_invalid         <= 1'b0;
        addr_en_counter_hold <= 1'b0;
        rst_inner_rest       <= 1'b0;
        clear_invalid        <= 1'b0;
      end else begin

            

      if (en_count) 
        cfg[WB_A_EN_COUNT] <= WIDTH'(0);
    

      if (en_counter_hold_invalid) 
        hold_invalid <= 0;
  
      if (en_counter_last) 
        addr_en_counter_hold <= 1;
      
      if (addr_en_counter_delay) 
        addr_en_counter_hold <= 0;
    

      if (en_counter_last || start_increment_2) 
        cfg[WB_A_ADDR_INVALID] <= WIDTH'(1);

      if (addr_enable_counter) 
        cfg[WB_A_ADDR_EN_COUNTER] <= WIDTH'(0);

      if (sm_auto_ack || pool_done)                      // PL accepts the logit pixel itself / has finished the pooling of the pixel
        cfg[WB_A_ADDR_EN_COUNTER] <= WIDTH'(1);
      

      if ((wb_a_addr_invalid) && (!en_counter_last) && (!hold_invalid) && (sum_proceed) && (!conv_stride_invalid) && (!addr_en_counter_hold))
        clear_invalid <= 1; 

      if (clear_invalid) begin
        clear_invalid <= 0; 
        cfg[WB_A_ADDR_INVALID   ] <= WIDTH'(0);
      end



      if (start) begin
        cfg[WB_A_ALL_DONE] <= WIDTH'(0);
        cfg[WB_A_START] <= WIDTH'(0);
      end

      if (i_valid_next) begin
        if (all_done_now)
          cfg[WB_A_ALL_DONE] <= WIDTH'(1);
      end
      if (ram_rd_valid_d && all_done_now)
        cfg[WB_A_ALL_DONE] <= WIDTH'(1);

      if((bank_end) && (en_counter_last)) begin
        cfg[WB_A_ADDR_DONE] <= 1;
        rst_inner_rest <= 1;
        hold_invalid <= 1;
      end
      if(wb_a_addr_done) 
        rst_inner_rest <= 0;


      cfg[WB_A_DONE_WRITE_ANY] <= WIDTH'(cfg[WB_A_DONE_WRITE_0][0] || cfg[WB_A_DONE_WRITE_1][0]) ;

      cfg[WB_A_BUNDLE_STATUS] <= WIDTH'(WIDTH'(cfg[WB_A_DONE_WRITE_0][0] || cfg[WB_A_DONE_WRITE_1][0]) | (WIDTH'(cfg[WB_A_ALL_DONE][0]) << 1));
  

      cfg[WB_A_IY_NHWC_CONV] <= iy_nhwc_conv;
      if (sm_em_valid)                                   // softmax result of one class: valid pixel for the PS
        cfg[WB_A_OUT_FLAGS] <= WIDTH'(sm_em_data & 32'h3FFF_FFFF);
      else if (pool_em_valid)                            // pooled pixel: valid pixel for the PS
        cfg[WB_A_OUT_FLAGS] <= WIDTH'(pool_em_data & 32'h3FFF_FFFF);
      else if (ram_is_softmax)                           // logits are hidden; done only once the softmax unit is idle
        cfg[WB_A_OUT_FLAGS] <= WIDTH'((WIDTH'(cfg[WB_A_ADDR_DONE][0] && sm_idle) << 30) | (WIDTH'(1) << 31));
      else if (ram_is_pool)                              // conv pixels are hidden; the PS only sees pooled pixels
        cfg[WB_A_OUT_FLAGS] <= WIDTH'((WIDTH'(cfg[WB_A_ADDR_DONE][0] && pool_idle) << 30) | (WIDTH'(1) << 31));
      else
        cfg[WB_A_OUT_FLAGS] <= WIDTH'((outval_p & 32'h3FFF_FFFF) 
                     | (WIDTH'(cfg[WB_A_ADDR_DONE][0])    << 30) 
                     | (WIDTH'(cfg[WB_A_ADDR_INVALID][0]) << 31));
      cfg[WB_A_PIXEL_PACK  ] <= sm_em_valid   ? sm_em_pack :
                                pool_em_valid ? (WIDTH'(pool_em_n) | (WIDTH'(pool_em_h) << 5)) : pack_p;
      cfg[WB_A_PIXEL_PACK2 ] <= sm_em_valid   ? (WIDTH'(sm_em_w) | (WIDTH'(sm_em_k[21:0]) << 10)) :
                                pool_em_valid ? (WIDTH'(pool_em_w) | (WIDTH'(pool_em_c) << 10)) : pack2_p;

      

      if (rstn && beat_handshake && out_tlast) begin
          if (raw_ocm_bank == 1'b0) cfg[WB_A_DONE_WRITE_0] <= WIDTH'(1);
          else                      cfg[WB_A_DONE_WRITE_1] <= WIDTH'(1);
      end
 

    
      if (reg_wr_en && reg_wr_addr < N_REG && !(em_any_valid && reg_wr_addr == WB_A_ADDR_EN_COUNTER)) // PS has priority in writing to registers (its ack of a softmax result must not move the counters)
        cfg[reg_wr_addr] <= WIDTH'(reg_wr_data);

    end

 

  // ============================================================================
  // synthesis translate_off
  // ---- Timing self-check (simulation only) ----
  // Pixel timing, in clocks after the cycle in which en_counter_last is high (the counters move at its end):
  //    sum_val is valid at +LAT_SUM, outval_p at +(LAT_SUM + LAT_PIPE), ADDR_INVALID falls at +(EN_CNT_DELAY + LAT_TAIL).
  // These are equal to the measured worst cases. The property that must hold: once a pixel is taken (the flag fell while its raw
  // bank was complete, or the softmax unit accepted it) none of the values the PS / softmax unit reads may change until the
  // counters move again. If a pipeline stage is added or removed without updating the LAT_* constants, this stops the simulation.
  wire [4*WIDTH-1:0]  chk_val = {outval_p, pack_p, pack2_p, iy_nhwc_conv};
  logic [4*WIDTH-1:0] chk_val_d;
  logic               chk_inv_d, chk_open;
  always @(posedge clk) begin
    chk_val_d <= chk_val;
    chk_inv_d <= cfg[WB_A_ADDR_INVALID][0];
    if (!rstn) chk_open <= 1'b0;
    else begin
      if (chk_open && (chk_val != chk_val_d))
        $fatal(1, "[TIMING] value changed after the pixel was taken: changed fields (outval_p,pack,pack2,iy_nhwc_conv) = %b%b%b%b  out %0d->%0d  (EN_CNT_DELAY=%0d, PX_SETTLE=%0d)",
               chk_val[3*WIDTH +: WIDTH] != chk_val_d[3*WIDTH +: WIDTH], chk_val[2*WIDTH +: WIDTH] != chk_val_d[2*WIDTH +: WIDTH],
               chk_val[WIDTH +: WIDTH] != chk_val_d[WIDTH +: WIDTH], chk_val[0 +: WIDTH] != chk_val_d[0 +: WIDTH],
               $signed(chk_val_d[3*WIDTH +: WIDTH]), $signed(chk_val[3*WIDTH +: WIDTH]), EN_CNT_DELAY, PX_SETTLE);
      if (rst_inner || en_counter_last) chk_open <= 1'b0;
      if (!ram_is_softmax && !ram_is_pool && chk_inv_d && !cfg[WB_A_ADDR_INVALID][0] && px_bank_written && !cfg[WB_A_ADDR_DONE][0] && !rst_inner) chk_open <= 1'b1;
      if (sm_auto_ack || pool_start) chk_open <= 1'b1;
    end
  end
  // synthesis translate_on

endmodule
