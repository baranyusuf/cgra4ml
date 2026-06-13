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
    NHWC_ADDR_W = $clog2(NHWC_WORDS)
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
    ram_wr_en   = reg_wr_en && (reg_wr_addr >= N_REG);
    ram_wr_addr = SRAM_WR_ADDR_WIDTH'(reg_wr_addr - N_REG);
    ram_wr_data = WIDTH'(reg_wr_data);
    // ram_rd_en   = reg_rd_en && (reg_rd_addr >= N_REG); Cannot read from CPU
    reg_rd_data = AXI_DATA_WIDTH'(cfg[reg_rd_addr]);
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

  localparam ACTUAL_READ_WIDTH = 20 * WIDTH;
  assign ram_rd_addr = SRAM_RD_ADDR_WIDTH'(i_b);
  wire last_wkw2_now = (i_wkw2 == ram_max_wkw2);  
  wire [WIDTH-1:0] w_last_hw = last_wkw2_now ? ((ram_kw >> 1) + 32'd1) : 32'd1;

  logic [SRAM_RD_DATA_WIDTH-1:0] ram_rd_data_q;
  always_ff @(posedge clk)
    if (!rstn) ram_rd_data_q <= '0;
    else       ram_rd_data_q <= ram_rd_data; 

  assign {ram_pe_mod, ram_pe_div,
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
  logic [WIDTH-1:0] pack_p, pack2_p, outval_p, iyc_p, iyn_p, ibias_p;

  always_ff @(posedge clk)
    if (!rstn) begin
      pack_p<='0; pack2_p<='0; outval_p<='0;
      iyc_p<='0; iyn_p<='0; ibias_p<='0;
    end else begin
      pack_p   <= WIDTH'(WIDTH'(iyn[4:0]) | (WIDTH'(ch_quot[9:0]) << 5) | (WIDTH'(i_bias) << 15));
      pack2_p  <= WIDTH'(WIDTH'(cw_quot[9:0]) | (WIDTH'(iyc[21:0]) << 10));
      outval_p <= sum_val;
      iyc_p    <= iyc;
      iyn_p    <= iyn;
      ibias_p  <= i_bias;
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
  n_delay #(.W(1), .N(7)) HOLD_INVALID_DELAY (.c(clk), .rng(rstn), .rnl(rstn), .e(1'b1), .i(en_count), .o(en_counter_hold_invalid));

  logic all_done_now; //all done signal for firmware
  assign all_done_now = l_b && l_p && l_t && l_n && l_l && l_wkw2;

  logic clear_invalid;

  wire [WIDTH-1:0] i_bias; 
  assign i_bias = coe_t_q + ram_b_offset + i_icoe;

  logic addr_en_counter_delay;
  n_delay #(.W(1), .N(3)) EN_COUNTER_DELAY (.c(clk), .rng(rstn), .rnl(rstn), .e(1'b1), .i(en_counter_last), .o(addr_en_counter_delay));
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
      cfg[WB_A_OUT_FLAGS] <= WIDTH'((outval_p & 32'h3FFF_FFFF) 
                     | (WIDTH'(cfg[WB_A_ADDR_DONE][0])    << 30) 
                     | (WIDTH'(cfg[WB_A_ADDR_INVALID][0]) << 31));
      cfg[WB_A_PIXEL_PACK  ] <= pack_p;
      cfg[WB_A_PIXEL_PACK2 ] <= pack2_p;

      

      if (rstn && beat_handshake && out_tlast) begin
          if (raw_ocm_bank == 1'b0) cfg[WB_A_DONE_WRITE_0] <= WIDTH'(1);
          else                      cfg[WB_A_DONE_WRITE_1] <= WIDTH'(1);
      end
 

    
      if (reg_wr_en && reg_wr_addr < N_REG) // PS has priority in writing to registers
        cfg[reg_wr_addr] <= WIDTH'(reg_wr_data);

    end

 
endmodule