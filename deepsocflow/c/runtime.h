#ifndef RISCV
  #include <assert.h>
  #include <stdlib.h>
  #include <stdio.h>
  #include <math.h>
#endif
#include <limits.h>
#include <stdint.h>

#define DEBUG_INFO "--- ib:%d \n",ib


typedef struct { int quot; int rem; } idiv_t;
static inline idiv_t idiv(int numer, int denom) {
  idiv_t r; 
  r.quot = numer / denom; 
  r.rem = numer % denom; 
  return r;
}

typedef const struct {
  const uint16_t  n, l, kw, coe, h, w, ci, co, w_kw2, t, p, cm, cm_p0, on, oh, ow, oc, ch, ph, cw, pw, pkh, psh, pkw, psw;
  const int32_t   xp_words, b_offset, w_bpt, w_bpt_p0, x_bpt, x_bpt_p0, o_words, o_bytes;
  const int8_t    ib_out, in_buffer_idx, out_buffer_idx, add_out_buffer_idx, add_in_buffer_idx;
  const int8_t    is_bias, is_pool, is_flatten, is_softmax;
  const int8_t    x_pad, b_val_shift, b_bias_shift, ca_nzero, ca_shift, ca_pl_scale, aa_nzero, aa_shift, aa_pl_scale, pa_nzero, pa_shift, pa_pl_scale, softmax_frac;
  const int8_t    csh, csh_shift, psh_shift, csw, csw_shift, psw_shift, pool;
  const int32_t   softmax_max_i;
  const uint64_t  header;
  const int32_t   debug_nhwc_words;
} Bundle_t;

typedef enum {POOL_NONE, POOL_MAX, POOL_AVG} Pool_t;

#include "config_fw.h"

#define f32__ O_TYPE
#define X_BITS            (1 << X_BITS_L2)
#define X_WORDS_PER_BYTE  (8 / X_BITS)
#define X_BITS_MASK       ((1 << X_BITS) -1)
#ifdef SIM
  #define XDEBUG
  void usleep(int x) {}
#endif

typedef struct {
  // These can be kept in DDR
  int8_t  w              [W_BYTES     ];
  B_TYPE  b              [B_WORDS     ]; // keep next to w. weights are loaded to w_ptr
  int8_t  x              [X_BYTES     ]; // keep next to wb. wbx is loaded to w_ptr
  O_TYPE  y              [O_WORDS     ];
  
  // These are written often, keep them on OCM
  Y_TYPE ocm            [2][PE_COLS*PE_ROWS];
  int32_t nhwc           [NHWC_WORDS  ];
  int8_t  out_buffers    [N_OUT_BUF   ][O_BYTES_MAX ];
  
#ifdef XDEBUG
  int8_t  debug_tiled    [O_WORDS_MAX ];
  int32_t debug_nhwc     [NHWC_WORDS  ];
#endif
  int8_t  add_buffers    [N_ADD_BUF   ][NHWC_WORDS  ]; // should be last, since N_ADD_BUF can be empty
} Memory_st;

#define MEM_BASEADDR 0x20000000
#include "fb_fw_wrap.h"

#define WORDS_IN_CFG  1024/4

#define A_START        0x0
#define A_DONE_READ    0x1 // 2
#define A_DONE_WRITE   0x3 // 2
#define A_OCM_BASE     0x5 // 2
#define A_WEIGHTS_BASE 0x7
#define A_BUNDLE_DONE  0x8
#define A_N_BUNDLES_1  0x9
#define A_W_DONE       0xA // W,X,O done are written by PL, read by PS to debug which one hangs
#define A_X_DONE       0xB 
#define A_O_DONE       0xC

enum wb_reg {
#include "config_reg.def"
  WB_A_NUM_REGS
};

#ifdef __cplusplus
  #define EXT_C "C"
  #define restrict __restrict__ 
#else
  #define EXT_C
#endif
int32_t *p_config = (int32_t *)CONFIG_BASEADDR;
int32_t *p_writeback = (int32_t *)CONFIG_BASEADDR + WORDS_IN_CFG;

extern EXT_C void model_setup(Memory_st *restrict mp) {

#ifdef SIM
  FILE *fp;
  char f_path [1000];
  sprintf(f_path, "%s/wbx.bin", DATA_DIR);
  fp = fopen(f_path, "rb");
  debug_printf("DEBUG: Reading from file %s \n", f_path);
  if(!fp) debug_printf("ERROR! File not found: %s \n", f_path);
  int bytes = fread(mp->w, 1, WB_BYTES+X_BYTES, fp);
  fclose(fp);
#endif
  flush_cache(mp->w, WB_BYTES+X_BYTES);  // force transfer to DDR, starting addr & length

  // Write registers in controller
  fb_write_reg32(p_config + A_START       , 0);  // Start
  fb_write_reg32(p_config + A_DONE_READ +0, 1);  // Done read mp->ocm bank 0
  fb_write_reg32(p_config + A_DONE_READ +1, 1);  // Done read mp->ocm bank 1
  fb_write_reg32(p_config + A_DONE_WRITE+0, 0);  // Done write mp->ocm bank 0
  fb_write_reg32(p_config + A_DONE_WRITE+1, 0);  // Done write mp->ocm bank 1
  fb_write_reg32(p_config + A_OCM_BASE  +0, fb_addr_64to32(mem_phy.ocm[0]));  // Base addr mp->ocm bank 0
  fb_write_reg32(p_config + A_OCM_BASE  +1, fb_addr_64to32(mem_phy.ocm[1]));  // Base addr mp->ocm bank 1
  fb_write_reg32(p_config + A_WEIGHTS_BASE, fb_addr_64to32(mem_phy.w));  // Base adddr weights
  fb_write_reg32(p_config + A_BUNDLE_DONE , 1);  // Bundle done writing (pixel dma waits for this)
  fb_write_reg32(p_config + A_N_BUNDLES_1 , N_BUNDLES);  // Number of bundles
  fb_write_reg32(p_config + A_W_DONE      , 0);  // Weigths done
  fb_write_reg32(p_config + A_X_DONE      , 0);  // Bundle done
  fb_write_reg32(p_config + A_O_DONE      , 0);  // Output done

  // Write into BRAM the config for controller
  i32 parameters[8*N_BUNDLES];
  for (int var = 0; var < N_BUNDLES; var++){
    parameters[8*var] = (var == 0) ? fb_addr_64to32(mem_phy.x) : fb_addr_64to32(mem_phy.out_buffers[bundles[var].in_buffer_idx]);       // x_base address
    parameters[8*var+1] = bundles[var].x_bpt_p0;  // x_bpt0
    parameters[8*var+2] = bundles[var].x_bpt;     // x_bpt
    parameters[8*var+3] = bundles[var].w_bpt_p0;  // w_bpt0
    parameters[8*var+4] = bundles[var].w_bpt;     // w_bpt

    assert_printf(bundles[var].p, <, 1<<16, "", "P should be less than 2**16 for bundle:%x", var);
    assert_printf(bundles[var].t, <, 1<<16, "", "T should be less than 2**16 for bundle:%x", var);
    parameters[8*var+5] = (bundles[var].t << 16) + bundles[var].p; // max p
    uint64_t h = bundles[var].header;
    parameters[8*var + 6] = (uint32_t)(h & 0xFFFFFFFFu);
    parameters[8*var + 7] = (uint32_t)(h >> 32);
  }
  for (int var = 0; var < 8*N_BUNDLES; var++){
    fb_write_reg32(p_config + 16+var, parameters[var]);
  }

  fb_write_reg32(p_writeback + WB_A_N_BUNDLES_1, N_BUNDLES-1);
  int val = fb_read_reg32(p_writeback + WB_A_N_BUNDLES_1);

  int32_t wb_bundles [N_BUNDLES][32] = {{0}};
  
  for (int ib = 0; ib < N_BUNDLES; ib++){
    wb_bundles[ib][0]= bundles[ib].p-1;
    wb_bundles[ib][1]= bundles[ib].t-1;
    wb_bundles[ib][2]= bundles[ib].n-1;
    wb_bundles[ib][3]= bundles[ib].l-1;
    wb_bundles[ib][4]= bundles[ib].w_kw2-1;
    wb_bundles[ib][5]= bundles[ib].coe;
    wb_bundles[ib][6]= PE_ROWS;
    wb_bundles[ib][7]= bundles[ib].kw;
    wb_bundles[ib][8]= bundles[ib].b_offset;
    wb_bundles[ib][9]= bundles[ib].h;  
    wb_bundles[ib][10]= bundles[ib].w; 
    wb_bundles[ib][11]= bundles[ib].co;
    wb_bundles[ib][12]= bundles[ib].csh;
    wb_bundles[ib][13]= bundles[ib].csh_shift;
    wb_bundles[ib][14]= bundles[ib].csw;
    wb_bundles[ib][15]= bundles[ib].csw_shift;
    wb_bundles[ib][16]= bundles[ib].ch;
    wb_bundles[ib][17]= bundles[ib].cw;
    wb_bundles[ib][18] = PE_ROWS / bundles[ib].csh;
    wb_bundles[ib][19] = PE_ROWS % bundles[ib].csh;
    wb_bundles[ib][20] = bundles[ib].is_bias;  // PL adds bias
    wb_bundles[ib][21] = bundles[ib].b_val_shift;
    wb_bundles[ib][22] = bundles[ib].b_bias_shift;
    wb_bundles[ib][23] = bundles[ib].ca_nzero;      // PL does core activation (quant_lrelu)
    wb_bundles[ib][24] = bundles[ib].ca_shift;
    wb_bundles[ib][25] = bundles[ib].ca_pl_scale;
    wb_bundles[ib][26] = -(1 << (X_BITS - bundles[ib].ca_pl_scale - 1));  // clip min
    wb_bundles[ib][27] = (1 << (X_BITS - 1)) - 1;                         // clip max
  }
  
  int32_t * wb_bundles_p = (p_writeback + 32);
  
  for (int i=0; i < sizeof(wb_bundles)/sizeof(i32); i++){
    fb_write_reg32(wb_bundles_p + i, ((i32*)wb_bundles)[i]);
  }
  
  // Bias values into the PL bias RAM (index = i_bias = b_offset + channel)
  for (int i = 0; i < B_WORDS; i++)
    fb_write_reg32(p_writeback + WB_BIAS_BASE + i, (i32)mp->b[i]);

  fb_write_reg32(p_writeback + WB_A_START, 1);
}

// Helper functions

static inline void write_flush_u8(u8*restrict addr, u8 val) {
  *addr = val; // Leave flushing to the end of bundle
}

#define flatten_nhwc(in,ih,iw,ic, N,H,W,C, optional_debug_info,...)\
  ((in*H + ih)*W + iw)*C + ic;\
  assert_printf (in, <, N, optional_debug_info,__VA_ARGS__); assert_printf (ih, <, H, optional_debug_info,__VA_ARGS__); assert_printf (iw, <, W, optional_debug_info,__VA_ARGS__); assert_printf (ic, <, C, optional_debug_info,__VA_ARGS__); assert_printf ((((in*H + ih)*W + iw)*C + ic), <, NHWC_WORDS, optional_debug_info,__VA_ARGS__);

#define max(x, y) ((x) > (y) ? (x) : (y))
#define min(x, y) ((x) < (y) ? (x) : (y))
#define clip(x, xmin, xmax) (((x) < (xmin)) ? (xmin) : ((x) > (xmax)) ? (xmax) : (x))
#define shift_round(n, s) (((n) + ((s)>0 ? (1<<((s)-1)) - (~((n)>>(s))&1) : 0)) >> s) // === np.around(n/2**s).astype(i32)
#define div_round(a, b) (((a)+((b)/2) - (~((b)|(a)/(b)) &1))/(b))


static inline i32 quant_lrelu(i32 x, i8 nzero, i8 shift, i8 pl_scale){
  x = x < 0 ? (nzero ? x: 0) : x << pl_scale; // Conditional, targeting ARM
  x = shift_round(x, shift);
  x = clip(x, -(1<<(X_BITS-pl_scale-1)), (1<<(X_BITS-1))-1);
  return x;
}


static inline void write_x(i8 val, i8 *restrict p_out_buffer, Memory_st *restrict mp, i32 ib, i32 ixp, i32 ixn, i32 ixl, i32 ixw, i32 ixcm, i32 ixr, Bundle_t *restrict pb_out, i32 xcm) {

  #define WRITEX_DEBUG_INFO "--- ib:%d ixp:%d ixn:%d ixl:%d ixw:%d ixcm:%d ixr:%d xcm :%d \n",ib,ixp,ixn,ixl,ixw,ixcm,ixr,xcm
  assert_printf (ixr , <, PE_ROWS+pb_out->x_pad, "write_x", WRITEX_DEBUG_INFO);
  assert_printf (ixcm, <, xcm          , "write_x", WRITEX_DEBUG_INFO);
  assert_printf (ixw , <, pb_out->w    , "write_x", WRITEX_DEBUG_INFO);
  assert_printf (ixl , <, pb_out->l    , "write_x", WRITEX_DEBUG_INFO);
  assert_printf (ixn , <, pb_out->n    , "write_x", WRITEX_DEBUG_INFO);
  assert_printf (ixp , <, pb_out->p    , "write_x", WRITEX_DEBUG_INFO);

  i32 p_offset       = (ixp == 0) ? 0 : (pb_out->cm_p0 + (ixp-1)*pb_out->cm) * pb_out->xp_words;
  i32 flat_index_n2r = (((ixn*pb_out->l + ixl)*pb_out->w + ixw)*xcm + ixcm)*(PE_ROWS+pb_out->x_pad) + ixr; // multidim_index -> flat_index [n,l,w,cm,r]
  i32 flat_index     = p_offset + flat_index_n2r;

#ifdef XDEBUG
  mp->debug_tiled[flat_index] = val;
#endif

  // Pack bits and store
  idiv_t packed_idx = idiv(flat_index, X_WORDS_PER_BYTE);
  assert_printf (packed_idx.quot , <, bundles[ib].o_bytes, "write_x", WRITEX_DEBUG_INFO);

  u8 packed_val      = ((u8)val & X_BITS_MASK) << (packed_idx.rem * X_BITS);
  u8 mem_val         = p_out_buffer[packed_idx.quot];
  u8 mem_val_cleaned = X_POSITION_INVERTED_MASKS[packed_idx.rem] & mem_val;
  write_flush_u8((u8*)(p_out_buffer + packed_idx.quot), mem_val_cleaned | packed_val);
}


static inline void tile_write( i32 out_val, i8 *restrict p_out_buffer, i32 ib, Bundle_t *restrict pb, Memory_st *restrict mp, i32 i_yn, i32 i_yh, i32 i_yw, i32 i_yc, i32 yn, i32 yh, i32 yw, i32 yc ) {

  // ------ FLATTEN ------
  if (pb->is_flatten) {
    i_yc = (i_yh*yw + i_yw)*yc + i_yc;  // (H*W*C) -> C
    i_yw = 0;                           // W=1
    i_yh = i_yn;                        // N -> H
    i_yn = 0;                           // N=1

    yc = yh*yw*yc;
    yw = 1;
    yh = yn;
    yn = 1;
  }

  i32 iy_nhwc = flatten_nhwc(i_yn,i_yh,i_yw,i_yc, pb->on,pb->oh,pb->ow,pb->oc,,);
#ifdef XDEBUG
  mp->debug_nhwc[iy_nhwc] = out_val;
#endif
 // ------ STORE IN NHWC  ------

  if (ib == N_BUNDLES-1) {
    mp->y[iy_nhwc] = out_val; // Last bundle: save as NHWC
    return;
  }

  // Store for residual add
  if (pb->add_out_buffer_idx != -1)
    mp->add_buffers[pb->add_out_buffer_idx][iy_nhwc] = (i8)out_val;

  // If output only goes to residual add, early return
  Bundle_t*restrict pb_out;
  if (pb->ib_out == -1)
    return;
  else
    pb_out = &bundles[pb->ib_out];
    

  // ------ TILING: Calculate X coordinates ------
  // y [n,h,w,c] -> x[p, n, l, w,cmp, r+pad]

  i8 yp_first  = i_yc < pb_out->cm_p0;

  idiv_t div_oh  = idiv(i_yh, PE_ROWS);
  i32   i_yr    = div_oh.rem;
  i32   i_yl    = div_oh.quot;

  idiv_t div_oc    = idiv(i_yc-pb_out->cm_p0, pb_out->cm);
  i32   i_yp      = yp_first ? 0             : div_oc.quot + 1;
  i32   i_ycm     = yp_first ? i_yc          : div_oc.rem;
  i32   ycm       = yp_first ? pb_out->cm_p0 : pb_out->cm  ;

  // ------ STORE FOR NEXT BUNDLE  ------
  // Other bundles: pad & save as tiled
  i32 yr_sweep = i_yh==yh-1 ? PE_ROWS : i_yr + 1;

  for (i32 i_yr_dest = i_yr; i_yr_dest < yr_sweep; i_yr_dest++) {
    write_x(out_val, p_out_buffer, mp, ib, i_yp, i_yn, i_yl, i_yw, i_ycm, i_yr_dest,   pb_out, ycm);

    // --- PADDING: the [bottom x_pad rows of previous block (l-1)] with [first x_pad rows of this block (l)]
    if (i_yr_dest < pb_out->x_pad) {
      i32 pad_val = (i_yl == 0) ? 0         : out_val;
      i32 dest_yl = (i_yl == 0) ? pb_out->l-1 : i_yl-1;
      write_x(pad_val, p_out_buffer, mp, ib, i_yp, i_yn, dest_yl, i_yw, i_ycm, i_yr_dest+PE_ROWS,   pb_out, ycm);
    }
    out_val = 0;
  }
  
}






extern EXT_C void run(Memory_st *restrict mp) {


  static Bundle_t *restrict pb = &bundles[0];
  static i32 ib=0;
  static i8 *restrict p_out_buffer = 0;

  i32   iy_nhwc;
  idiv_t div_ixh, div_ixw;
  i32   ph_end, ph_beg_const, ixh_beg, xh_sweep;
  i32   pw_end, pw_beg_const, ixw_beg, xw_sweep;
  i32   bundle_counter;
  i32   bundle_limit;

  static i8 ocm_bank = 0; 

  model_setup(mp);

  fb_write_reg32(p_config + A_START, 1); 
  bundle_counter = 0;
  i32 done;
  i32 out_val=0;
  i32 yh;
  i32 yw;
  i32 i_yn;
  i32 i_yh;
  i32 i_yw;
  i32 i_yc;
  i32 i_bias;
  i32 yn;
  i32 yc;
  i32 status;
  i32 ov;
  i32 v;
  i32 pk; 
  i32 pk2;


  pb = &bundles[ib];
  bundle_limit = pb->p * pb->t * pb->n * pb->l * pb->w_kw2;
  p_out_buffer = (i8*)&(mp->out_buffers[pb->out_buffer_idx]);
  yn = pb->n;
  yc = pb->co;
  yh   = pb->ch;
  yw   = pb->cw;


  while(1) {

    while (!((status = fb_read_reg32(p_writeback + WB_A_BUNDLE_STATUS)) & 0x1)) {
    }
    done = (status >> 1) & 0x1;
    bundle_counter++;
    fb_write_reg32(p_config + A_DONE_WRITE + ocm_bank, 0);


    while (1) {

PROCESS_AND_STORE_DONE:

      while ((ov = fb_read_reg32(p_writeback + WB_A_OUT_FLAGS)) & 0x80000000) { 
        if (ov & 0x40000000) goto BUNDLE_DONE;                      
      }
      v       = ov & 0x3FFFFFFF;
      out_val = (v ^ 0x20000000) - 0x20000000;

      pk      = fb_read_reg32(p_writeback + WB_A_PIXEL_PACK);
      i_yn    =  pk         & 0x1F;       
      i_yh    = (pk >> 5)   & 0x3FF;    
      i_bias  = (pk >> 15)  & 0x1FFFF;

      iy_nhwc = fb_read_reg32(p_writeback + WB_A_IY_NHWC_CONV);

      pk2 = fb_read_reg32(p_writeback + WB_A_PIXEL_PACK2);
      i_yw = pk2 & 0x3FF;       
      i_yc = (pk2 >> 10) & 0x3FFFFF;

      fb_write_reg32(p_writeback + WB_A_ADDR_EN_COUNTER, 1);
  
     

      // ------ RESIDUAL ADD ---

      if (pb->add_in_buffer_idx != -1) {
        out_val += mp->add_buffers[pb->add_in_buffer_idx][iy_nhwc];
        out_val = quant_lrelu(out_val, pb->aa_nzero, pb->aa_shift, pb->aa_pl_scale);
      }

      // ------ SOFTMAX ------

      if (pb->is_softmax) {
        assert_printf (ib , !=, N_BUNDLES, "Softmax is only allowed for the last bundle.", DEBUG_INFO);

        f32__ val = (f32__)out_val;
        val = val / (f32__)(1 << pb->softmax_frac);
        val = val - ((f32__)pb->softmax_max_i)/(1 << 17);
        val = (f32__)exp(val);
        mp->y[iy_nhwc] = val;

        if (i_yc == pb->co-1) {
          f32__ sum = 0;
          i32 iy_nhwc;
          for (int i=0; i<pb->co; i++){
            iy_nhwc = flatten_nhwc(i_yn,i_yh,i_yw,i, yn,yh,yw,yc, "Before softmax sum", DEBUG_INFO);
            sum += mp->y[iy_nhwc];
          }
          for (int i=0; i<pb->co; i++){
            iy_nhwc = flatten_nhwc(i_yn,i_yh,i_yw,i, yn,yh,yw,yc, "After softmax sum", DEBUG_INFO);
            mp->y[iy_nhwc] = mp->y[iy_nhwc] / sum;
          }
        }
        goto PROCESS_AND_STORE_DONE;
      }

      // ------ MAX/AVG POOL ---

      if (pb->pool == POOL_NONE) {
        tile_write(out_val, p_out_buffer, ib, pb, mp, i_yn, i_yh, i_yw, i_yc, yn, yh, yw, yc);
        goto PROCESS_AND_STORE_DONE;
      }

      mp->nhwc[iy_nhwc] = out_val;

      div_ixh = idiv(i_yh+pb->psh_shift-pb->pkh+1, pb->psh);
      div_ixw = idiv(i_yw+pb->psw_shift-pb->pkw+1, pb->psw);
      ixh_beg = div_ixh.quot; // ix(hw) that corresponds to the pooling window
      ixw_beg = div_ixw.quot;

      if (ixh_beg < 0 || ixw_beg < 0) // skip when target ix(h,w) < 0
        goto PROCESS_AND_STORE_DONE;

      // Pool Striding
      if (div_ixh.rem != 0) {                       // invalid ixh
        if (i_yh==yh-1) ixh_beg += 1;                  //but last yh. start sweeping
        else            goto PROCESS_AND_STORE_DONE;   // not last yh. skip
      }

      if (div_ixw.rem != 0) {
        if (i_yw==yw-1) ixw_beg += 1;
        else            goto PROCESS_AND_STORE_DONE;
      }

      ph_end       = i_yh; // iy(h,w) is the bottom-right of pooling window -> All values in pooling window have been computed
      pw_end       = i_yw;
      ph_beg_const = max(pb->psh*ixh_beg-pb->psh_shift, 0)-1; // p(h,w)_beg is the index of top left corner of pooling window. If negative, set to zero
      pw_beg_const = max(pb->psw*ixw_beg-pb->psw_shift, 0)-1;

      xh_sweep = i_yh == yh-1 ? pb->ph : ixh_beg+1; // ix(hw) is sweeped from ix(hw)_beg to x(h,w)_sweep. Normally sweep is 1.
      xw_sweep = i_yw == yw-1 ? pb->pw : ixw_beg+1; // But when iy(h,w) is at its edges, need to compute remaining ix(hw) pixels by sweeping
      

      // Sweep the pooling window
      for (i32 ixh = ixh_beg, ph_beg = ph_beg_const;  ixh < xh_sweep;  ixh++, ph_beg += pb->psh) {
        for (i32 ixw = ixw_beg, pw_beg = pw_beg_const;  ixw < xw_sweep;  ixw++, pw_beg += pb->psw) {

          // Traverse each pool window & perform pooling
          i32 result = pb->pool == POOL_MAX ? INT_MIN : 0;
          for (i32 ipyh = ph_end; ipyh > ph_beg; ipyh--){
            for (i32 ipyw = pw_end; ipyw > pw_beg; ipyw--){

            i32 read_idx = flatten_nhwc(i_yn, ipyh, ipyw, i_yc,    yn, yh, yw, yc, "Inside pool window", DEBUG_INFO);
            i32 read_val = mp->nhwc[read_idx];
            result = pb->pool==POOL_MAX ? max(result, read_val) : (result + read_val);
            }
         }

        // ------ AVG POOL: Divide & Activation ------
        if (pb->pool == POOL_AVG) {
          i32 count  = (ph_end-ph_beg)*(pw_end-pw_beg);
          result = div_round(result, count);
          out_val = quant_lrelu(out_val, pb->pa_nzero, pb->pa_shift, pb->pa_pl_scale);
          }

        tile_write(result, p_out_buffer, ib, pb, mp,   i_yn, ixh, ixw, i_yc,  yn, pb->ph, pb->pw, yc); // Write
        }

      }
  }
  BUNDLE_DONE:
  fb_write_reg32(p_writeback + WB_A_ADDR_DONE, 0);

  if (ocm_bank){
    fb_write_reg32(p_writeback + WB_A_DONE_WRITE_1, 0);
  }
  else{
    fb_write_reg32(p_writeback + WB_A_DONE_WRITE_0, 0);
  }     
  fb_write_reg32(p_config + A_DONE_READ + ocm_bank, 1);       
  fb_write_reg32(p_writeback + WB_A_EN_COUNT, 1);
              

#ifdef SIM
    char f_path_debug [1000];
    sprintf(f_path_debug, "%s/%0d_y_nhwc_sim.txt", DATA_DIR, ib);
    FILE *fp_debug = fopen(f_path_debug, "w");
    for (i32 i=0; i<pb->debug_nhwc_words; i++)
      sim_fprintf(fp_debug,"%d\n", mp->debug_nhwc[i]);
    fclose(fp_debug);

    char f_path_tiled [1000];
    sprintf(f_path_tiled, "%s/%0d_y_tiled_sim.txt", DATA_DIR, ib);
    FILE *fp_tiled = fopen(f_path_tiled, "w");
    for (i32 i=0; i<pb->o_words; i++)
      if (ib == N_BUNDLES-1) sim_fprintf(fp_tiled,"%d\n", (i32)(mp->y[i] * (1 << 17)));
      else sim_fprintf(fp_tiled,"%d\n", mp->debug_tiled[i]);
    fclose(fp_tiled);

    if (ib != N_BUNDLES-1){
      char f_path_packed [1000];
      sprintf(f_path_packed, "%s/%0d_y_packed_sim.bin", DATA_DIR, ib);
      FILE *fp_packed = fopen(f_path_packed, "wb");
      fwrite(p_out_buffer, 1, pb->o_bytes, fp_packed);
      fclose(fp_packed);
    }
#endif

  if (done) {
    ocm_bank ^= 1;
    break;
  }

  if(bundle_counter == bundle_limit){
    flush_cache(p_out_buffer, pb->o_bytes);
    fb_write_reg32(p_config + A_BUNDLE_DONE, 1);
    bundle_counter = 0;
    ib++;
    pb = &bundles[ib];
    bundle_limit = pb->p * pb->t * pb->n * pb->l * pb->w_kw2;
    p_out_buffer = (i8*)&(mp->out_buffers[pb->out_buffer_idx]);
    yn = pb->n;
    yc = pb->co;
    yh   = pb->ch;
    yw   = pb->cw;
  };
  ocm_bank ^= 1;
  } // first while
}

extern EXT_C void print_output (Memory_st *restrict mp) {
  flush_cache(mp->y, sizeof(mp->y));
  for (int i=0; i<O_WORDS; i++){
    int val_1000 = (int)(1000 * mp->y[i]);
    printf("y[%d]: %d/1000 \n", i, val_1000);
  }
}