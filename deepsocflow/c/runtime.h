#ifndef RISCV
  #include <assert.h>
  #include <stdlib.h>
  #include <stdio.h>
  #endif
#include <limits.h>
#include <stdint.h>

typedef const struct {
  const uint16_t  n, l, kw, coe, h, w, ci, co, w_kw2, t, p, cm, cm_p0, on, oh, ow, oc, ch, ph, cw, pw, pkh, psh, pkw, psw;
  const int32_t   xp_words, b_offset, w_bpt, w_bpt_p0, x_bpt, x_bpt_p0, o_words, o_bytes;
  const int8_t    ib_out, in_buffer_idx, out_buffer_idx, add_out_buffer_idx, add_in_buffer_idx;
  const int8_t    is_bias, is_flatten, is_softmax, sm_shift, sm_out_frac;
  const int8_t    x_pad, b_val_shift, b_bias_shift, ca_nzero, ca_shift, ca_pl_scale, aa_nzero, aa_shift, aa_pl_scale, add_val_shift, add_a_shift, pa_nzero, pa_shift, pa_pl_scale;
  const int8_t    csh, csh_shift, psh_shift, csw, csw_shift, psw_shift, pool;
  const uint64_t  header;
} Bundle_t;

typedef enum {POOL_NONE, POOL_MAX, POOL_AVG} Pool_t;

#include "config_fw.h"

#define f32__ O_TYPE
#define X_BITS            (1 << X_BITS_L2)
#ifdef SIM
  #define XDEBUG
  void usleep(int x) {}
#endif

// Everything here is in DDR. The PL reads w, b, x and the out buffers, and writes the out buffers and y (the firmware never touches the outputs).
typedef struct {
  int8_t  w              [W_BYTES     ];
  B_TYPE  b              [B_WORDS     ]; // keep next to w. weights are loaded to w_ptr
  int8_t  x              [X_BYTES     ]; // keep next to wb. wbx is loaded to w_ptr
  O_TYPE  y              [O_WORDS     ]; // written by the PL (final output)
  int8_t  out_buffers    [N_OUT_BUF   ][O_BYTES_MAX ] __attribute__((aligned(64))); // written (whole AXI beats) by the PL, read by the PL pixel DMA
} Memory_st;

#define MEM_BASEADDR 0x20000000
#include "fb_fw_wrap.h"

#define WORDS_IN_CFG  1024/4

#define WB_BUNDLE_WORDS 64  // words reserved per bundle in the writeback header SRAM

#define A_START        0x0
#define A_WEIGHTS_BASE 0x1
#define A_N_BUNDLES_1  0x2
#define A_W_DONE       0x3 // W,X done are written by PL, read by PS to debug which one hangs
#define A_X_DONE       0x4

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
  fb_write_reg32(p_config + A_WEIGHTS_BASE, fb_addr_64to32(mem_phy.w));  // Base adddr weights
  fb_write_reg32(p_config + A_N_BUNDLES_1 , N_BUNDLES);  // Number of bundles

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

  int32_t wb_bundles [N_BUNDLES][WB_BUNDLE_WORDS] = {{0}};
  
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
    // Pooling parameters go to the unused upper bits of words 20..25 (layout: see the comment above ram_pool_mode in writeback.sv); the low byte keeps the old field
    wb_bundles[ib][20] = (bundles[ib].is_bias & 0xFF) | ((bundles[ib].pool & 0x3) << 8);  // PL adds bias | pool mode
    wb_bundles[ib][21] = (bundles[ib].b_val_shift & 0xFF) | ((bundles[ib].pkh & 0xFF) << 8) | ((bundles[ib].pkw & 0xFF) << 16) | (((uint32_t)bundles[ib].psh_shift & 0xFF) << 24);
    wb_bundles[ib][22] = (bundles[ib].b_bias_shift & 0xFF) | ((bundles[ib].psh & 0xFF) << 8) | ((bundles[ib].psw & 0xFF) << 16) | (((uint32_t)bundles[ib].psw_shift & 0xFF) << 24);
    wb_bundles[ib][23] = (bundles[ib].ca_nzero != 0) | ((bundles[ib].pa_nzero != 0) << 8);      // PL does core activation (quant_lrelu) | pool activation nzero
    wb_bundles[ib][24] = (bundles[ib].ca_shift & 0xFF) | ((bundles[ib].pa_shift & 0xFF) << 8) | ((bundles[ib].ph & 0x3FF) << 16);
    wb_bundles[ib][25] = (bundles[ib].ca_pl_scale & 0xFF) | ((bundles[ib].pa_pl_scale & 0xFF) << 8) | ((bundles[ib].pw & 0x3FF) << 16);
    wb_bundles[ib][26] = -(1 << (X_BITS - bundles[ib].ca_pl_scale - 1));  // clip min
    wb_bundles[ib][27] = (1 << (X_BITS - 1)) - 1;                         // clip max
    wb_bundles[ib][28] = bundles[ib].add_in_buffer_idx == -1 ? -1 : bundles[ib].add_in_buffer_idx * NHWC_WORDS;  // PL residual add: buffer base (-1 = none)
    wb_bundles[ib][29] = (bundles[ib].aa_nzero != 0) | ((bundles[ib].aa_shift & 0xFF) << 8) | ((bundles[ib].aa_pl_scale & 0xFF) << 16);
    wb_bundles[ib][30] = -(1 << (X_BITS - bundles[ib].aa_pl_scale - 1));  // residual-add clip min
    // Words 32..40: where and how the PL writes this bundle's output (layout: see the comment above ram_out_base in writeback.sv)
    const Bundle_t *pn = bundles[ib].ib_out == -1 ? &bundles[ib] : &bundles[bundles[ib].ib_out];   // bundle that reads this output (tiling of its input)
    const int is_last = (ib == N_BUNDLES-1);
    wb_bundles[ib][32] = is_last ? fb_addr_64to32(mem_phy.y) : (bundles[ib].ib_out == -1 ? 0 : fb_addr_64to32(mem_phy.out_buffers[bundles[ib].out_buffer_idx]));
    wb_bundles[ib][33] = bundles[ib].o_bytes;
    wb_bundles[ib][34] = (bundles[ib].oh & 0xFFFF) | ((bundles[ib].ow & 0xFFFF) << 16);
    wb_bundles[ib][35] = bundles[ib].oc;
    wb_bundles[ib][36] = (pn->l & 0xFFFF) | ((pn->w & 0xFFFF) << 16);
    wb_bundles[ib][37] = (pn->cm & 0xFFFF) | ((pn->cm_p0 & 0xFFFF) << 16);
    wb_bundles[ib][38] = pn->xp_words;
    wb_bundles[ib][39] = (bundles[ib].ib_out != -1) | ((bundles[ib].is_flatten != 0) << 1) | (is_last << 2) | ((pn->x_pad & 0xFF) << 8);
    wb_bundles[ib][40] = bundles[ib].add_out_buffer_idx == -1 ? -1 : bundles[ib].add_out_buffer_idx * NHWC_WORDS;
    wb_bundles[ib][31] = (bundles[ib].add_val_shift & 0xFF) | ((bundles[ib].add_a_shift & 0xFF) << 8) | ((bundles[ib].is_softmax != 0) << 16) | ((ib != N_BUNDLES-1) << 17) | ((bundles[ib].sm_shift & 0xF) << 20) | ((bundles[ib].sm_out_frac & 0xF) << 24);  // residual-add operand alignment shifts | bit16: PL softmax, bit17: quantize its output for the next bundle, 20-23: exp table shift, 24-27: output fraction bits
  }
  
  int32_t * wb_bundles_p = (p_writeback + 32);
  
  for (int i=0; i < sizeof(wb_bundles)/sizeof(i32); i++){
    fb_write_reg32(wb_bundles_p + i, ((i32*)wb_bundles)[i]);
  }
  
  // Bias values into the PL bias RAM (index = i_bias = b_offset + channel)
  for (int i = 0; i < B_WORDS; i++)
    fb_write_reg32(p_writeback + WB_BIAS_BASE + i, (i32)mp->b[i]);

  // exp table of the PL softmax: tab[j] = exp(-j/2^SM_TAB_F) in Q24, shared by all softmax bundles
  for (int i = 0; i < SM_TAB_N; i++)
    fb_write_reg32(p_writeback + WB_SM_TAB_BASE + i, (i32)sm_tab[i]);

  fb_write_reg32(p_writeback + WB_A_START, 1);
}

extern EXT_C void run(Memory_st *restrict mp) {

  model_setup(mp);                            // weights, bias, bundle headers, exp table -> PL
  fb_write_reg32(p_config + A_START, 1);      // from here the PL does everything: DMAs, conv, bias, activation, residual, softmax, pooling,
                                              // tiling of the outputs into DDR for the next bundle, final y.

#ifdef SIM
  // Simulation only: dump what the PL has put in DDR, bundle by bundle, for the python checks. The out buffers are reused every N_OUT_BUF
  // bundles, so each one is dumped as soon as the PL reports the bundle as complete (WB_A_BUNDLES_DONE).
  for (i32 ib = 0; ib < N_BUNDLES-1; ib++) {
    if (bundles[ib].ib_out == -1) continue;
    while ((i32)fb_read_reg32(p_writeback + WB_A_BUNDLES_DONE) <= ib) {}
    char f_path_packed [1000];
    sprintf(f_path_packed, "%s/%0d_y_packed_sim.bin", DATA_DIR, ib);
    FILE *fp_packed = fopen(f_path_packed, "wb");
    fwrite(mp->out_buffers[bundles[ib].out_buffer_idx], 1, bundles[ib].o_bytes, fp_packed);
    fclose(fp_packed);
  }
#endif

  while (!(fb_read_reg32(p_writeback + WB_A_ALL_DONE) & 1)) {}     // the last y word is in DDR

#ifdef SIM
  {
    const Bundle_t *pb = &bundles[N_BUNDLES-1];
    char f_path_y [1000];
    sprintf(f_path_y, "%s/%0d_y_tiled_sim.txt", DATA_DIR, N_BUNDLES-1);
    FILE *fp_y = fopen(f_path_y, "w");
    for (i32 i=0; i<pb->o_words; i++)
      sim_fprintf(fp_y,"%d\n", pb->is_softmax ? (i32)(mp->y[i] * (1 << SM_FRAC_BITS)) : (i32)mp->y[i]);
    fclose(fp_y);
  }
#endif
}

extern EXT_C void print_output (Memory_st *restrict mp) {
  flush_cache(mp->y, sizeof(mp->y));
  for (int i=0; i<O_WORDS; i++){
    int val_1000 = (int)(1000 * mp->y[i]);
    printf("y[%d]: %d/1000 \n", i, val_1000);
  }
}