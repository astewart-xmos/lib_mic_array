
#include "app_config.h"
#include "app_mic_array.h"
#include "pdm_rx.h"

#include <platform.h>
#include <xs1.h>
#include <xclib.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>


// Mic array config
pdm_rx_config_t ma_config;

uint32_t pdm_buffer[ MA_PDM_BUFFER_SIZE_WORDS( N_MICS, STAGE2_DEC_FACTOR ) ];


struct {
  xs3_filter_fir_s32_t filter[N_MICS];
  int32_t state_buffer[N_MICS][STAGE2_TAP_COUNT];
} stage2_filters;


#define MIC_ARRAY_CLK1  XS1_CLKBLK_1
#define MIC_ARRAY_CLK2  XS1_CLKBLK_2

// MCLK connected to pin 14 --> X1D11 --> port 1D
// MIC_CLK connected to pin 39 --> X1D22 --> port 1G
// MIC_DATA connected to pin 32 --> X1D13 --> port 1F
port_t p_mclk     = XS1_PORT_1D;
port_t p_pdm_clk  = XS1_PORT_1G;
port_t p_pdm_mics = XS1_PORT_1F;


// Divider to bring the 24.576 MHz clock down to 3.072 MHz
#define MCLK_DIV  8


void app_mic_array_setup_resources()
{

  ma_config.mic_count = N_MICS;
  ma_config.stage1.p_pdm_mics = (unsigned) p_pdm_mics;
  ma_config.stage1.filter_coef = (uint32_t*) stage1_coef;
  ma_config.stage1.pdm_buffers = &pdm_buffer[0];

  ma_config.stage2.decimation_factor = STAGE2_DEC_FACTOR;
  ma_config.stage2.filters = &stage2_filters.filter[0];


  unsigned div = MCLK_DIV;
  if(N_MICS == 4)
    div >>= 1;
  else if(N_MICS == 8)
    div >>= 2;

  if( N_MICS == 1 ){
    mic_array_setup_sdr((unsigned) MIC_ARRAY_CLK1,
                        (unsigned) p_mclk, (unsigned) p_pdm_clk,
                        (unsigned) p_pdm_mics, div);
  } else if( N_MICS >= 2 ){
    mic_array_setup_ddr((unsigned) MIC_ARRAY_CLK1, (unsigned) MIC_ARRAY_CLK2,
                        (unsigned) p_mclk, (unsigned) p_pdm_clk,
                        (unsigned) p_pdm_mics, div );
  } else {
    assert(0);
  }


}

static void init_filters(
    pdm_rx_config_t* config)
{
  for(int k = 0; k < config->mic_count; k++){
    xs3_filter_fir_s32_init(&stage2_filters.filter[k],
                            &stage2_filters.state_buffer[k][0],
                            STAGE2_TAP_COUNT,
                            stage2_coef,
                            STAGE2_SHR);
  }
}

void app_mic_array_start()
{
  if( N_MICS == 1 ){
    mic_array_start_sdr((unsigned) MIC_ARRAY_CLK1);
  } else if( N_MICS >= 2 ){
    mic_array_start_ddr((unsigned) MIC_ARRAY_CLK1,
                        (unsigned) MIC_ARRAY_CLK2,
                        (unsigned) p_pdm_mics );
  }
}

#include <xcore/interrupt_wrappers.h>

// DEFINE_INTERRUPT_PERMITTED(mic_array_isr_cb_grp, void, app_mic_array_task)
// {
// }

// void __xcore_interrupt_permitted_ugs_cmain(void);

DEFINE_INTERRUPT_PERMITTED(mic_array_isr_cb_grp, void, cmain)
{
  init_filters( &ma_config );
  mic_array_proc_pdm( &ma_config );
}
