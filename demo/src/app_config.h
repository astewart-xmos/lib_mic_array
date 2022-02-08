#pragma once


#define N_MICS  1

#define SAMPLES_PER_FRAME         16


#define AUDIO_BUFFER_SAMPLES       17

#define APP_AUDIO_CLOCK_FREQUENCY        24576000
#define APP_AUDIO_PIPELINE_SAMPLE_RATE   16000


#define APP_I2S_AUDIO_SAMPLE_RATE   APP_AUDIO_PIPELINE_SAMPLE_RATE

#define APP_USE_DC_OFFSET_ELIMINATION   1


#define MIC_ARRAY_CLK1  XS1_CLKBLK_1
#define MIC_ARRAY_CLK2  XS1_CLKBLK_2