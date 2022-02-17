#pragma once

#include "app_config.h"

#include "util/audio_buffer.h"
#include "mic_array.h"
#include "mic_array/etc/filters_default.h"

#include <stdint.h>


#define I2S_CLKBLK XS1_CLKBLK_3

#define FRAME_BUFFERS 2


#ifdef __XC__
#  define static_const  static const
extern "C" {
#else
#  define static_const const
#endif // __XC__


streaming_channel_t app_s_chan_alloc();

void app_dac3101_init();


#ifdef __XC__
}

#endif // __XC__

void eat_audio_frames_task(
    chanend_t c_from_decimator,
    static_const unsigned frame_words);

void receive_and_buffer_audio_task(
    chanend_t c_from_decimator,
    audio_ring_buffer_t* output_buffer,
    const unsigned mic_count,
    const unsigned samples_per_frame,
    static_const unsigned frame_words);
    