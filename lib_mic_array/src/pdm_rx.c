
#include "pdm_rx.h"
#include "mic_array.h"
#include "fir_1x16_bit.h"

#include "xs3_math.h"

#include <xcore/port.h>
// #include <xcore/channel.h>
#include <xcore/channel_streaming.h>
#include <xcore/hwtimer.h>
#include <xcore/interrupt.h>
#include <xcore/triggerable.h>

#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// extern void pdm_rx_isr(void);
#include <xcore/interrupt_wrappers.h>
/*
  This struct is allocated directly in pdm_rx_isr.S
*/
static struct {
  port_t p_pdm_data;
  uint32_t* buffA;
  uint32_t* buffB;
  unsigned phase1;
  unsigned phase1_reset;
  chanend_t c_pdm_out;
  chanend_t c_pdm_in;
} pdm_rx_context;


// #include "rtos_printf.h"

#include <xcore/hwtimer.h>
static hwtimer_t tmr;
static uint32_t first_run = 0;
static uint32_t cnter = 0;

DEFINE_INTERRUPT_CALLBACK(mic_array_isr_cb_grp, pdm_rx_isr, arg)
{
    uint32_t p_val;
    uint32_t p_val2;

    if (first_run == 0) {
        first_run =  hwtimer_get_time(tmr);
    }
    p_val = port_in(pdm_rx_context.p_pdm_data);

    cnter++;
    if (cnter > 10)
    {
         printf("10 iters now %d start %d\n", hwtimer_get_time(tmr) , first_run);
        // cnter = 0;
    }
    // _Exit(0);
                // p_val = port_in(pdm_rx_context.p_pdm_data);
    // printf("-*******in isr %d\n", p_val);
    uint32_t *bufA_ptr = pdm_rx_context.buffA ;
    bufA_ptr[pdm_rx_context.phase1] = p_val;
    if(pdm_rx_context.phase1) {
        pdm_rx_context.phase1--;
    } else {
        pdm_rx_context.phase1 = pdm_rx_context.phase1_reset;
        uint32_t *tmp_ptr = pdm_rx_context.buffA;
        pdm_rx_context.buffA = pdm_rx_context.buffB;
        pdm_rx_context.buffB = tmp_ptr;
        s_chan_out_word(pdm_rx_context.c_pdm_out, (uint32_t)tmp_ptr);
    }
    // if(bits > 32)
    // printf("-*******in isr %d\n", bits);
    // printf("-*******done isr\n");
}


static inline void deinterleave_pdm_samples(
    uint32_t* samples,
    const unsigned n_mics,
    const unsigned stage2_dec_factor)
{
  switch(n_mics)
  {
    case 1:
      break;
    case 2:
      for(int k = 0; k < stage2_dec_factor; k++){
        asm volatile(
          "ldw r0, %0[1]      \n"
          "ldw r1, %0[0]      \n"
          "unzip r1, r0, 0    \n"
          "stw r0, %0[0]      \n"
          "stw r1, %0[1]        " :: "r"(samples) : "r0", "r1", "memory" );
        samples += 2;
      }
      break;
    default:
      assert(0); // TODO: not yet implemented
  }
}

/**
 * Move buff[0:7] to buff[1:8]. Don't care what ends up in buff[0]
 */
static inline void shift_buffer(uint32_t* buff)
{
  uint32_t* src = &buff[-1];
  asm volatile("vldd %0[0]; vstd %1[0];" :: "r"(src), "r"(buff) : "memory" );
}



__attribute__((weak))
void proc_pcm_user(int32_t pcm_frame[])
{
  // Do nothing...
}

void mic_array_pdm_rx_setup(
    pdm_rx_config_t* config)
{
tmr = hwtimer_alloc();
  streaming_channel_t c_pdm = s_chan_alloc();

  assert(c_pdm.end_a != 0 && c_pdm.end_b != 0);

  // 32-bits per 96k PCM sample per microphone
  // and stage2_dec_factor 96k PCM samples per pipeline sample
  unsigned buffA_size_words = config->mic_count * config->stage2.decimation_factor;

  pdm_rx_context.p_pdm_data = config->stage1.p_pdm_mics;
  pdm_rx_context.c_pdm_out = c_pdm.end_a;
  pdm_rx_context.buffA = &config->stage1.pdm_buffers[0];
  pdm_rx_context.buffB = &config->stage1.pdm_buffers[buffA_size_words];
  pdm_rx_context.phase1_reset = buffA_size_words - 1;
  pdm_rx_context.phase1 = pdm_rx_context.phase1_reset;
  pdm_rx_context.c_pdm_in = c_pdm.end_b;

  // Let's make sure interrupts are masked before we setup another
  interrupt_mask_all();
  // asm volatile(
  //      "setc res[%0], %1       \n"
  //      "ldap r11, pdm_rx_isr   \n"
  //      "setv res[%0], r11      \n"
  //      "eeu res[%0]              "
  //        :
  //        : "r"(config->stage1.p_pdm_mics), "r"(XS1_SETC_IE_MODE_INTERRUPT)
  //        : "r11" );
  triggerable_setup_interrupt_callback(config->stage1.p_pdm_mics,NULL,INTERRUPT_CALLBACK(pdm_rx_isr));
  triggerable_enable_trigger(config->stage1.p_pdm_mics);
  // Note that this does not unmask interrupts on this core

}

// DECLARE_INTERRUPT_PERMITTED(void, mic_array_proc_pdm_int, pdm_rx_config_t* config);
// NOTE: When the ISR triggers, it will immediately issue an  `extsp 4` instruction, decrementing the
//       stack pointer by 4 words (this is how it avoids clobbering registers). That means this
//       function needs 4 more words of stack space than the compiler will decide it needs.
//      Unfortunately, I know of no way to force the compiler to automatically set the symbol
//      mic_array_proc_pdm.maxstackwords to 4 more than it would otherwise. So for right now I am
//      just overriding mic_array_proc_pdm.maxstackwords to a constant much larger than it should
//      ever need.
// void mic_array_proc_pdm(
//     pdm_rx_config_t* config )
// {
//     mic_array_proc_pdm_int(config);
// }

void mic_array_proc_pdm(
    pdm_rx_config_t* config )
{
// printf("call setup\n");
  // TODO: Not sure whether this should be called inside here or if we should make the
  //       user call it before this task. I'm just putting it here now for simplicity.
  mic_array_pdm_rx_setup( config );
  // printf("setup done\n");

  // Just as a convenience for the user, the PDM receive buffers and PDM history buffers are
  // provided as one long buffer (see MA_PDM_BUFFER_SIZE_WORDS()). This constant just tells
  // us where the history part starts.
  const unsigned buffA_size_words = config->mic_count * config->stage2.decimation_factor;

  // The first stage decimator applies a 256-tap FIR filter for each channel, so we need
  // a history of 256 PDM samples (8 words) for each microphone.
  // pdm_history is where we keep that filter stage. Its shape is (MIC_COUNT, 8 words)
  uint32_t* pdm_history = &config->stage1.pdm_buffers[2*buffA_size_words];

  // Initializing the filter state to 0x55's hopefully minimizes start-up transients on the filter
  memset(pdm_history, 0x55, config->mic_count * 8 * sizeof(uint32_t));

  // A pointer to the array of second stage filters. Shape is (MIC_COUNT)
  xs3_filter_fir_s32_t* s2_filters = config->stage2.filters;

  // samples_out will be passed to proc_pcm_user(). The first MIC_COUNT elements of it will be
  // the next output samples from the second stage decimators.
  int32_t samples_out[MAX_MIC_COUNT] = {0};
  // Once we unmask interrupts, the ISR will begin triggering and collecting PDM samples. At
  // that point we need to be ready to pull PDM samples from the ISR via the c_pdm_in chanend.
// rtos_printf("unmask isrs\n");
  interrupt_unmask_all();

  while(1) {

    ////// Wait for the next batch of PDM samples from the ISR.
    // This will deliver enough PDM samples to do a first AND second stage filter
    // for each microphone channel. (Note: All we're pulling out of the channel itself
    // is a pointer to the PDM buffer.
    uint32_t* pdm_samples = (uint32_t*) s_chan_in_word(pdm_rx_context.c_pdm_in);
// rtos_printf("got samples\n");
// printf("got samples");
    ////// De-interleave the channels in the received PDM buffer.
    // Because of the way multi-bit buffered ports work, each word pulled from the port in the ISR
    // will contain (32/MIC_COUNT) PDM samples for each microphone. In order to update each
    // channel's history (first stage filter state) we need to consolidate the samples for each
    // mic into a single word. This function does that in-place.
    // The input to this function are the raw words pulled from the port in reverse order (i.e.
    // the oldest samples are at the highest index).
    // Note that because we've collected enough samples to do a new *second* stage output, we
    // have STAGE2_DEC_FACTOR *words* for *each* microphone in this buffer. Once pdm_samples
    // is boggled by this function, then pdm_samples can be treated as:
    //     uint32_t pdm_samples[STAGE2_DEC_FACTOR][MIC_COUNT]
    // with the first axis in reverse time order (pdm_samples[0][:] has the *newest* samples)
    deinterleave_pdm_samples(pdm_samples, config->mic_count, config->stage2.decimation_factor);

    ////// Iterate over microphones, producing one second stage output for each.
    // For each one we'll run the first stage decimator STAGE2_DEC_FACTOR times, and the
    // second stage decimator will be run once. The results go into samples_out[]
    for(unsigned mic = 0; mic < config->mic_count; mic++) {
      for(unsigned k = 0; k < config->stage2.decimation_factor; k++) {

        ////// Compute next streamA sample for this microphone
        int idx = (config->stage2.decimation_factor - 1 - k) * config->mic_count + mic;
        pdm_history[8*mic] = pdm_samples[idx];
        int32_t streamA_sample = fir_1x16_bit(&pdm_history[8*mic], config->stage1.filter_coef);

        ////// Rotate the PDM history vector
        // This shifts the first 7 words of the history vector up by a word, clobbering
        // whatever was in the 8th word (which isn't needed anymore).
        shift_buffer(&pdm_history[8*mic]);

        // Up until the last iteration of k we're just adding the sample to our stage2 FIR.
        // On the last iteration we'll actually produce a new output sample.
        if(k < (config->stage2.decimation_factor-1)){
          xs3_filter_fir_s32_add_sample(&s2_filters[mic], streamA_sample);
        } else {
          samples_out[mic] = xs3_filter_fir_s32(&s2_filters[mic], streamA_sample);
        }
      }
    }

    // Once we're done with all that, we just need to call proc_pcm_user()
    proc_pcm_user(samples_out);
  }
}





/////////// This version hardcodes 1 mic and s2df of 6. Uses significantly fewer MIPS
// #pragma stackfunction 400
// void mic_array_proc_pdm(
//     pdm_rx_config_t* config )
// {
//   mic_array_pdm_rx_setup( config );
//   const unsigned buffA_size_words = 6;
//   uint32_t* pdm_history = &config->stage1.pdm_buffers[2*buffA_size_words];
//   memset(pdm_history, 0x55, 1 * 8 * sizeof(uint32_t));
//   xs3_filter_fir_s32_t* s2_filters = config->stage2.filters;
//   int32_t samples_out[MAX_MIC_COUNT] = {0};
//   interrupt_unmask_all();
//   while(1) {
//     uint32_t* pdm_samples = (uint32_t*) s_chan_in_word(pdm_rx_context.c_pdm_in);
//     deinterleave_pdm_samples(pdm_samples, 1, 6);
//     for(unsigned k = 0; k < 5; k++) {
//       pdm_history[0] = pdm_samples[5-k];
//       int32_t streamA_sample = fir_1x16_bit(&pdm_history[0], config->stage1.filter_coef);
//       shift_buffer(&pdm_history[0]);
//       xs3_filter_fir_s32_add_sample(&s2_filters[0], streamA_sample);
//     }
//     for(unsigned k = 5; k < 6; k++) {
//       pdm_history[0] = pdm_samples[5-k];
//       int32_t streamA_sample = fir_1x16_bit(&pdm_history[0], config->stage1.filter_coef);
//       shift_buffer(&pdm_history[0]);
//       samples_out[0] = xs3_filter_fir_s32(&s2_filters[0], streamA_sample);
//     }
//     proc_pcm_user(samples_out);
//   }
// }
