#pragma once

#include <platform.h>

#define I2S_CLKBLK XS1_CLKBLK_3

#ifdef __XC__
extern "C" {
#endif //__XC__

void app_i2s_task();

#if XCOREAI_EXPLORER
void reset_codec(void);
#endif


#ifdef __XC__
} // extern "C"
#endif //__XC__
