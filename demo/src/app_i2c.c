
#include <stdint.h>

#include <platform.h>


#include "i2c.h"
#include "i2s.h"

#include <stdlib.h>
#include <assert.h>

#define PDM_CLKBLK_1  XS1_CLKBLK_1
#define PDM_CLKBLK_2  XS1_CLKBLK_2

i2c_master_t i2c_context;

#if XVF3610_Q60A
#include "dac3101/dac3101.h"
#define codec_init  dac3101_init
#elif XCOREAI_EXPLORER
#include "aic3204/aic3204.h"
#define codec_init  aic3204_init
#endif

void i2c_init()
{
  int res = 0;

  i2c_master_init(&i2c_context,
                  PORT_I2C_SCL, 0, 0,
                  PORT_I2C_SDA, 0, 0,
                  100);
  assert( res == 0 );


  res = codec_init(&i2c_context);
  assert( res == 0 );


}
