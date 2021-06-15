#ifndef PICO_TOF
#define PICO_TOF

#include "hardware/i2c.h"
#include "pico/stdlib.h"

typedef struct _ToF
{
    i2c_inst_t* bus;
} ToF;

int tof_init(ToF*, i2c_inst_t*, uint, uint, uint);

#endif /* PICO_TOF */
