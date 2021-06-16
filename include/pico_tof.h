#ifndef PICO_TOF
#define PICO_TOF

#include "hardware/i2c.h"
#include "pico/stdlib.h"

typedef enum {SHORT = 0, MEDIUM = 1, LONG = 2} tof_range_t;

typedef struct _ToF
{
    i2c_inst_t* bus;
} ToF;

int tof_init(ToF*, i2c_inst_t*, uint, uint, uint);
int tof_set_range(ToF*, tof_range_t);
int tof_set_timing_budget(ToF*, uint32_t);

#endif /* PICO_TOF */
