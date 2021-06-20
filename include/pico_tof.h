#ifndef PICO_TOF
#define PICO_TOF

#include "hardware/i2c.h"
#include "pico/stdlib.h"

typedef enum {SHORT = 0, MEDIUM = 1, LONG = 2} tof_range_t;

typedef struct _ToF
{
    i2c_inst_t* bus;
    uint32_t macro_period_us;
    uint32_t ts;
    
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;
    uint16_t fast_osc_freq;
    uint16_t osc_calibrate_val;
    bool calibrated;
    uint8_t saved_vhv_init;
    uint8_t saved_vhv_timeout;
    tof_range_t distance_mode;
    uint8_t last_status;

} ToF;

int tof_init(ToF*, i2c_inst_t*, uint, uint, uint);
int tof_set_range(ToF*, tof_range_t);
int tof_set_timing_budget(ToF*, uint32_t);

#endif /* PICO_TOF */
