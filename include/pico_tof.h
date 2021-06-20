#ifndef PICO_TOF
#define PICO_TOF

#include "hardware/i2c.h"
#include "pico/stdlib.h"

typedef enum {SHORT = 0, MEDIUM = 1, LONG = 2} tof_range_t;
typedef enum {VALID = 0, SIGMAFAIL = 1, SIGNALFAIL = 2, MINCLIP = 3, OOB = 4, HWFAIL = 5, NOWRAPFAIL = 6, XTALKFAIL = 7, SYNCINT = 8, MINRANGEFAIL = 9, NONE = 10, UNKNOWN = 11} range_status_t;

typedef struct _RangeData
{
    uint16_t range_mm;
    range_status_t status;
    float peak_signal_count_rate_mcps;
    float ambient_count_rate_mcps;
} RangeData;

typedef struct _ResultBuffer
{
    uint8_t range_status;
    uint8_t stream_count;
    uint16_t dss_actual_effective_spads_sd0;
    uint16_t ambient_count_rate_mcps_sd0;
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
} ResultBuffer;

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
    bool blocking;

    ResultBuffer result_buf;
    RangeData range_data;
} ToF;

int tof_init(ToF*, i2c_inst_t*, uint, uint, uint);
int tof_set_range(ToF*, tof_range_t);
int tof_set_timing_budget(ToF*, uint32_t);
int tof_set_roi_size(ToF*, uint8_t, uint8_t);
void tof_start_continuous(ToF*, uint32_t);
void tof_stop_continuous(ToF*);
int tof_get_status_string(ToF*, char*);

#endif /* PICO_TOF */

