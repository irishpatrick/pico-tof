#ifdef VL53L1X

#include "pico_tof.h"
#include "vl53lx_register_map.h"

#include <string.h>

#define I2C_ADDR 0x52
#define TIMING_GUARD (uint32_t)4528
#define TARGET_RATE 0x0A00

static inline uint32_t millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void start_timeout(ToF* sensor)
{
    sensor->ts = millis();
}

static bool check_timeout_expired(ToF* sensor)
{
    return (sensor->io_timeout > 0) && ((uint16_t)(millis() - sensor->ts) > sensor->io_timeout);
}

/**
 * This code below is terrible, TODO change ASAP
 */
static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    uint32_t lsb = 0;
    uint16_t msb = 0;

    if (timeout_mclks > 0)
    {
        lsb = timeout_mclks - 1;

        while ((lsb & 0xFFFFFF00) > 0)
        {
            lsb >>= 1;
            ++msb;
        }

        return (msb << 8) | (lsb & 0xFF);
    }
    else
    {
        return 0;
    }
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

static uint32_t calc_macro_period(ToF* sensor, uint8_t vcsel_period)
{
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / sensor->fast_osc_freq;
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

static uint8_t rd8(ToF* sensor, uint16_t reg)
{
    uint8_t buf[2];
    uint8_t dst;

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = (reg >> 0) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 2, false);
    
    i2c_read_blocking(sensor->bus, I2C_ADDR, &dst, 1, false);

    return dst;
}

static uint16_t rd16(ToF* sensor, uint16_t reg)
{
    uint16_t dst;
    uint8_t buf[2];

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = (reg >> 0) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 2, false);
   
    i2c_read_blocking(sensor->bus, I2C_ADDR, buf, 2, false);
    dst = (uint16_t)buf[0] << 8 | (buf[1] & 0xFF);

    return dst;
}

static uint32_t inline rd32(ToF* sensor, uint32_t reg)
{
    uint32_t dst;
    uint8_t buf[4];
    
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = (reg >> 0) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 2, false);
    
    i2c_read_blocking(sensor->bus, I2C_ADDR, buf, 4, false);
    dst = 
        buf[0] << 24 & 0xFF000000 | 
        buf[1] << 16 & 0x00FF0000 | 
        buf[2] <<  8 & 0x0000FF00 |
        buf[3] <<  0 & 0x000000FF;

    return dst;
}

static void inline wr8(ToF* sensor, uint16_t reg, uint8_t val)
{
    uint8_t buf[2];

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = (reg >> 0) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 2, false);

    i2c_write_blocking(sensor->bus, I2C_ADDR, &val, 1, false);
}

static void inline wr16(ToF* sensor, uint16_t reg, uint16_t val)
{
    uint8_t buf[2];

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = (reg >> 0) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 2, false);

    buf[0] = (val >> 8) & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 2, false);
}

static void inline wr32(ToF* sensor, uint16_t reg, uint32_t val)
{
    uint8_t buf[4];

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = (reg >> 0) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 2, false);

    buf[0] = (val >> 24) & 0xFF;
    buf[1] = (val >> 16) & 0xFF;
    buf[2] = (val >>  8) & 0xFF;
    buf[3] = (val >>  0) & 0xFF;
    i2c_write_blocking(sensor->bus, I2C_ADDR, buf, 4, false);
}

int tof_init(ToF* sensor, i2c_inst_t* bus, uint sda_pin, uint scl_pin, uint baud)
{
    i2c_init(bus, baud);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    sensor->bus = bus;

    if (rd16(sensor, VL53LX_IDENTIFICATION__MODEL_ID) != 0xEACC)
    {
        return false;
    }

    wr8(sensor, VL53LX_SOFT_RESET, 0x00);
    sleep_us(100);
    wr8(sensor, VL53LX_SOFT_RESET, 0x01);
    sleep_ms(1);

    start_timeout(sensor);

    while ((rd8(sensor, VL53LX_FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 || sensor->last_status != 0)
    {
        if (check_timeout_expired(sensor))
        {
            sensor->did_timeout = true;
            return 1;
        }
    }

    // 2V8 logic level
    wr8(sensor, VL53LX_PAD_I2C_HV__EXTSUP_CONFIG, rd8(sensor, VL53LX_PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);

    sensor->fast_osc_freq = rd16(sensor, VL53LX_OSC_MEASURED__FAST_OSC__FREQUENCY);
    sensor->osc_calibrate_val = rd16(sensor, VL53LX_RESULT__OSC_CALIBRATE_VAL);

    wr16(sensor, VL53LX_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TARGET_RATE);
    wr8(sensor, VL53LX_GPIO__TIO_HV_STATUS, 0x02);
    wr8(sensor, VL53LX_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);
    wr8(sensor, VL53LX_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);
    wr8(sensor, VL53LX_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
    wr8(sensor, VL53LX_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
    wr8(sensor, VL53LX_ALGO__RANGE_MIN_CLIP, 0);
    wr8(sensor, VL53LX_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);

    wr16(sensor, VL53LX_SYSTEM__THRESH_RATE_HIGH, 0x0000);
    wr16(sensor, VL53LX_SYSTEM__THRESH_RATE_LOW, 0x0000);
    wr8(sensor, VL53LX_DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

    wr16(sensor, VL53LX_RANGE_CONFIG__SIGMA_THRESH, 360);
    wr16(sensor, VL53LX_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);

    wr8(sensor, VL53LX_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
    wr8(sensor, VL53LX_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
    wr8(sensor, VL53LX_SD_CONFIG__QUANTIFIER, 2);

    wr8(sensor, VL53LX_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
    wr8(sensor, VL53LX_SYSTEM__SEED_CONFIG, 0x01);

    wr8(sensor, VL53LX_SYSTEM__SEQUENCE_CONFIG, 0x8B);
    wr16(sensor, VL53LX_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
    wr8(sensor, VL53LX_DSS_CONFIG__ROI_MODE_CONTROL, 2);

    tof_set_range(sensor, LONG);

    tof_set_timing_budget(sensor, 50000); // 50 ms

    wr16(sensor, VL53LX_ALGO__PART_TO_PART_RANGE_OFFSET_MM, rd16(sensor, VL53LX_MM_CONFIG__OUTER_OFFSET_MM) * 4);

    return 0;
}

int tof_set_range(ToF* sensor, tof_range_t range)
{
    uint8_t period_a;
    uint8_t period_b;
    uint8_t phase_hi;
    uint8_t woi_sd0;
    uint8_t woi_sd1;
    uint8_t phase_sd0;
    uint8_t phase_sd1;

    switch (range)
    {
        case SHORT:
            period_a = 0x0b;
            period_b = 0x09;
            phase_hi = 0x78;
            woi_sd0 = 0x0b;
            woi_sd1 = 0x09;
            phase_sd0 = 10;
            phase_sd1 = 10;
            break;

        case MEDIUM:
            period_a = 0x0b;
            period_b = 0x09;
            phase_hi = 0x78;
            woi_sd0 = 0x0b;
            woi_sd1 = 0x09;
            phase_sd0 = 10;
            phase_sd1 = 10;
            break;

        case LONG:
            period_a = 0x0b;
            period_b = 0x09;
            phase_hi = 0x78;
            woi_sd0 = 0x0b;
            woi_sd1 = 0x09;
            phase_sd0 = 10;
            phase_sd1 = 10;
            break;

        default:
            return 1;
    }

    wr8(sensor, VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A, period_a);
    wr8(sensor, VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B, period_b);
    wr8(sensor, VL53LX_RANGE_CONFIG__VALID_PHASE_HIGH, phase_hi);
    wr8(sensor, VL53LX_SD_CONFIG__WOI_SD0, woi_sd0);
    wr8(sensor, VL53LX_SD_CONFIG__WOI_SD1, woi_sd1);
    wr8(sensor, VL53LX_SD_CONFIG__INITIAL_PHASE_SD0, phase_sd0);
    wr8(sensor, VL53LX_SD_CONFIG__INITIAL_PHASE_SD1, phase_sd1);

    return 0;
}

int tof_set_timing_budget(ToF* sensor, uint32_t budget_us)
{
    uint32_t range_config_timeout_us = budget_us -= TIMING_GUARD;
    range_config_timeout_us /= 2;   

    uint32_t macro_period_us;

    macro_period_us = calc_macro_period(sensor, rd8(sensor, VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A));

    uint32_t phasecal_timeout_mclks = timeout_us_to_mclks(1000, macro_period_us);
    wr8(sensor, VL53LX_PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

    wr16(sensor, VL53LX_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, encode_timeout(
                timeout_us_to_mclks(range_config_timeout_us, macro_period_us)));

    macro_period_us = calc_macro_period(sensor, rd8(sensor, VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B));

    wr16(sensor, VL53LX_MM_CONFIG__TIMEOUT_MACROP_B_HI, encode_timeout(
            timeout_us_to_mclks(1, macro_period_us)));

    wr16(sensor, VL53LX_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, encode_timeout(
            timeout_us_to_mclks(range_config_timeout_us, macro_period_us)));

    return 0;
}

int tof_set_roi_size(ToF* sensor, uint8_t width, uint8_t height)
{
    if (width > 16)
    {
        width = 16;
    }

    if (height > 16)
    {
        height = 16;
    }

    if (width > 10 || height > 10)
    {
        wr8(sensor, VL53LX_ROI_CONFIG__USER_ROI_CENTRE_SPAD, 199);
    }

    wr8(sensor, VL53LX_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (height - 1) << 4 | (width - 1));
    return 0;
}

void tof_set_roi_center(ToF* sensor, uint8_t spad_n)
{
    wr8(sensor, VL53LX_ROI_CONFIG__USER_ROI_CENTRE_SPAD, spad_n);
}

void tof_start_continuous(ToF* sensor, uint32_t period_ms)
{
    wr32(sensor, VL53LX_SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * sensor->osc_calibrate_val);

    wr8(sensor, VL53LX_SYSTEM__INTERRUPT_CLEAR, 0x01);
    wr8(sensor, VL53LX_SYSTEM__MODE_START, 0x80);
}

void tof_stop_continuous(ToF* sensor)
{
    wr8(sensor, VL53LX_SYSTEM__MODE_START, 0x80);
    sensor->calibrated = false;

    if (sensor->saved_vhv_init != 0)
    {
        wr8(sensor, VL53LX_VHV_CONFIG__INIT, sensor->saved_vhv_init);
    }

    if (sensor->saved_vhv_timeout != 0)
    {
        wr8(sensor, VL53LX_PHASECAL_CONFIG__OVERRIDE, 0x00);
    }

    wr8(sensor, VL53LX_PHASECAL_CONFIG__OVERRIDE, 0x00);
}

int tof_get_status_string(ToF* sensor, char* dest)
{
    switch (sensor->range_data.status)
    {
        default:
            strcpy(dest, "Unknown");
            break;
    }
    return 0;
}



#endif

