#ifdef VL53LX

#include "pico_tof.h"
#include "vl53lx_register_map.h"

#define I2C_ADDR 0x52

static uint8_t rd8(ToF* sensor, uint16_t reg)
{
    uint8_t dst;
    i2c_write_blocking(sensor->bus, (reg >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (reg >> 0) & 0xFF, 1, false);
    
    i2c_read_blocking(sensor->bus, val, 1, false);
    return dst;
}

static uint16_t rd16(ToF* sensor, uint16_t reg)
{
    uint16_t dst;
    uint8_t buf[2];
    i2c_write_blocking(sensor->bus, (reg >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (reg >> 0) & 0xFF, 1, false);
   
    i2c_read_blocking(sensor->bus, buf, 2, false);
    dst = (uint16_t)buf[0] << 8 | (buf[1] & 0xFF);
    return dst;
}

static uint32_t inline rd32(ToF* sensor, uint32_t reg)
{
    uint32_t dst;
    uint8_t buf[4];
    i2c_write_blocking(sensor->bus, (reg >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (reg >> 0) & 0xFF, 1, false);
    
    i2c_read_blocking(sensor->bus, buf, 4, false);
    dst = 
        buf[0] << 24 & 0xFF000000 | 
        buf[1] << 16 & 0x00FF0000 | 
        buf[2] <<  8 & 0x0000FF00 |
        buf[3] <<  0 & 0x000000FF;

    return dst;
}

static void inline wr8(ToF* sensor, uint16_t reg, uint8_t val)
{
    i2c_write_blocking(sensor->bus, (reg >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (reg >> 0) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, val, 1, ftrue);
}

static void inline wr16(ToF* sensor, uint16_t reg, uint16_t val)
{
    i2c_write_blocking(sensor->bus, (reg >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (reg >> 0) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (val >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (val >> 0) & 0xFF, 1, false);
}

static void inline wr32(ToF* sensor, uint16_t reg, uint32_t val)
{
    i2c_write_blocking(sensor->bus, (reg >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (reg >> 0) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (val >> 24) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (val >> 16) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (val >> 8) & 0xFF, 1, true);
    i2c_write_blocking(sensor->bus, (val >> 0) & 0xFF, 1, false);
}

int tof_init(ToF* sensor, i2c_inst_t* bus, uint sda_pin, uint scl_pin, uint baud)
{
    i2c_init(bus, baud);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    tof->bus = bus;

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

    wr8(sensor, RANGE_CONFIG__VCSEL_PERIOD_A, period_a);
    wr8(sensor, RANGE_CONFIG__VCSEL_PERIOD_B, period_b);
    wr8(sensor, RANGE_CONFIG__VALID_PHASE_HIGH, phase_hi);
    wr8(sensor, SD_CONFIG__WOI_SD0, woi_sd0);
    wr8(sensor, SD_CONFIG__WOI_SD1, woi_sd1);
    wr8(sensor, SD_CONFIG__INITIAL_PHASE_SD0, phase_sd0);
    wr8(sensor, SD_CONFIG__INITIAL_PHASE_SD1, phase_sd1);

    return 0;
}

int tof_set_timing_budget(ToF* sensor, uint32_t budget_us)
{
    
    return 0;
}

#endif
