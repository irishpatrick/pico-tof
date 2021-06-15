#ifdef VL53L3CX

#include "pico_tof.h"

#define I2C_ADDR 0x52

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

#endif
