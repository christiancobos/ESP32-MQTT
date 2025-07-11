// EJERCICIO EF11
// MSEEI-UMA
#ifndef MAIN_I2C_MASTER_H_
#define MAIN_I2C_MASTER_H_

#include "driver/i2c.h"
#include "esp_err.h"

extern esp_err_t i2c_master_init(int i2c_master_port, gpio_num_t sda_io_num,
                                    gpio_num_t scl_io_num,  gpio_pullup_t pull_up_en, bool fastMode);


extern esp_err_t i2c_master_close(void);

#endif /* MAIN_I2C_MASTER_H_ */
