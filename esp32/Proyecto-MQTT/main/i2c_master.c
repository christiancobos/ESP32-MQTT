// EJERCICIO EF11
// MSEEI-UMA


#include "driver/i2c.h"
#include "esp_err.h"

static int _i2c_master_port=-1;

// FUNCI�N para CONFIGURACI�N e INSTALACI�N del DRIVER I2C ESP32
esp_err_t i2c_master_init(int i2c_master_port, gpio_num_t sda_io_num,
                                    gpio_num_t scl_io_num,  gpio_pullup_t pull_up_en, bool fastMode)
{

	if (_i2c_master_port < 0)
	{
		i2c_config_t conf = {

				.mode = I2C_MODE_MASTER,
				.sda_io_num = sda_io_num,
				.sda_pullup_en = pull_up_en,
				.scl_io_num = scl_io_num,
				.scl_pullup_en = pull_up_en,
				.master.clk_speed = fastMode ? 400000 : 100000
		};

		i2c_param_config(i2c_master_port, &conf);
		esp_err_t err=i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
		if (err==0)
		{
				_i2c_master_port=i2c_master_port;
				//err=i2c_set_timeout(i2c_master_port, 1000000);
		}

	    return err;
	}
	else
	{
		if (_i2c_master_port==i2c_master_port)
			return 0; //puerto ya inicializado antes....
		else
			return -1; //otro puerto inicializado antes, devuelve error....(control de errores basico podria mejorar)
	}
}

 // FUNCI�N de CIERRE del DRIVER I2C ESP32
esp_err_t i2c_master_close(void)
{

	esp_err_t err=i2c_driver_delete(_i2c_master_port);
	_i2c_master_port=-1;
	return err;
}
