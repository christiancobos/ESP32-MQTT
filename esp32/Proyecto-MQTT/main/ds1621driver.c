// EJERCICIO EF11
// MSEEI-UMA

#include "ds1621driver.h"

//****************************************************************************
//      VARIABLES
//****************************************************************************

static unsigned char ds1621_slave_address = 0;
static i2c_port_t ds1621_i2c_port;

//****************************************************************************
//     FUNCIONES
//****************************************************************************

esp_err_t ds1621_i2c_master_init(uint8_t address, i2c_port_t i2c_master_port)
{


    ds1621_slave_address = DS1621_BASE_ADDRESS | (address & 0x7);
    ds1621_i2c_port = i2c_master_port;

    return 0;

}

esp_err_t ds1621_config(uint8_t config)
{
    unsigned char data_buffer[2];
    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    data_buffer[0] = DS1621_CMD_ACCESS_CONFIG;
    data_buffer[1] = config;
    i2c_master_write(cmd, data_buffer, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};

esp_err_t ds1621_read_temperature_low_resolution(float* temperature)
{
    unsigned char temp_low_resolution[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_READ_TEMP, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, temp_low_resolution, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);

    *temperature = (temp_low_resolution[1] != 0) ? (float)temp_low_resolution[0] : (float)(temp_low_resolution[0] + 0.5);

	return ret;
};

esp_err_t ds1621_write_TH(float temperature)
{
    uint8_t data_buffer[2];
    int16_t value;
   // Conversion de la temperatura a valores a escribir en el comando TH
    value= (int16_t)(temperature*2.0);
    data_buffer[0]=(value>>1)&0xFF; // 8 bits m�s significativos
    data_buffer[1]= (value & 0x01)<<7; // noveno bit

    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_ACCESS_TH, true);
    i2c_master_write(cmd, data_buffer, sizeof(data_buffer), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS/1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};

esp_err_t ds1621_write_TL(float temperature)
{
    uint8_t data_buffer[2];
    int16_t value;
   // Conversion de la temperatura a valores a escribir en el comando TH
    value= (int16_t)(temperature*2.0);
    data_buffer[0]=(value>>1)&0xFF; // 8 bits m�s significativos
    data_buffer[1]= (value & 0x01)<<7; // noveno bit

    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_ACCESS_TL, true);
    i2c_master_write(cmd, data_buffer, sizeof(data_buffer), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};


esp_err_t ds1621_read_TH(float* temperature)
{
    uint8_t buffer[2];
    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_ACCESS_TH, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    *temperature = ((((int16_t)((int8_t)buffer[0])) << 1) | (buffer[1] >> 7)) / 2.0;

    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};

esp_err_t ds1621_read_counter(uint8_t* counter)
{
	if(ds1621_slave_address == 0)
	        return ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_READ_COUNTER, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, counter, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);


	return ret; 	  // Definir como en la funcion ds1621_config()
				  // una variable de retorno de la funcion i2c_master_cmd_begin()
};

esp_err_t ds1621_read_slope(uint8_t* slope)
{
	if(ds1621_slave_address == 0)
	        return ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_READ_SLOPE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, slope, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);

	return ret; 	  // Definir como en la funcion ds1621_config()
				  // una variable de retorno de la funcion i2c_master_cmd_begin()
};

esp_err_t ds1621_start_conversion(void)
{
	if(ds1621_slave_address == 0)
	        return ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_START_CONVERT, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);

    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);

	return ret; 	  // Definir como en la funcion ds1621_config()
				  // una variable de retorno de la funcion i2c_master_cmd_begin()
};

esp_err_t ds1621_stop_conversion(void)
{
	if(ds1621_slave_address == 0)
	        return ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_STOP_CONVERT, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);

	return ret;
};

esp_err_t ds1621_read_temperature_high_resolution(float* temperature)
{
	if(ds1621_slave_address == 0)
	        return ESP_FAIL;

	uint8_t counter, slope;
	float temp_low_res;

	ds1621_start_conversion();
	vTaskDelay(0.75 * configTICK_RATE_HZ);
	ds1621_read_slope(&slope);
	ds1621_read_counter(&counter);
	ds1621_read_temperature_low_resolution(&temp_low_res);
	ds1621_stop_conversion();

	*temperature = ((float)((uint8_t)temp_low_res) - 0.25) + ((float)(slope - counter) / (float)slope);

	return 0; 	  // Definir como en la funcion ds1621_config()
				  // una variable de retorno de la funcion i2c_master_cmd_begin()
};
