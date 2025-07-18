/* Console example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "cmd_system.h"
#include "miscomandos.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"

#include <time.h>

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "wifi.h"
#include "tcp_server.h"

#include "bsp/esp-bsp.h"
#include "lvgl_demo_ui.h"
#include "gpio_leds.h"
#include "adc_simple.h"

#include "mqtt.h"
#include "bluetooth.h"
#include "ds1621driver.h"
#include "i2c_master.h"


//TAG para los mensajes de consola
static const char *TAG = "example";

// Flag para la instalación de la rutina de interrupción.
#define ESP_INTR_FLAG_DEFAULT 0

#define I2C_MASTER_SCL_IO           27      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           26      /*!< GPIO number used for I2C master data  */


/* Console command history can be stored to and loaded from a file.
 * The easiest way to do this is to use FATFS filesystem on top of
 * wear_levelling library.
 */
#if CONFIG_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_STORE_HISTORY

//Inicializa el interprete de comandos/consola serie
static void initialize_console(void)
{
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .source_clk = UART_SCLK_DEFAULT,
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );
    ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

#if CONFIG_STORE_HISTORY
    /* Load command history from filesystem */
    linenoiseHistoryLoad(HISTORY_PATH);
#endif
}

extern void ui_example_lvgl_demo_init(lv_disp_t *disp);

void ClockTask (void *pvparameters)
{
	uint8_t Minute =0;
	uint8_t Hour =0;
	uint8_t Second=0 ;

	time_t now_t;
	struct tm* tm_info;


    lv_disp_t *disp=bsp_display_start();
    ui_example_lvgl_demo_init(disp);
    bsp_display_backlight_on();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
    	vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ);

    	time(&now_t);
        tm_info = localtime(&now_t);
        ui_update_clock(tm_info->tm_hour,tm_info->tm_min,tm_info->tm_sec);
//        if(Second++>60)
//    	    {
//    	       Second=0;
//    	        if(Minute++>60)
//    	        {
//    	            Minute=0;
//    	           if(Hour++>12) Hour=0;
//    	        }
//
//    	    }
//    	ui_update_clock(Hour,Minute,Second);
    }

}



void app_main(void)
{

	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Inicializa el driver del I2C
	// Utilizacion de los pines GPIO_NUM_26--> SDA GPIO_NUM_27--> SCL
	ret = i2c_master_init(I2C_NUM_0, I2C_MASTER_SDA_IO , I2C_MASTER_SCL_IO, GPIO_PULLUP_ENABLE, false);
	ESP_ERROR_CHECK(ret);

	// CONFIGURACION SENSOR DS1621
	ret = ds1621_i2c_master_init(0, I2C_NUM_0);
	ESP_ERROR_CHECK(ret);
	// DISPARO DE LA CONVERSI�N EN MODO CONTINUO DS1621
	while(ds1621_config(DS1621_CTRL_POL_HIGH) != ESP_OK){
		ESP_LOGE(TAG, "ERROR ds1621Config");
		vTaskDelay(0.25 * configTICK_RATE_HZ);
	}

	ESP_LOGI(TAG, "DS1621 initialized");

	while(ds1621_start_conversion()){
		ESP_LOGE(TAG, "ERROR ds1621StartConvertion");
		vTaskDelay(0.25 * configTICK_RATE_HZ);
	}

	ESP_LOGI(TAG, "DS1621 conversion done");

	//Inicializa el GPIO
	GL_initGPIO(); //Inicializa los pines de salida

	//Inicializa los pines de entrada de los botones.
	gpio_config_t io_conf = {
	    .pin_bit_mask = (1ULL << GPIO_NUM_25) | (1ULL << GPIO_NUM_14) ,
	    .mode = GPIO_MODE_INPUT,
	    .pull_up_en = GPIO_PULLUP_ENABLE,
	    .pull_down_en = GPIO_PULLDOWN_DISABLE,
	    .intr_type = GPIO_INTR_ANYEDGE
	};
	gpio_config(&io_conf);

    //Instalar rutina de tratamiento de interrupciones de GPIO.
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //Solución EF41: Anclar rutina a los pines seleccionados para los botones (GPIO25 y GPIO14)
    gpio_isr_handler_add(GPIO_NUM_25, gpio_isr_handler, (void*) GPIO_NUM_25);
    gpio_isr_handler_add(GPIO_NUM_14, gpio_isr_handler, (void*) GPIO_NUM_14);

    // Deshabilitación inicial de interrupciones
    gpio_intr_disable(GPIO_NUM_25);
    gpio_intr_disable(GPIO_NUM_14);

	//
	adc_simple_init();


#if CONFIG_STORE_HISTORY
    initialize_filesystem();
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif


    xTaskCreatePinnedToCore(ClockTask,"clock",4096,NULL,1,NULL,1);

    //Inicializa la biblioteca WiFi (crea el DEFAULT event loop()
    wifi_initlib();

    //Conecta a la Wifi o se pone en modo AP
#if CONFIG_EXAMPLE_CONNECT_WIFI_STATION
    wifi_init_sta();
#else // CONFIG_EXAMPLE_CONNECT_WIFI_STATION
    wifi_init_softap();
#endif

    //Inicializa el bluetooth.
    bluetooth_init();

    //inicializa la consola
    initialize_console();

    /* Register commands */
    esp_console_register_help_command();
    register_system();
    init_MisComandos();


    //Inicializa el servidor TCP
    initTCPServer();

    // Arranca el cliente MQTT.
#if CONFIG_EXAMPLE_CONNECT_WIFI_STATION
    mqtt_app_start(MQTT_BROKER_URL);
    // error control missing SHOULD BE ADDED.
#endif

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char* prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;

    printf("\n"
           "This is an example of ESP-IDF console component.\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n");



    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "esp32> ";
#endif //CONFIG_LOG_COLORS
    }

    /* Main loop (ejecuta el interprete de comandos)*/
    while(true) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char* line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
        /* Save command history to filesystem */
        linenoiseHistorySave(HISTORY_PATH);
#endif

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}


