idf_component_register(SRCS "proyecto_main.c" "miscomandos.c" "wifi.c" "tcp_server.c" "gpio_leds.c" "lvgl_demo_ui.c" "clockwidget.c" "adc_simple.c" "frozen.c" "mqtt.c" "bluetooth.c" "ds1621driver.c" "i2c_master.c"					
     INCLUDE_DIRS "."                    	
)

# IMPORTANTE: CREA la imagen.... cuidado con el nombre que debe ser el de la partisao.                    
# Create a SPIFFS image from the contents of the 'spiffs_image' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash'.
# partspiffs es el nombre que tiene en la tabla de particiones y debe coincidir con el c�digo que lo monta. 