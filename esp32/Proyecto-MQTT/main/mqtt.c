//Include standard lib headers
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


//Include FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

//Include ESP submodules headers.
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "adc_simple.h"

//Include own project  headers
#include "gpio_leds.h"
#include "mqtt.h"

//FROZEN JSON parsing/fotmatting library header
#include "frozen.h"

//****************************************************************************
//      DEFINES
//****************************************************************************

#define PONG_TOPIC "/pong"
#define POLL_TOPIC "/button_poll"
#define ADC_TOPIC  "/adc_read"

#define ADC_SAMPLE_PERIOD 0.2f

//****************************************************************************
//      TIPOS DE DATOS
//****************************************************************************

typedef enum{
    PING,
    POLL,
    ADC_READ
}mqtt_sendType_t;

typedef enum{
    LED_ROJO,
    LED_VERDE,
    LED_AZUL
} rgb_colourIndex_t;

typedef struct{
    mqtt_sendType_t messageType;
    char payload[128];
}mqtt_send_t; // Tipo de datos para comunicar al hilo de envío de datos el tipo de mensaje y su payload.

//****************************************************************************
//      VARIABLES GLOBALES STATIC
//****************************************************************************

static const char *TAG = "MQTT_CLIENT";
static esp_mqtt_client_handle_t client=NULL;
static TaskHandle_t senderTaskHandler=NULL;
static TaskHandle_t adcTaskHandler = NULL;
static QueueHandle_t sendQueueHandler=NULL;
static uint8_t rgbPwmValues[3] = {0};
static uint8_t binaryLEDValues[3] = {0};

//****************************************************************************
// Funciones.
//****************************************************************************

static void mqtt_sender_task(void *pvParameters);
static void adc_task(void *pvParameters);


// callback that will handle MQTT events. Will be called by  the MQTT internal task.
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_SUBSCRIBE_BASE, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            sendQueueHandler = xQueueCreate(10, sizeof(mqtt_send_t));                        // Crea la cola para comunicar datos a la tarea de envío.


            //Crea la tarea MQTT sennder
            if (xTaskCreate(mqtt_sender_task, "mqtt_sender", 4096, NULL, 5, &senderTaskHandler) != pdPASS)
            {
                while(1);
            }

            // Crea la tarea de sensado del ADC.
            if (xTaskCreate( adc_task, "Adc", configMINIMAL_STACK_SIZE + (0.5 * configMINIMAL_STACK_SIZE), NULL, tskIDLE_PRIORITY+1, &adcTaskHandler)!=pdPASS){
                while (1);
            }

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            vTaskDelete(senderTaskHandler);
            vTaskDelete(adcTaskHandler);
            //Deber�amos destruir la tarea que env�a....
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
        {
        	//Para poder imprimir el nombre del topic lo tengo que copiar en una cadena correctamente terminada....
        	char topic_name[event->topic_len+1]; //Esto va a la pila y es potencialmente peligroso si el nombre del topic es grande....
        	strncpy(topic_name,event->topic,event->topic_len);
        	topic_name[event->topic_len]=0; //a�ade caracter de terminacion al final.

        	ESP_LOGI(TAG, "MQTT_EVENT_DATA: Topic %s",topic_name);

        	bool booleano;
        	uint8_t PWM_value;

        	if(json_scanf(event->data, event->data_len, "{ redLed: %B }", &booleano)==1)
        	{
        		ESP_LOGI(TAG, "redLed: %s", booleano ? "true":"false");

        		binaryLEDValues[LED_ROJO] = booleano ? 1 : 0;

        		gpio_set_level(BLINK_GPIO_1, booleano);
        	}
        	if(json_scanf(event->data, event->data_len, "{ greenLed: %B }", &booleano)==1)
			{
				ESP_LOGI(TAG, "greenLed: %s", booleano ? "true":"false");

				binaryLEDValues[LED_VERDE] = booleano ? 1 : 0;

				gpio_set_level(BLINK_GPIO_2, booleano);
			}
        	if(json_scanf(event->data, event->data_len, "{ blueLed: %B }", &booleano)==1)
			{
				ESP_LOGI(TAG, "blueLed: %s", booleano ? "true":"false");

				binaryLEDValues[LED_AZUL] = booleano ? 1 : 0;

				gpio_set_level(BLINK_GPIO_3, booleano);
			}

        	if(json_scanf(event->data, event->data_len, "{ ping: %B }", &booleano)==1)
            {
                ESP_LOGI(TAG, "ping received: %s", booleano ? "true":"false");

                mqtt_send_t ping;
                ping.messageType = PING;

                xQueueSend(sendQueueHandler, &ping, portMAX_DELAY);
            }

        	if(json_scanf(event->data, event->data_len, "{ button_poll: %B }", &booleano)==1)
            {
                ESP_LOGI(TAG, "button poll request received: %s", booleano ? "true":"false");

                mqtt_send_t poll;
                bool boton1, boton2;
                poll.messageType = POLL;

                boton1 = gpio_get_level(GPIO_NUM_25);
                boton2 = gpio_get_level(GPIO_NUM_26);

                poll.payload[0] = boton1 ? '0' : '1'; // Lógica invertida por tener resistencia pull-up
                poll.payload[1] = boton2 ? '0' : '1'; // Lógica invertida por tener resistencia pull-up

                xQueueSend(sendQueueHandler, &poll, portMAX_DELAY);
            }

            if(json_scanf(event->data, event->data_len, "{ PWM_mode: %B }", &booleano)==1)
            {
                ESP_LOGI(TAG, "LED mode request received: %s", booleano ? "true":"false");
                if (!booleano)
                {
                    ESP_LOGI(TAG, "LED BINARY mode request received.");

                    GL_stopLEDC();

                    if (binaryLEDValues[LED_ROJO] == 1){
                        gpio_set_level(BLINK_GPIO_1, true);
                    }

                    if (binaryLEDValues[LED_VERDE] == 1){
                        gpio_set_level(BLINK_GPIO_2, true);
                    }

                    if (binaryLEDValues[LED_AZUL] == 1){
                        gpio_set_level(BLINK_GPIO_3, true);
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "LED PWM mode request received.");

                    GL_initLEDC();
                }
            }

            if(json_scanf(event->data, event->data_len, "{ PWM_Rojo: %i }", &PWM_value) == 1)
            {
                ESP_LOGI(TAG, "LED ROJO valor: %d", PWM_value);

                rgbPwmValues[0] = PWM_value;

                GL_setRGB(rgbPwmValues);
            }

            if(json_scanf(event->data, event->data_len, "{ PWM_Verde: %i }", &PWM_value) == 1)
            {
                ESP_LOGI(TAG, "LED VERDE valor: %d", PWM_value);

                rgbPwmValues[LED_VERDE] = PWM_value;

                GL_setRGB(rgbPwmValues);
            }

            if(json_scanf(event->data, event->data_len, "{ PWM_Azul: %d }", &PWM_value) == 1)
            {
                ESP_LOGI(TAG, "LED AZUL valor: %d", PWM_value);

                rgbPwmValues[LED_AZUL] = PWM_value;

                GL_setRGB(rgbPwmValues);
            }



        }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}


static void mqtt_sender_task(void *pvParameters)
{
	char buffer[100]; //"buffer" para guardar el mensaje. Me debo asegurar que quepa...
	char output_topic[100]; //string para el topic de salida.

	while (1)
	{
	    mqtt_send_t msg;
	    int msg_id;

		if(xQueueReceive(sendQueueHandler, &msg, portMAX_DELAY) == pdTRUE){
		    switch(msg.messageType){
		    case PING:
		        struct json_out out1 = JSON_OUT_BUF(buffer, sizeof(buffer)); // Inicializa la estructura que gestiona el buffer.
		        json_printf(&out1," { ping: %B }",true);
		        snprintf(output_topic, sizeof(output_topic), "%s%s", MQTT_TOPIC_PUBLISH_BASE, PONG_TOPIC);
		        msg_id = esp_mqtt_client_publish(client, output_topic, buffer, 0, 0, 0);
		        ESP_LOGI(TAG, "PING sent successfully, msg_id=%d: %s", msg_id, buffer);
		        break;
		    case POLL:
		        struct json_out out2 = JSON_OUT_BUF(buffer, sizeof(buffer)); // Inicializa la estructura que gestiona el buffer.
		        bool boton1, boton2;
		        boton1 = msg.payload[0] == '1' ? true : false;
		        boton2 = msg.payload[1] == '1' ? true : false;
		        json_printf(&out2," { button1: %B , button2: %B }", boton1, boton2);
		        snprintf(output_topic, sizeof(output_topic), "%s%s", MQTT_TOPIC_PUBLISH_BASE, POLL_TOPIC);
                msg_id = esp_mqtt_client_publish(client, output_topic, buffer, 0, 0, 0);
                ESP_LOGI(TAG, "Button poll sent successfully, msg_id=%d: %s", msg_id, buffer);
		        break;
		    case ADC_READ:
		        struct json_out out3 = JSON_OUT_BUF(buffer, sizeof(buffer));
		        uint16_t lectura;
		        lectura = atoi(msg.payload);
		        json_printf(&out3, " { adc_read: %u } ", lectura);
		        snprintf(output_topic, sizeof(output_topic), "%s%s", MQTT_TOPIC_PUBLISH_BASE, ADC_TOPIC);
		        msg_id = esp_mqtt_client_publish(client, output_topic, buffer, 0, 0, 0);
		        ESP_LOGI(TAG, "ADC read sent successfully, msg_id=%d: %s", msg_id, buffer);
		        break;

		    default:
		        break;
		    }
		}
	}
}

void adc_task(void *pvParameters){
    mqtt_send_t adc_read;
    adc_read.messageType = ADC_READ;

    for(;;)
    {
        sprintf(adc_read.payload, "%u", adc_simple_read_raw());
        xQueueSend(sendQueueHandler, &adc_read, portMAX_DELAY);
        vTaskDelay((int)(ADC_SAMPLE_PERIOD*configTICK_RATE_HZ));
    }
}

esp_err_t mqtt_app_start(const char* url)
{
	esp_err_t error;

	if (client==NULL){

		esp_mqtt_client_config_t mqtt_cfg = {
				.broker.address.uri = MQTT_BROKER_URL,
		};
		if(url[0] != '\0'){
			mqtt_cfg.broker.address.uri= url;
		}


		client = esp_mqtt_client_init(&mqtt_cfg);
		esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
		error=esp_mqtt_client_start(client);
		return error;
	}
	else {

		ESP_LOGE(TAG,"MQTT client already running");
		return ESP_FAIL;
	}
}


