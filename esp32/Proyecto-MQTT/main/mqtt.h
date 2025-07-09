#ifndef __MQTT_H__
#define __MQTT_H__

//*****************************************************************************
//      DEFINICIONES
//*****************************************************************************

#define MQTT_BROKER_URL      CONFIG_EXAMPLE_MQTT_BROKER_URI
#define MQTT_TOPIC_SUBSCRIBE_BASE      CONFIG_EXAMPLE_MQTT_TOPIC_SUBSCRIBE_BASE
#define MQTT_TOPIC_PUBLISH_BASE  CONFIG_EXAMPLE_MQTT_TOPIC_PUBLISH_BASE

//****************************************************************************
//      TIPOS DE DATOS
//****************************************************************************

typedef enum{
    PING,
    BUTTON_STATUS,
    ADC_READ,
    LAST_WILL_MESSAGE,
	TEMPERATURE_READ,
	BLUETOOTH_DEVICE
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

//*****************************************************************************
//      VARIABLES COMPARTIDAS EXTERN
//*****************************************************************************

extern QueueHandle_t sendQueueHandler;

//*****************************************************************************
//      PROTOTIPOS DE FUNCIONES
//*****************************************************************************

esp_err_t mqtt_app_start(const char* url);
void IRAM_ATTR gpio_isr_handler(void* arg);


#endif //  __MQTT_H__
