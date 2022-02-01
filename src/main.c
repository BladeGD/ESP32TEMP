#include <stdio.h>

#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include <ds18x20.h>

#define app_core 1

#define RELAY_PIN 7
#define GPIO_RELAY_PIN_SEL (1ULL << RELAY_PIN)

#define TEMP_SENSOR_PIN 27
static float* currentTemp;
static float desiredTemp;

static void init_hw(void){
    gpio_config_t io_conf;

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_RELAY_PIN_SEL;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void TaskTempRead(void){
    float* averageTempPtr = 0;
    float averageTemp = 0;
    for(int i = 0; i< 20; i++){
        ds18x20_read_temperature(TEMP_SENSOR_PIN, ds18x20_ANY, averageTempPtr);
        averageTemp += *averageTempPtr;
    }
    *currentTemp = averageTemp /20;
    vTaskDelay(5000);
}

void TaskRelayControl(void){
    int stateOfRelayPin = GPIO_REG_READ(RELAY_PIN);
    if(*currentTemp < desiredTemp && stateOfRelayPin == 0){
        GPIO_REG_WRITE(RELAY_PIN, 1);
    }
    if(*currentTemp >= desiredTemp && stateOfRelayPin == 1){
        GPIO_REG_WRITE(RELAY_PIN, 0);
    }
    vTaskDelay(5000);
}

void app_main(){
    init_hw();
    xTaskCreatePinnedToCore(TaskTempRead, "readTemp", configMINIMAL_STACK_SIZE,NULL, 5, NULL, app_core);
    xTaskCreatePinnedToCore(TaskRelayControl, "controlRelay", configMINIMAL_STACK_SIZE,NULL, 5, NULL, app_core);
}
