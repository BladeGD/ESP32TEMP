#include <stdio.h>

#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define GPIO_DS18B20_0 16
#define DS18B20_RESOLUTION 12
#define SAMPLE_PERIOD 1000

#define app_core 1

#define RELAY_PIN 4
#define GPIO_RELAY_PIN_SEL (1ULL << RELAY_PIN)

static DS18B20_Info * ds18b20_info;
static TickType_t last_wake_time;
static OneWireBus * owb;
static owb_rmt_driver_info rmt_driver_info;
static OneWireBus_ROMCode rom_code;
static owb_status status;

static float currentTemp;
static float desiredTemp = 26;
static const char *TAG = "CTRL_APP";
static int relayPower;

typedef enum FSMstates{
        Idle_State,
        Measure_State,
        Temp_Low_State,
        Temp_High_State,
        Turn_On_Relay_State,
        Turn_Off_Relay_State
}FSMstates;

static void init_hw(void){
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = GPIO_RELAY_PIN_SEL;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(RELAY_PIN, 0);

    // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true); // enable CRC check for ROM code

    // For a single device only:
    status = owb_read_rom(owb, &rom_code);
    if (status == OWB_STATUS_OK){
        char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
        owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
        printf("Single device %s present\n", rom_code_s);
    }
    else{
        printf("An error occurred reading ROM code: %d", status);
    }

    ds18b20_info = ds18b20_malloc();
    printf("Single device optimisations enabled\n");
    ds18b20_init_solo(ds18b20_info, owb);
    ds18b20_use_crc(ds18b20_info, true); // enable CRC check on all reads
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);

    last_wake_time = xTaskGetTickCount();
}

static float TempRead(){

        static float readTemp;
        ds18b20_convert_all(owb);
        ds18b20_wait_for_conversion(ds18b20_info);

        ds18b20_read_temp(ds18b20_info, &readTemp);
        printf("Temperature readings (degrees C): ");

        printf("%.1f\n", readTemp);

        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        ESP_LOGI(TAG,"Ram Left %d" ,xPortGetFreeHeapSize());
        return readTemp;
}

static void FSMTempCtrl(void* pvParameters)
{
    enum FSMstates state = Idle_State;
    for(;;)
    {
        switch(state){
            case Idle_State:
                state = Measure_State;
            break;
            case Measure_State:
                currentTemp = TempRead();
                if(currentTemp < desiredTemp){
                    ESP_LOGI(TAG,"Going to Low Temp State");
                    state = Temp_Low_State;
                }
                if(currentTemp >= desiredTemp){
                    ESP_LOGI(TAG,"Going to High Temp State");
                    state = Temp_High_State;
                }
            break;
            case Temp_Low_State:
                ESP_LOGI(TAG,"At Low Temp State");
                relayPower = gpio_get_level(RELAY_PIN);
                if(relayPower == 1){
                    ESP_LOGI(TAG,"Going to Relay On State");
                    state = Turn_On_Relay_State;
                }else {
                    state = Measure_State;
                }
            break;
            case Temp_High_State:
                ESP_LOGI(TAG,"At High Temp State");
                relayPower = gpio_get_level(RELAY_PIN);
                if(relayPower ==  0){
                    ESP_LOGI(TAG,"Going to Relay Off State");
                    state = Turn_Off_Relay_State;
                }else {
                    state = Measure_State;
                }
            break;
            case Turn_On_Relay_State:
                ESP_LOGI(TAG,"At Relay On State");
                gpio_set_level(RELAY_PIN, 0);
                ESP_LOGI(TAG,"Relay State %i" ,gpio_get_level(RELAY_PIN));
                state = Measure_State;
            break;
            case Turn_Off_Relay_State:
                ESP_LOGI(TAG,"At Relay Off State");
                gpio_set_level(RELAY_PIN, 1);
                ESP_LOGI(TAG,"Relay State %i" ,gpio_get_level(RELAY_PIN));
                state = Measure_State;
            break;
        }
        vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }
}

void app_main(){
    init_hw();
    xTaskCreatePinnedToCore(&FSMTempCtrl, "FSM run.", 3*configMINIMAL_STACK_SIZE,NULL, 5, NULL, app_core);
    //xTaskCreatePinnedToCore(TaskRelayControl, "controlRelay", 3*configMINIMAL_STACK_SIZE,NULL, 4, NULL, app_core);
}
