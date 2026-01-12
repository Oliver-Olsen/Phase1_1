#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

// --- Hardware Pins
#define RGB_RED      GPIO_NUM_0
#define RGB_GREEN    GPIO_NUM_1
#define RGB_BLUE     GPIO_NUM_2
#define MODE_LED     GPIO_NUM_9
#define BUTTON_PIN   GPIO_NUM_5
#define PHOTO_CHAN   ADC_CHANNEL_3

#define LIGHT   0
#define TEMP    1
#define HUM     2

#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))

adc_oneshot_unit_handle_t adc_handle;
int upper_bound = 0;
int lower_bound = 0;
int current_mode = 1; // 1: Light Mode, 2: Temp mode 3:Hum mode

// RGB Color Map:
int colors[6][3] = {
    {1,0,1},
    {0,0,1},
    {0,1,1},
    {0,1,0},
    {1,1,0},
    {1,0,0}
};

void set_rgb(int index) {
    if (index < 0 || index > 5)
    {
        gpio_set_level(RGB_RED, 0);
        gpio_set_level(RGB_GREEN, 0);
        gpio_set_level(RGB_BLUE, 0);
        return;
    }
    gpio_set_level(RGB_RED, colors[index][0]);
    gpio_set_level(RGB_GREEN, colors[index][1]);
    gpio_set_level(RGB_BLUE, colors[index][2]);
}




void initGPIO_ADC();
void startup();
void monitoring();
void lightMode();
void tempMode();
void humMode();



void app_main() {

    initGPIO_ADC();
    startup();
    monitoring();

}






void startup() {
    printf("STARTUP: Initializing Verification Sequence...\n");

    // 1. MODE_LED flash 3 times (1s on/off)
    for (int i = 0; i < 3; i++) {
        gpio_set_level(MODE_LED, 1);
        delay(1000);
        gpio_set_level(MODE_LED, 0);
        delay(1000);
    }

    // 2. RGB LED Cycle (lowest to highest intensity)
    for (int i = 0; i < 6; i++) {
        set_rgb(i);
        delay(1000);
    }
    set_rgb(-1); // Turn off after cycle

    // 3. Sensor Calibration
    int raw;
    printf("CALIBRATION: Keep sensor CLEAR and press button.\n");
    while(gpio_get_level(BUTTON_PIN) == 0) delay(20);
    adc_oneshot_read(adc_handle, PHOTO_CHAN, &raw);
    upper_bound = raw;
    printf("Upper Bound Set: %d\n", upper_bound);
    while(gpio_get_level(BUTTON_PIN) == 1) delay(20); // Wait for release

    printf("CALIBRATION: COVER sensor completely and press button.\n");
    while(gpio_get_level(BUTTON_PIN) == 0) delay(20);
    adc_oneshot_read(adc_handle, PHOTO_CHAN, &raw);
    lower_bound = raw;
    printf("Lower Bound Set: %d\n", lower_bound);
    while(gpio_get_level(BUTTON_PIN) == 1) delay(20);

    printf("STARTUP: Complete. Starting Monitoring...\n");
}




void initGPIO_ADC()
{
     // GPIO Configuration
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<MODE_LED) | (1ULL<<RGB_RED) | (1ULL<<RGB_GREEN) | (1ULL<<RGB_BLUE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0
    };
    gpio_config(&io_conf);

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL<<BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 1 // Circuit uses 10k pulldown
    };
    gpio_config(&btn_conf);

    // ADC Configuration
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&init_config1, &adc_handle);
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(adc_handle, PHOTO_CHAN, &config);
}




void monitoring()
{
    while (1)
    {
        // Step 1: Check for Mode Switch
        if (gpio_get_level(BUTTON_PIN) == 1)
        {
            current_mode++;
            printf("MODE SWITCH: Switched to Mode %d\n", current_mode);
            while(gpio_get_level(BUTTON_PIN) == 1) delay(20); // Wait for release
        }

        // Step 2 & 3: Measure and Display
        switch (current_mode)
        {
        case LIGHT:
            lightMode();
            break;

        case TEMP:
            tempMode();
            break;

        case HUM:
            humMode();
            break;

        default:
            current_mode = LIGHT;
            break;
        }
    }
}


void lightMode()
{
    int val;
    adc_oneshot_read(adc_handle, PHOTO_CHAN, &val);
    printf("Light Intensity: %d\n", val);

    // Map 6 colors based on calibration
    int colorIdx = (val - lower_bound) * 5 / (upper_bound - lower_bound);
    if (colorIdx < 0) {
        colorIdx = 0;
    }
    if (colorIdx > 5) {
        colorIdx = 5;
    }
    set_rgb(colorIdx);
    delay(1000);
}


void tempMode()
{

}

void humMode()
{

}