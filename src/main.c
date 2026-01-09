#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>

#define RGB_RED 2
#define RGB_GREEN 1
#define RGB_BLUE 0

#define MODE_LED 9
#define BUTTON_PIN 5
#define photoresistor 3

#define HIGH 1
#define LOW 0

#define LIGHT_MODE  1
#define TEMP_MODE   2
#define HUM_MODE    3

#define FIRST   1
#define SECOND  2



int firstButtonPress = 0;
int secondButtonPress = 0;

uint8_t calibration = 0;

uint8_t button_pressed = 0;
uint8_t modeState = 0;

int colors[6][3] = {
    {1,0,1},
    {0,0,1},
    {0,1,1},
    {0,1,0},
    {1,1,0},
    {1,0,0}
};

int next_color = 0;


void modeShift();
void initSensors();
void startUp();
void minitoring();
bool getButtonState();
void measureLight();
void measureHum();
void measureTemp();


void app_main() {
    initSensors();
    startUp();
    while (true)
    {
        minitoring();
    }

}

void initSensors()
{
    // Output
    gpio_config_t io_conf_outs = {
        .pin_bit_mask = (1ULL << RGB_RED) + (1ULL << RGB_GREEN) + (1ULL << RGB_BLUE) + (1ULL << MODE_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_outs);

    // Input
    gpio_config_t io_conf_ins = {
        .pin_bit_mask = (1ULL << BUTTON_PIN) | (1ULL << photoresistor),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_ins);

   // ADC
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1, // ADC1
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_new_unit(&adc_config, &adc_handle);

    // Configure ADC channel 3 ()
    adc_oneshot_chan_cfg_t adc_channel_config = {
        .bitwidth = 12,
        .atten = ADC_ATTEN_DB_12,
    };
    // ADC unit 1 channel 3 samples pin 3
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &adc_channel_config);









  // Turn off LEDs
    gpio_set_level(MODE_LED, 0);
    gpio_set_level(RGB_RED, 0);
    gpio_set_level(RGB_GREEN, 0);
    gpio_set_level(RGB_BLUE, 0);

  // 1. Mode LED
    for (int i=0; i<3; i++) {
        gpio_set_level(MODE_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(MODE_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

  // 2. RGB LED colors
  for (int i=0; i<6; i++) {
    gpio_set_level(RGB_RED, !colors[next_color][0]);
    gpio_set_level(RGB_GREEN, !colors[next_color][1]);
    gpio_set_level(RGB_BLUE, !colors[next_color][2]);

    next_color += 1;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

}


void startUp()
{

  // 3. Start process
    int LDR_RAW;
    int pot_mapped = LDR_RAW << 1;
    printf("Keep sensor clear, and press button\n");


    button_pressed = gpio_get_level(BUTTON_PIN);
    printf("%d", button_pressed);
    if ((button_pressed == 1) && (firstButtonPress == 0)) {
        int lowerBoundLDR = adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &LDR_RAW);
        printf("Upper LDR bound: %d \n", pot_mapped);
        firstButtonPress = 1;
        vTaskDelay(pdMS_TO_TICKS(3000));
        printf("Cover sensor completely, and press button again \n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    if ((button_pressed == HIGH) && (secondButtonPress == 0)) {
        int upperBoundLDR = adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &LDR_RAW);
        printf("Release finger from LDR \n lower LDR bound: %d \n", pot_mapped);
        secondButtonPress = 1;
    }
    vTaskDelay(pdMS_TO_TICKS(100));


}

bool getButtonState()
{
    if (button_pressed == HIGH)
    {
        modeState++;
    }
    return true;
}



void minitoring()
{

}

void modeShift(uint8_t *modeState)
{
    switch (*modeState)
    {
    case LIGHT_MODE:
        measureLight();
        break;

    case TEMP_MODE:
        measureTemp();
        break;

    case HUM_MODE:
        measureHum();
        break;

    default:
    *modeState = 0;
        break;
    }
}

void measureLight()
{

}


void measureHum()
{

}

void measureTemp()
{

}

void displayResults(uint8_t modeShift)
{

}

void lightToLED()
{

}