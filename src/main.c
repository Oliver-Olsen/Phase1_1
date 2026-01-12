#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/i2c.h>


// --- Hardware Pins
#define RGB_RED         GPIO_NUM_0
#define RGB_GREEN       GPIO_NUM_1
#define RGB_BLUE        GPIO_NUM_2
#define MODE_LED        GPIO_NUM_9
#define BUTTON_PIN      GPIO_NUM_5
#define PHOTO_CHAN      ADC_CHANNEL_3
#define SDA_PIN         GPIO_NUM_21
#define SCL_PIN         GPIO_NUM_20
#define I2C_PORT        I2C_NUM_0

#define AM2320_ADDR 0x5C

#define START_FLASH 3 // flashes 3 times

#define LIGHT   0
#define TEMP    1
#define HUM     2

#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))

adc_oneshot_unit_handle_t adc_handle;
int upper_bound = 0;
int lower_bound = 0;
uint8_t current_mode = 0; // 0 for the first button press, so user triggers modeselect. 1: Light Mode, 2: Temp mode 3:Hum mode

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
void i2c_config();
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


    i2c_config();



}

void i2c_config()
{
    // i2c config
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };

    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
}



void startup() {
    printf("STARTUP: Initializing Verification Sequence...\n");

    // 1. MODE_LED flash 3 times (1s on/off)
    for (int i = 0; i < START_FLASH; i++) {
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
    while(gpio_get_level(BUTTON_PIN) == false) delay(20);
    adc_oneshot_read(adc_handle, PHOTO_CHAN, &raw);
    upper_bound = raw;
    printf("Upper Bound Set: %d\n", upper_bound);
    while(gpio_get_level(BUTTON_PIN) == true) delay(20); // Wait for release

    printf("CALIBRATION: COVER sensor completely and press button.\n");
    while(gpio_get_level(BUTTON_PIN) == false) delay(20);
    adc_oneshot_read(adc_handle, PHOTO_CHAN, &raw);
    lower_bound = raw;
    printf("Lower Bound Set: %d\n", lower_bound);
    while(gpio_get_level(BUTTON_PIN) == true) delay(20);

    printf("STARTUP: Complete. Starting Monitoring...\n");
}



void monitoring()
{
    while (1)
    {
        // Step 1: Check for Mode Switch
        if (gpio_get_level(BUTTON_PIN) == true)
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

void am2320Data()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    uint8_t req[4] = {
        (AM2320_ADDR << 1) | I2C_MASTER_WRITE,
        0x03,
        0x00,
        0x04
    };

    i2c_master_write(cmd, req, sizeof(req), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_master_link_delete(cmd);

    uint8_t rx[2] = {0};
    i2c_master_read_byte(cmd, &rx[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &rx[1], I2C_MASTER_NACK);

}

void tempMode()
{

}


void humMode()
{

}