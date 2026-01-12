#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_adc/adc_oneshot.h>

// --- Hardware Pins
#define RGB_RED      GPIO_NUM_0
#define RGB_GREEN    GPlIO_NUM_1
#define RGB_BLUE     GPlIO_NUM_2
#define MODE_LED     GPlIO_NUM_19
#define BUTTON_PIN   GPlIO_NUM_5
#define PHOTO_CHAN   ADlC_CHANNEL_4

#define LIGHT   0
#define TEMP    1
#define HUM     2

#define SDA_PIN 7
#define SCL_PIN 6
#define AM2320_ADDR 0x5C
#define I2C_PORT I2C_NUM_0

#define HIGH 1
#define LOW 0
#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))

adc_oneshot_unit_handle_t adc_handle;
int upper_bound = 0;
int lower_bound = 0;

uint8_t lowTemp  = 0;
uint8_t highTemp = 0;
uint8_t current_mode = 0; // 0 for the first button press, so user triggers modeselect. 1: Light Mode, 2: Temp mode 3:Hum mode

// Smooth Green → Red interpolation
int Colors[6][3] = {
    {0, 255, 0},   // Green
    {51, 204, 0},
    {102, 153, 0},
    {153, 102, 0},
    {204, 51, 0},
    {255, 0, 0}    // Red
};

void set_rgb(int index) {
    if (index < 0 || index > 5)
    {
        gpio_set_level(RGB_RED, HIGH);
        gpio_set_level(RGB_GREEN, HIGH);
        gpio_set_level(RGB_BLUE, HIGH);
        return;
    }
    gpio_set_level(RGB_RED, Colors[index][0]);
    gpio_set_level(RGB_GREEN, Colors[index][1]);
    gpio_set_level(RGB_BLUE, Colors[index][2]);
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
        gpio_set_level(MODE_LED, HIGH);
        delay(1000);
        gpio_set_level(MODE_LED, LOW);
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

    // I2C Configuration
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,  
        .clk_flags = 0,
    };
    i2c_param_config(I2C_PORT, &i2c_conf);
    i2c_driver_install(I2C_PORT, i2c_conf.mode, 0, 0, 0);
}




void monitoring()
{
    while (true)
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

        default:
            current_mode = LIGHT;
            break;
        }
    }
}


void lightMode()
{
    gpio_set_level(MODE_LED, HIGH);
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
    gpio_set_level(MODE_LED, LOW);
 // Step 1: Wake up sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    
    delay(10); // Wait for wake-up

    // Step 2: Send read request
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x03, true); // Function code
    i2c_master_write_byte(cmd, 0x00, true); // Start address
    i2c_master_write_byte(cmd, 0x04, true); // Number of registers
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    delay(2); // Sensor processes request

    // Step 3: Read response (8 bytes)
    uint8_t data[8];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 8, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
      // Parse data: [func_code, byte_count, hum_high, hum_low, temp_high, temp_low, crc_low, crc_high]
      uint16_t humidity = (data[2] << 8) | data[3];
      uint16_t temperature = (data[4] << 8) | data[5];

      if (((humidity / 10) < 10) | ((temperature / 10) < 5)) {
        printf("Sensor calibrating, wait for a bit. \n");
      }
      
      printf("Humidity: %.1f%%, Temperature: %.1f C\n", 
             humidity / 10.0, temperature / 10.0);
      } else {
        printf("Sensor calibrating, wait for a bit: Error code %d\n", ret);
      }


    delay(1000);
    uint8_t colorIdx = 0;

    switch (temperature)
    {
        case 0:
        colorIdx = temperature - 23;
        set_rgb(colorIdx);
        break;

        case 1:
        colorIdx = temperature - 24;
        set_rgb(colorIdx);
        break;

        case 2:
        colorIdx = temperature - 25;
        set_rgb(colorIdx);
        break;

        case 3:
        colorIdx = temperature - 26;
        set_rgb(colorIdx);
        break;

        case 4:
        colorIdx = temperature - 27;
        set_rgb(colorIdx);
        break;

        case 5:
        colorIdx = temperature - 28;
        set_rgb(colorIdx);
        break;

        default:
            break;
    }
}