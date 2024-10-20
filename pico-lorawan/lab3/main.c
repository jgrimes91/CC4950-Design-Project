#include <stdio.h>
#include <string.h>
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"
#include "config.h"

// Define the GPIO pins for the RGB LED
#define RED_PIN 16
#define GREEN_PIN 17
#define BLUE_PIN 18

// LoRaWAN pin configuration (SX12xx radio module)
const struct lorawan_sx12xx_settings sx12xx_settings = {
    .spi = {
        .inst = spi1,
        .mosi = 11,
        .miso = 12,
        .sck = 10,
        .nss = 3},
    .reset = 15,
    .busy = 2,
    .dio1 = 20};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui = LORAWAN_DEVICE_EUI,
    .app_eui = LORAWAN_APP_EUI,
    .app_key = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK};

// Data buffer for sending temperature and LDR values (3 bytes)
uint8_t data_buffer[9];
int receive_length = 0;
u_int8_t receive_buffer[242];
u_int8_t receive_port = 0;

// Functions to initialize sensors and read values
void internal_temperature_init();
float internal_temperature_get();
void voltage_init();
uint16_t read_ldr();
void ldr_init();
float voltage_read();
void rbg_led_init();
void set_rgb_colour(uint8_t red, u_int8_t blue, u_int8_t green);

// Main function
int main(void)
{
    stdio_init_all();
    while (!tud_cdc_connected())
    {
        tight_loop_contents();
    }

    printf("Pico LoRaWAN - OTAA - Temp + LDR\n\n");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    uint16_t count = 0; 

    // Initialise adc
    internal_temperature_init();
    voltage_init();
    

    // Initialise RGB
    rbg_led_init();

    // Initialize LoRaWAN
    printf("Initializing LoRaWAN ... ");
    if (lorawan_init_otaa(&sx12xx_settings, LORAWAN_REGION, &otaa_settings) < 0)
    {
        printf("failed!\n");
        while (1)
        {
            tight_loop_contents();
        }
    }
    printf("success!\n");

    printf("Joining LoRaWAN network ...");
    lorawan_join();

    while (!lorawan_is_joined())
    {
        lorawan_process_timeout_ms(1000);
        printf(".");
    }
    printf(" joined successfully!\n");

    while (1)
    {
        // Read temperature (1 byte)
        int8_t adc_temperature_byte = internal_temperature_get();

        // Read LDR value (2 bytes)
        uint16_t ldr_value = read_ldr();
        uint8_t ldr_msb = (ldr_value >> 8) & 0xFF;
        uint8_t ldr_lsb = ldr_value & 0xFF;

        uint8_t count_msb = (count >> 8);
        uint8_t count_lsb = count & 0xFF;

        float voltage = voltage_read();

        // Populate data buffer
        data_buffer[0] = adc_temperature_byte; // 1 byte for temperature
        data_buffer[1] = ldr_msb;              // MSB of LDR
        data_buffer[2] = ldr_lsb;              // LSB of LDR
        data_buffer[3] = count_msb;            // count
        data_buffer[4] = count_lsb;

        memcpy(data_buffer + 5, &voltage, 4);

        // Send data buffer (3 bytes) in an unconfirmed uplink message
        printf("sending data: Temp=%d 'C, LDR=%d, Count= %d, Voltage=%f ... ", adc_temperature_byte, ldr_value, count, voltage);
        if (lorawan_send_unconfirmed(data_buffer, sizeof(data_buffer), 2) < 0)
        {
            printf("failed!\n");
        }
        else
        {
            printf("success!\n");
        }
        // Process downlink message and handle LED control 
        receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
        if (receive_length > -1) {
            printf("Received %d bytes on port %d\n", receive_length, receive_port);

            gpio_put(PICO_DEFAULT_LED_PIN, receive_buffer[0] != 0);

            uint8_t red_value = receive_buffer[1];
            uint8_t blue_value = receive_buffer[2];
            uint8_t green_value = receive_buffer[3];

            set_rgb_colour(red_value, blue_value, green_value);
        }


        count++;
        lorawan_process_timeout_ms(30000);
    }
    return 0;
}

// Initialize internal temperature sensor
void internal_temperature_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

// Get internal temperature (return as signed byte)
float internal_temperature_get()
{
    const float v_ref = 3.3;
    adc_select_input(4);
    uint16_t adc_raw = adc_read();
    float adc_voltage = adc_raw * v_ref / 4095.0f;
    float adc_temperature = 27.0 - ((adc_voltage - 0.706) / 0.001721);
    return (int8_t)adc_temperature;
}

// Read LDR value (ADC read from specific input)
uint16_t read_ldr()
{
    adc_select_input(2);
    return adc_read();
}

void voltage_init()
{
    adc_gpio_init(29);
}

float voltage_read()
{
    adc_select_input(3);
    float conversion_factor = 3.3 * 3.0 / (1 << 12);
    return adc_read() * conversion_factor;
}


void rbg_led_init(){
    gpio_set_function(RED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BLUE_PIN, GPIO_FUNC_PWM);
    gpio_set_function(GREEN_PIN, GPIO_FUNC_PWM);

    uint slice_red = pwm_gpio_to_slice_num(RED_PIN);
    uint slice_blue = pwm_gpio_to_slice_num(BLUE_PIN);
    uint slice_green = pwm_gpio_to_slice_num(GREEN_PIN);

    pwm_set_wrap(slice_red, 255);
    pwm_set_wrap(slice_blue, 255);
    pwm_set_wrap(slice_green, 255);

    pwm_set_enabled(slice_red, true);
    pwm_set_enabled(slice_blue, true);
    pwm_set_enabled(slice_green, true);
}

void set_rgb_colour(uint8_t red, u_int8_t blue, u_int8_t green){
    uint slice_red = pwm_gpio_to_slice_num(RED_PIN);
    uint slice_blue = pwm_gpio_to_slice_num(BLUE_PIN);
    uint slice_green = pwm_gpio_to_slice_num(GREEN_PIN);

    pwm_set_gpio_level(RED_PIN, red * red);
    pwm_set_gpio_level(BLUE_PIN, blue * blue);
    pwm_set_gpio_level(GREEN_PIN, green * green);

}