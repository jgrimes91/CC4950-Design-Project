#include <stdio.h>
#include <string.h>
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "config.h" 
#include "tusb.h"

// I2C pins (adjust as per your wiring)
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

// MPU-6050 I2C address (0x68 by default)
#define MPU6050_ADDR 0x68

// MPU-6050 Registers
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A // Be careful with the name 'CONFIG', it may conflict with other definitions
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// I2C instance
#define I2C_PORT i2c0

void mpu6050_reset(){
    // Wake up the MPU-6050 as it starts in sleep mode
    uint8_t buf[] = {PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_init() {
    uint8_t buf[2];

    // Set sample rate to 1kHz
    buf[0] = SMPLRT_DIV;
    buf[1] = 0x07;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    // Set accelerometer configuration
    buf[0] = ACCEL_CONFIG;
    buf[1] = 0x00; // ±2g
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    // Set gyroscope configuration
    buf[0] = GYRO_CONFIG;
    buf[1] = 0x00; // ±250°/s
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

int16_t read_mpu6050_register(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true); // No stop bit
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    return (int16_t)(buf[0] << 8 | buf[1]);
}


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
uint8_t data_buffer[12];
int receive_length = 0;
u_int8_t receive_buffer[242];
u_int8_t receive_port = 0;


int main(void) {
    stdio_init_all(); 
    i2c_init(I2C_PORT, 100 * 1000);     // 100 kHz I2C Frequency

    // Set up I2C pins (SDA: GP0, SCL: GP1)
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Reset and initialize MPU6050
    mpu6050_reset();
    mpu6050_init();

    printf("MPU-6050 Initialization Complete\n");

    while (!tud_cdc_connected())
    {
        tight_loop_contents();
    }

    printf("Pico LoRaWAN - I2C - Gyro\n\n");

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

    while (1) {
        // Main loop
        // Read accelerometer data
        int16_t acc_x = read_mpu6050_register(ACCEL_XOUT_H);
        int16_t acc_y = read_mpu6050_register(ACCEL_XOUT_H + 2);
        int16_t acc_z = read_mpu6050_register(ACCEL_XOUT_H + 4);

        // Read gyroscope data
        int16_t gyro_x = read_mpu6050_register(GYRO_XOUT_H);
        int16_t gyro_y = read_mpu6050_register(GYRO_XOUT_H + 2);
        int16_t gyro_z = read_mpu6050_register(GYRO_XOUT_H + 4);

        // Convert to physical values
        float ax = acc_x / 16384.0f; // Accelerometer sensitivity factor for ±2g
        float ay = acc_y / 16384.0f;
        float az = acc_z / 16384.0f;

        float gx = gyro_x / 131.0f; // Gyroscope sensitivity factor for ±250°/s
        float gy = gyro_y / 131.0f;
        float gz = gyro_z / 131.0f;

        // Print the results
        printf("Accelerometer (g): Ax=%.2f, Ay=%.2f, Az=%.2f\n", ax, ay, az);
        printf("Gyroscope (°/s): Gx=%.2f, Gy=%.2f, Gz=%.2f\n", gx, gy, gz);
        printf("-----------------------------------\n");

        uint8_t ax_msb = (acc_x >> 8) & 0xFF;
        uint8_t ax_lsb = acc_x & 0xFF;
        uint8_t ay_msb = (acc_y >> 8) & 0xFF;
        uint8_t ay_lsb = acc_y & 0xFF;
        uint8_t az_msb = (acc_z >> 8) & 0xFF;
        uint8_t az_lsb = acc_z & 0xFF;

        u_int8_t gx_msb = (gyro_x >> 8) & 0xFF;
        u_int8_t gx_lsb = gyro_x & 0xFF;
        u_int8_t gy_msb = (gyro_y >> 8) & 0xFF;
        u_int8_t gy_lsb = gyro_y & 0xFF;
        u_int8_t gz_msb = (gyro_z >> 8) & 0xFF;
        u_int8_t gz_lsb = gyro_z & 0xFF;

        data_buffer[0] = ax_msb;
        data_buffer[1] = ax_lsb;
        data_buffer[2] = ay_msb;
        data_buffer[3] = ay_lsb; 
        data_buffer[4] = az_msb;
        data_buffer[5] = az_lsb;
        data_buffer[6] = gx_msb;
        data_buffer[7] = gx_lsb;
        data_buffer[8] = gy_msb;
        data_buffer[9] = gy_lsb;
        data_buffer[10] = gz_msb;
        data_buffer[11] = gz_lsb;

        // Send data buffer (3 bytes) in an unconfirmed uplink message
        printf("sending data: Accel_x = %d, Accel_y = %d, Accel_z = %d, Gyro_x = %d, Gyro_y, Gyro_z = %d ", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
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

        }

        lorawan_process_timeout_ms(15000);

    }
}
