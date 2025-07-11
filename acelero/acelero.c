#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

const uint8_t SENSOR_ADDR = 0x68;

const uint8_t REG_ACCEL_X_H = 0x3B;
const uint8_t REG_GYRO_X_H  = 0x43;
const uint8_t REG_PWR_MGMT_1 = 0x6B;

#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

void mpu6050_read_raw(mpu6050_data_t *data) {
    uint8_t buffer[14];

    uint8_t start_reg = REG_ACCEL_X_H;
    i2c_write_blocking(i2c0, SENSOR_ADDR, &start_reg, 1, true);
    i2c_read_blocking(i2c0, SENSOR_ADDR, buffer, 14, false);

    data->accel_x = (buffer[0] << 8) | buffer[1];
    data->accel_y = (buffer[2] << 8) | buffer[3];
    data->accel_z = (buffer[4] << 8) | buffer[5];
    data->gyro_x = (buffer[8] << 8) | buffer[9];
    data->gyro_y = (buffer[10] << 8) | buffer[11];
    data->gyro_z = (buffer[12] << 8) | buffer[13];
}

int main() {
    stdio_init_all();
    sleep_ms(4000);

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uint8_t wakeup_cmd[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(i2c0, SENSOR_ADDR, wakeup_cmd, 2, false);

    mpu6050_data_t sensor_data;

    printf("\033[2J\033[H");
    printf("--- Monitor MPU-6050 ---\n");
    printf("I2C nos pinos SDA=%d, SCL=%d\n\n", I2C_SDA_PIN, I2C_SCL_PIN);
    printf("----------------------------------------\n");
    printf("EIXO\t\tX\t\tY\t\tZ\n");
    printf("----------------------------------------\n");

    while (1) {
        mpu6050_read_raw(&sensor_data);

        printf("\033[7;1H");

        printf("Acelerômetro:\t%-6d\t\t%-6d\t\t%-6d\n", sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        printf("Giroscópio:\t%-6d\t\t%-6d\t\t%-6d\n", sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);

        sleep_ms(1000);
    }

    return 0;
}
