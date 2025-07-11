#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

const uint8_t SENSOR_ADDR = 0x38;
const uint8_t AHT10_CMD_CALIBRATE = 0xE1;
const uint8_t AHT10_CMD_TRIGGER   = 0xAC;
const uint8_t AHT10_CMD_SOFTRESET = 0xBA;

#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

void aht10_trigger_measurement() {
    uint8_t trigger_cmd[] = {AHT10_CMD_TRIGGER, 0x33, 0x00};
    i2c_write_blocking(i2c0, SENSOR_ADDR, trigger_cmd, 3, false);
}

int main() {
    stdio_init_all();
    sleep_ms(4000);

    printf("--- Leitor de Temperatura e Humidade AHT10 Iniciado ---\n");

    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C configurado nos pinos SDA=%d, SCL=%d.\n", I2C_SDA_PIN, I2C_SCL_PIN);

    uint8_t reset_cmd = AHT10_CMD_SOFTRESET;
    i2c_write_blocking(i2c0, SENSOR_ADDR, &reset_cmd, 1, false);
    sleep_ms(20);

    uint8_t calibrate_cmd[] = {AHT10_CMD_CALIBRATE, 0x08, 0x00};
    i2c_write_blocking(i2c0, SENSOR_ADDR, calibrate_cmd, 3, false);
    sleep_ms(400);

    printf("Sensor AHT10 inicializado e pronto para leitura.\n\n");

    float temperature, humidity;

    while (1) {
        aht10_trigger_measurement();

        sleep_ms(80);

        uint8_t buffer[6];
        int bytes_read = i2c_read_blocking(i2c0, SENSOR_ADDR, buffer, 6, false);

        if (bytes_read < 0) {
            printf("ERRO: Falha ao ler do sensor. A verificar conexao.\n");
            sleep_ms(2000);
            continue;
        }

        uint32_t raw_humidity = ((uint32_t)buffer[1] << 12) | ((uint32_t)buffer[2] << 4) | (buffer[3] >> 4);
        uint32_t raw_temp = (((uint32_t)buffer[3] & 0x0F) << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];

        humidity = (float)raw_humidity * 100.0f / 1048576.0f;
        temperature = (float)raw_temp * 200.0f / 1048576.0f - 50.0f;

        printf("Temperatura: %.2f C, Humidade: %.2f %%\n", temperature, humidity);

        sleep_ms(2000);
    }
    return 0;
}
