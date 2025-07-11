#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

const uint8_t BH1750_ADDR = 0x23;
const uint8_t BH1750_POWER_ON = 0x01;
const uint8_t BH1750_CONT_HIGH_RES_MODE = 0x10;

#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

void bh1750_read_lux(i2c_inst_t *i2c, float *lux) {
    uint8_t buffer[2];

    i2c_read_blocking(i2c, BH1750_ADDR, buffer, 2, false);

    uint16_t raw = (buffer[0] << 8) | buffer[1];

    *lux = raw / 1.2f;
}

int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("--- Leitor de Luminosidade BH1750 Iniciado ---\n");

    i2c_init(i2c0, 100 * 1000);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("I2C configurado nos pinos SDA=%d, SCL=%d.\n", I2C_SDA_PIN, I2C_SCL_PIN);

    uint8_t power_on_cmd = BH1750_POWER_ON;
    i2c_write_blocking(i2c0, BH1750_ADDR, &power_on_cmd, 1, false);
    sleep_ms(10);

    uint8_t set_mode_cmd = BH1750_CONT_HIGH_RES_MODE;
    i2c_write_blocking(i2c0, BH1750_ADDR, &set_mode_cmd, 1, false);

    sleep_ms(180);

    printf("Sensor BH1750 pronto para leitura.\n\n");

    float lux_value;

    while (1) {
        bh1750_read_lux(i2c0, &lux_value);

        printf("Luminosidade: %.2f luz\n", lux_value);

        sleep_ms(1000);
    }

    return 0;
}
