/**
 * @file main.c
 * @brief Leitor de Temperatura e Humidade com sensor AHT10.
 * * Este código combina a inicialização, configuração e leitura do sensor
 * AHT10 em um único ficheiro C para o Raspberry Pi Pico.
 * * Conexão:
 * - Sensor AHT10 conectado na porta I2C 0 da placa.
 * * O que o código faz:
 * - Configura a comunicação I2C nos pinos GPIO 0 e 1.
 * - Inicializa o sensor AHT10 com uma sequência robusta (Soft Reset).
 * - Em um loop infinito, lê os valores de temperatura e humidade e os imprime
 * no Monitor Série.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// --- Definições do Sensor AHT10 ---
// Endereço I2C padrão do sensor
const uint8_t SENSOR_ADDR = 0x38; 
// Comandos do sensor
const uint8_t AHT10_CMD_CALIBRATE = 0xE1;
const uint8_t AHT10_CMD_TRIGGER   = 0xAC;
const uint8_t AHT10_CMD_SOFTRESET = 0xBA;

// --- Pinos I2C utilizados (mesmo padrão dos outros projetos) ---
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1


/**
 * @brief Envia o comando para iniciar uma medição de temperatura e humidade.
 */
void aht10_trigger_measurement() {
    uint8_t trigger_cmd[] = {AHT10_CMD_TRIGGER, 0x33, 0x00};
    i2c_write_blocking(i2c0, SENSOR_ADDR, trigger_cmd, 3, false);
}


int main() {
    // Inicializa a comunicação USB para podermos usar o printf
    stdio_init_all();
    // Pausa para dar tempo de abrir o monitor série
    sleep_ms(4000);

    printf("--- Leitor de Temperatura e Humidade AHT10 Iniciado ---\n");

    // --- Inicialização do I2C ---
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C configurado nos pinos SDA=%d, SCL=%d.\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // --- Inicialização Robusta do Sensor AHT10 ---
    // 1. Envia um comando de Soft Reset para garantir um estado limpo
    uint8_t reset_cmd = AHT10_CMD_SOFTRESET;
    i2c_write_blocking(i2c0, SENSOR_ADDR, &reset_cmd, 1, false);
    sleep_ms(20); // Datasheet recomenda esperar 20ms após o reset

    // 2. Envia comando de calibração
    uint8_t calibrate_cmd[] = {AHT10_CMD_CALIBRATE, 0x08, 0x00};
    i2c_write_blocking(i2c0, SENSOR_ADDR, calibrate_cmd, 3, false);
    sleep_ms(400); // Aguarda a calibração

    printf("Sensor AHT10 inicializado e pronto para leitura.\n\n");

    // Variáveis para os dados lidos
    float temperature, humidity;

    // Loop infinito para ler e imprimir os dados
    while (1) {
        // 1. Pede ao sensor para fazer uma nova medição
        aht10_trigger_measurement();
        
        // 2. Aguarda o tempo necessário para a medição (datasheet diz ~75ms)
        sleep_ms(80);

        // 3. Lê os 6 bytes de dados do sensor
        uint8_t buffer[6];
        int bytes_read = i2c_read_blocking(i2c0, SENSOR_ADDR, buffer, 6, false);

        // Se a leitura falhar (retorna valor negativo), avisa e tenta de novo.
        if (bytes_read < 0) {
            printf("ERRO: Falha ao ler do sensor. A verificar conexao.\n");
            sleep_ms(2000);
            continue;
        }

        // 4. Converte os dados brutos em valores de Humidade e Temperatura
        uint32_t raw_humidity = ((uint32_t)buffer[1] << 12) | ((uint32_t)buffer[2] << 4) | (buffer[3] >> 4);
        uint32_t raw_temp = (((uint32_t)buffer[3] & 0x0F) << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];

        // Aplica as fórmulas de conversão do datasheet
        humidity = (float)raw_humidity * 100.0f / 1048576.0f; // 1048576 é 2^20
        temperature = (float)raw_temp * 200.0f / 1048576.0f - 50.0f;

        // Imprime os valores formatados no monitor série
        printf("Temperatura: %.2f C, Humidade: %.2f %%\n", temperature, humidity);
        
        // Espera 2 segundos antes da próxima leitura
        sleep_ms(2000);
    }

    return 0;
}