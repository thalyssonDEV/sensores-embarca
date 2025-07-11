/**
 * @file main.c
 * @brief Leitor de Luminosidade com sensor BH1750 em um único arquivo.
 * * Este código combina a inicialização, configuração e leitura do sensor de
 * luminosidade BH1750 em um único arquivo C para o Raspberry Pi Pico.
 * * Conexão:
 * - Sensor BH1750 conectado na porta I2C 0 da placa (SDA no GPIO 0, SCL no GPIO 1).
 * * O que o código faz:
 * - Configura a comunicação I2C nos pinos GPIO 0 e 1.
 * - Inicializa o sensor BH1750 no modo de alta resolução contínua.
 * - Em um loop infinito, lê o valor de luminosidade e o imprime no Monitor Serial.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// --- Definições do Sensor BH1750 ---
// Endereço I2C do sensor
const uint8_t BH1750_ADDR = 0x23; 
// Comando para ligar o sensor
const uint8_t BH1750_POWER_ON = 0x01;
// Comando para o modo de leitura contínua de alta resolução
const uint8_t BH1750_CONT_HIGH_RES_MODE = 0x10; 

// --- Pinos I2C utilizados ---
// A sua placa BitDogLab usa os pinos 0 e 1 para a porta I2C 0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1


/**
 * @brief Lê o valor de luminosidade do sensor.
 * * @param i2c A instância I2C a ser usada (ex: i2c0).
 * @param lux Ponteiro para uma variável float onde o valor lido será armazenado.
 */
void bh1750_read_lux(i2c_inst_t *i2c, float *lux) {
    uint8_t buffer[2];

    // Lê 2 bytes (MSB e LSB) do sensor através do barramento I2C.
    i2c_read_blocking(i2c, BH1750_ADDR, buffer, 2, false);
    
    // Combina os dois bytes (MSB << 8 | LSB) em um único valor inteiro de 16 bits.
    uint16_t raw = (buffer[0] << 8) | buffer[1];
    
    // Converte o valor bruto para lux, conforme a fórmula do datasheet.
    // Fórmula para modo de alta resolução: Lux = (Valor Medido) / 1.2
    *lux = raw / 1.2f;
}


int main() {
    // Inicializa a comunicação USB para podermos usar o printf
    stdio_init_all();
    // Pausa para dar tempo de abrir o monitor serial
    sleep_ms(3000);

    printf("--- Leitor de Luminosidade BH1750 Iniciado ---\n");

    // --- Inicialização do I2C e do Sensor ---
    // Inicializa a instância i2c0 com velocidade de 100kHz
    i2c_init(i2c0, 100 * 1000);
    
    // Define os pinos GPIO para as funções I2C
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    
    // Ativa as resistências de pull-up internas (boa prática)
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("I2C configurado nos pinos SDA=%d, SCL=%d.\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // Envia o comando para ligar o sensor
    uint8_t power_on_cmd = BH1750_POWER_ON;
    i2c_write_blocking(i2c0, BH1750_ADDR, &power_on_cmd, 1, false);
    sleep_ms(10); // Pequeno delay para estabilização

    // Envia o comando para configurar o modo de leitura
    uint8_t set_mode_cmd = BH1750_CONT_HIGH_RES_MODE;
    i2c_write_blocking(i2c0, BH1750_ADDR, &set_mode_cmd, 1, false);

    // Aguarda o tempo da primeira conversão de luz
    sleep_ms(180);

    printf("Sensor BH1750 pronto para leitura.\n\n");

    // Variável para armazenar o valor lido
    float lux_value;

    // Loop infinito para ler e imprimir a luminosidade
    while (1) {
        // Chama a função para ler o valor do sensor
        bh1750_read_lux(i2c0, &lux_value);

        // Imprime o valor formatado no monitor serial
        printf("Luminosidade: %.2f luz\n", lux_value);
        
        // Espera 1 segundo antes da próxima leitura
        sleep_ms(1000);
    }

    return 0;
}