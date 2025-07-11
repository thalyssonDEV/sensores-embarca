#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// --- Definições do Sensor MPU-6050 ---
// Endereço I2C padrão do sensor
const uint8_t SENSOR_ADDR = 0x68;

// Endereços dos Registradores
const uint8_t REG_ACCEL_X_H = 0x3B;
const uint8_t REG_GYRO_X_H  = 0x43;
const uint8_t REG_PWR_MGMT_1 = 0x6B;

// --- Pinos I2C utilizados ---
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

// Estrutura para guardar os valores lidos do sensor
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;


/**
 * @brief Lê todos os 6 eixos (acelerômetro e giroscópio) do sensor.
 * @param data Ponteiro para a estrutura onde os dados serão armazenados.
 */
void mpu6050_read_raw(mpu6050_data_t *data) {
    uint8_t buffer[14];

    // Começa a leitura a partir do registrador do acelerômetro X
    uint8_t start_reg = REG_ACCEL_X_H;
    i2c_write_blocking(i2c0, SENSOR_ADDR, &start_reg, 1, true);
    i2c_read_blocking(i2c0, SENSOR_ADDR, buffer, 14, false);

    // Combina os bytes altos e baixos para cada eixo
    // Os valores são de 16 bits com sinal (int16_t)
    data->accel_x = (buffer[0] << 8) | buffer[1];
    data->accel_y = (buffer[2] << 8) | buffer[3];
    data->accel_z = (buffer[4] << 8) | buffer[5];
    // Os bytes 6 e 7 são do sensor de temperatura, que ignoramos aqui
    data->gyro_x = (buffer[8] << 8) | buffer[9];
    data->gyro_y = (buffer[10] << 8) | buffer[11];
    data->gyro_z = (buffer[12] << 8) | buffer[13];
}


int main() {
    // Inicializa a comunicação USB para podermos usar o printf
    stdio_init_all();
    // Pausa para dar tempo de abrir o monitor série
    sleep_ms(4000); // Aumentado um pouco para garantir a conexão

    // --- Inicialização do I2C ---
    i2c_init(i2c0, 400 * 1000); // O MPU-6050 suporta uma velocidade maior (400kHz)
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // --- Inicialização do Sensor MPU-6050 ---
    // Acorda o sensor, pois ele começa em modo de suspensão (sleep mode)
    uint8_t wakeup_cmd[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(i2c0, SENSOR_ADDR, wakeup_cmd, 2, false);

    // Estrutura para guardar os dados
    mpu6050_data_t sensor_data;

    // --- ALTERAÇÃO 1: Imprime um cabeçalho fixo apenas uma vez ---
    // Limpa a tela do monitor serial e posiciona o cursor no início
    printf("\033[2J\033[H"); 
    printf("--- Monitor MPU-6050 ---\n");
    printf("I2C nos pinos SDA=%d, SCL=%d\n\n", I2C_SDA_PIN, I2C_SCL_PIN);
    printf("----------------------------------------\n");
    printf("EIXO\t\tX\t\tY\t\tZ\n");
    printf("----------------------------------------\n");

    // Loop infinito para ler e imprimir os dados
    while (1) {
        // Lê os dados brutos do sensor
        mpu6050_read_raw(&sensor_data);

        // --- ALTERAÇÃO 2: Saída de dados formatada e organizada ---
        // Posiciona o cursor em uma linha específica para sobrescrever os dados
        printf("\033[7;1H"); // Move o cursor para a linha 7, coluna 1
        
        // Imprime os valores com tabulação (\t) para alinhar em colunas
        printf("Acelerômetro:\t%-6d\t\t%-6d\t\t%-6d\n", sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        printf("Giroscópio:\t%-6d\t\t%-6d\t\t%-6d\n", sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        
        // --- ALTERAÇÃO 3: Delay de leitura maior ---
        // Espera 1 segundo (1000 ms) antes da próxima leitura
        sleep_ms(1000);
    }

    return 0;
}