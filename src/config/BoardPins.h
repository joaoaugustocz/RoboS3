#pragma once

#include <stdint.h>

namespace robos3
{
namespace board
{

/** @name Sensores de linha QTR
 *  Pinagem revisada para a placa. S1 e o sensor esquerdo e o bit 7.
 *  @{ */
constexpr uint8_t QTR_PINS[8] = {1, 2, 3, 5, 6, 7, 8, 9};
constexpr uint8_t QTR_EMITTER_PIN = 0xFF; ///< Nao existe conexao CTRL visivel.
/** @} */

/** @name Contrato ADC compartilhado pelo QTR-8A e botoes analogicos
 *  A RoboS3 le o QTR-8A diretamente para preservar os 12 bits do ESP32-S3,
 *  sem depender do limite interno de 10 bits da QTRSensors 4.0.0.
 *  @{ */
constexpr uint8_t ADC_RESOLUTION_BITS = 12;
constexpr uint16_t ADC_MAX_VALUE = (1U << ADC_RESOLUTION_BITS) - 1U;
/** @} */

/** @name Sensores ultrassonicos ULTRA1 e ULTRA2 @{ */
constexpr uint8_t ULTRASONIC_1_TRIG = 17;
constexpr uint8_t ULTRASONIC_1_ECHO = 18;
constexpr uint8_t ULTRASONIC_2_TRIG = 16;
constexpr uint8_t ULTRASONIC_2_ECHO = 42;
/** Alias dos nomes usados ate a versao 0.3.0; apontam para ULTRA1. */
constexpr uint8_t ULTRASONIC_TRIG = ULTRASONIC_1_TRIG;
constexpr uint8_t ULTRASONIC_ECHO = ULTRASONIC_1_ECHO;
/** @} */

/** @name Barramentos I2C independentes @{ */
constexpr uint8_t I2C0_SDA = 47;
constexpr uint8_t I2C0_SCL = 48;
constexpr uint8_t I2C1_SDA = 39;
constexpr uint8_t I2C1_SCL = 40;
/** @} */

/** @name Ponte H na ordem eletrica ENA, IN1, IN2, IN3, IN4, ENB
 *  Pinagem revisada para a placa.
 *  @{ */
constexpr uint8_t MOTOR_ENA = 14;
constexpr uint8_t MOTOR_IN1 = 15;
constexpr uint8_t MOTOR_IN2 = 11;
constexpr uint8_t MOTOR_IN3 = 12;
constexpr uint8_t MOTOR_IN4 = 13;
constexpr uint8_t MOTOR_ENB = 21;
/** @} */

/** @brief Endereco I2C padrao da MPU6050. */
constexpr uint8_t MPU6050_ADDRESS = 0x68;
/** @brief Interrupcao indisponivel porque nao ha trilha INT comprovada. */
constexpr int8_t MPU6050_INTERRUPT = -1;

/** @name Recursos nao identificados na PCB fornecida; -1 significa ausente. @{ */
constexpr int8_t LED = -1;
constexpr int8_t BUTTON = -1;
constexpr int8_t BUZZER = -1;
/** @} */

} // namespace board
} // namespace robos3
