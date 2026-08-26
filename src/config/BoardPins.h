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

/** @name Sensor ultrassonico ULTRA1
 *  @warning Esta atribuicao provisoria usa GPIO9, tambem usado por QTR S8.
 *  Atualize antes de usar QTR e ultrassom simultaneamente.
 *  @{ */
constexpr uint8_t ULTRASONIC_TRIG = 9;
constexpr uint8_t ULTRASONIC_ECHO = 46;
/** @} */

/** @name Barramentos I2C independentes
 *  @warning I2C0 SDA provisoriamente usa GPIO1, tambem usado por QTR S1.
 *  Atualize I2C0 antes de usar os dois modulos simultaneamente.
 *  @{ */
constexpr uint8_t I2C0_SDA = 1;
constexpr uint8_t I2C0_SCL = 42;
constexpr uint8_t I2C1_SDA = 41;
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
