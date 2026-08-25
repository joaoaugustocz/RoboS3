#pragma once

#include <stdint.h>

namespace robos3
{
namespace board
{

// Pinagem obtida das trilhas/rotulos da PCB ESP_FB V1 fornecida.
// S1 e o sensor fisico mais a esquerda e corresponde ao bit 7.
constexpr uint8_t QTR_PINS[8] = {4, 5, 6, 7, 15, 16, 17, 18};
constexpr uint8_t QTR_EMITTER_PIN = 0xFF; // nao existe conexao CTRL visivel

// Conector ULTRA1: as redes TRIG1/ECHO1 chegam a estes GPIOs.
constexpr uint8_t ULTRASONIC_TRIG = 9;
constexpr uint8_t ULTRASONIC_ECHO = 46;

// Barramentos independentes mostrados como I2C0 e I2C1.
constexpr uint8_t I2C0_SDA = 1;
constexpr uint8_t I2C0_SCL = 42;
constexpr uint8_t I2C1_SDA = 41;
constexpr uint8_t I2C1_SCL = 40;

// PONTE-H: ordem eletrica ENA, IN1, IN2, IN3, IN4, ENB.
constexpr uint8_t MOTOR_ENA = 0;
constexpr uint8_t MOTOR_IN1 = 36;
constexpr uint8_t MOTOR_IN2 = 39;
constexpr uint8_t MOTOR_IN3 = 38;
constexpr uint8_t MOTOR_IN4 = 37;
constexpr uint8_t MOTOR_ENB = 35;

constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr int8_t MPU6050_INTERRUPT = -1; // nao ha trilha INT comprovada

// Nao foram identificados buzzer, botoes ou LED controlavel nas fontes fornecidas.
constexpr int8_t LED = -1;
constexpr int8_t BUTTON = -1;
constexpr int8_t BUZZER = -1;

} // namespace board
} // namespace robos3
