#pragma once

#include <stdint.h>
#include "BoardPins.h"
#include "../types/RoboTypes.h"

namespace robos3
{

/** @brief GPIOs da ponte H na ordem eletrica usada pela RoboS3. */
struct MotorPins
{
    uint8_t ena; ///< Enable/PWM do motor esquerdo.
    uint8_t in1; ///< Direcao 1 do motor esquerdo.
    uint8_t in2; ///< Direcao 2 do motor esquerdo.
    uint8_t in3; ///< Direcao 1 do motor direito.
    uint8_t in4; ///< Direcao 2 do motor direito.
    uint8_t enb; ///< Enable/PWM do motor direito.
};

/** @brief GPIOs e frequencia de um barramento I2C. */
struct I2CPins
{
    uint8_t sda;        ///< GPIO de dados.
    uint8_t scl;        ///< GPIO de clock.
    uint32_t frequency; ///< Frequencia em hertz.
};

/** @brief Configuracao eletrica completa da placa usada por RoboS3::begin(). */
struct BoardConfig
{
    uint8_t qtrPins[8];      ///< GPIOs QTR na ordem S1...S8.
    uint8_t qtrEmitterPin;   ///< Controle do emissor ou 0xFF quando ausente.
    TipoQTR qtrType;         ///< Tipo eletrico do conjunto QTR.
    MotorPins motors;        ///< Pinagem da ponte H.
    uint8_t ultrasonicTrig;  ///< GPIO TRIG do ultrassom.
    uint8_t ultrasonicEcho;  ///< GPIO ECHO do ultrassom.
    I2CPins i2c0;            ///< Barramento do TCS esquerdo e MPU6050.
    I2CPins i2c1;            ///< Barramento do TCS direito.
    uint8_t mpuAddress;      ///< Endereco I2C da MPU6050.
    int8_t mpuInterrupt;     ///< GPIO INT ou -1 quando ausente.
    uint32_t pwmFrequency;   ///< Frequencia PWM dos motores em hertz.
    uint8_t pwmResolution;   ///< Resolucao PWM em bits.

    /** @return Configuracao correspondente a PCB ESP_FB V1. */
    static BoardConfig padrao()
    {
        BoardConfig c{};
        for (uint8_t i = 0; i < 8; ++i)
        {
            c.qtrPins[i] = board::QTR_PINS[i];
        }
        c.qtrEmitterPin = board::QTR_EMITTER_PIN;
        c.qtrType = QTR_8A;
        c.motors = {board::MOTOR_ENA, board::MOTOR_IN1, board::MOTOR_IN2,
                    board::MOTOR_IN3, board::MOTOR_IN4, board::MOTOR_ENB};
        c.ultrasonicTrig = board::ULTRASONIC_TRIG;
        c.ultrasonicEcho = board::ULTRASONIC_ECHO;
        c.i2c0 = {board::I2C0_SDA, board::I2C0_SCL, 400000};
        c.i2c1 = {board::I2C1_SDA, board::I2C1_SCL, 400000};
        c.mpuAddress = board::MPU6050_ADDRESS;
        c.mpuInterrupt = board::MPU6050_INTERRUPT;
        c.pwmFrequency = 20000;
        c.pwmResolution = 10;
        return c;
    }
};

} // namespace robos3
