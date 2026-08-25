#pragma once

#include <stdint.h>
#include "BoardPins.h"
#include "../types/RoboTypes.h"

namespace robos3
{

struct MotorPins
{
    uint8_t ena;
    uint8_t in1;
    uint8_t in2;
    uint8_t in3;
    uint8_t in4;
    uint8_t enb;
};

struct I2CPins
{
    uint8_t sda;
    uint8_t scl;
    uint32_t frequency;
};

struct BoardConfig
{
    uint8_t qtrPins[8];
    uint8_t qtrEmitterPin;
    TipoQTR qtrType;
    MotorPins motors;
    uint8_t ultrasonicTrig;
    uint8_t ultrasonicEcho;
    I2CPins i2c0;
    I2CPins i2c1;
    uint8_t mpuAddress;
    int8_t mpuInterrupt;
    uint32_t pwmFrequency;
    uint8_t pwmResolution;

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
