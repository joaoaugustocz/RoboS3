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

/** @brief Configuracao de um slot opcional para sensor VL53L0X. */
struct VL53L0XConfig
{
    uint8_t id;                 ///< Identificador logico escolhido pela equipe.
    bool habilitado;            ///< true quando o slot deve participar do begin().
    BarramentoI2C barramento;   ///< I2C ao qual o sensor esta conectado.
    uint16_t timeoutMs;         ///< Limite de espera de uma leitura bloqueante.
};

/** @brief Configuracao de um conector para sensor ultrassonico. */
struct UltrasonicConfig
{
    uint8_t id;        ///< Identificador logico escolhido pela equipe.
    bool habilitado;   ///< true quando o sensor deve participar do begin().
    uint8_t trigger;   ///< GPIO de disparo TRIG.
    uint8_t echo;      ///< GPIO de retorno ECHO.
    uint16_t maximoCm; ///< Maior distancia aguardada pelo driver.
};

/** @brief Configuracao eletrica completa da placa usada por RoboS3::begin(). */
struct BoardConfig
{
    uint8_t qtrPins[8];      ///< GPIOs QTR na ordem S1...S8.
    uint8_t qtrEmitterPin;   ///< Controle do emissor ou 0xFF quando ausente.
    TipoQTR qtrType;         ///< Tipo eletrico do conjunto QTR.
    MotorPins motors;        ///< Pinagem da ponte H.
    UltrasonicConfig ultrassom[2]; ///< Dois conectores HC-SR04 independentes.
    I2CPins i2c0;            ///< GPIOs e frequencia do controlador Wire.
    I2CPins i2c1;            ///< GPIOs e frequencia do controlador Wire1.
    BarramentoI2C corEsquerdaI2C; ///< Barramento atribuido ao TCS esquerdo.
    BarramentoI2C corDireitaI2C;  ///< Barramento atribuido ao TCS direito.
    bool corEsquerdaHabilitada;   ///< Inicializa o TCS esquerdo quando true.
    bool corDireitaHabilitada;    ///< Inicializa o TCS direito quando true.
    VL53L0XConfig vl53[2];        ///< Dois slots ToF opcionais e independentes.
    BarramentoI2C imuI2C;         ///< Barramento atribuido a MPU6050.
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
        c.ultrassom[0] = {DISTANCIA_ULTRASSOM_1, true, board::ULTRASONIC_1_TRIG,
                          board::ULTRASONIC_1_ECHO, 400};
        c.ultrassom[1] = {DISTANCIA_ULTRASSOM_2, true, board::ULTRASONIC_2_TRIG,
                          board::ULTRASONIC_2_ECHO, 400};
        c.i2c0 = {board::I2C0_SDA, board::I2C0_SCL, 400000};
        c.i2c1 = {board::I2C1_SDA, board::I2C1_SCL, 400000};
        c.corEsquerdaI2C = BARRAMENTO_I2C0;
        c.corDireitaI2C = BARRAMENTO_I2C1;
        c.corEsquerdaHabilitada = true;
        c.corDireitaHabilitada = true;
        c.vl53[0] = {DISTANCIA_VL53_1, false, BARRAMENTO_I2C0, 100};
        c.vl53[1] = {DISTANCIA_VL53_2, false, BARRAMENTO_I2C1, 100};
        c.imuI2C = BARRAMENTO_I2C0;
        c.mpuAddress = board::MPU6050_ADDRESS;
        c.mpuInterrupt = board::MPU6050_INTERRUPT;
        c.pwmFrequency = 20000;
        c.pwmResolution = 10;
        return c;
    }
};

} // namespace robos3
