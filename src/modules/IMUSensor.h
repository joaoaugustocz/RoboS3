#pragma once

#include <MPU6050.h>
#include <Wire.h>
#include "../types/RoboTypes.h"

namespace robos3
{

/** @brief Offsets crus de acelerometro e giroscopio da MPU6050. */
struct IMUOffsets
{
    int16_t ax; ///< Offset do acelerometro no eixo X.
    int16_t ay; ///< Offset do acelerometro no eixo Y.
    int16_t az; ///< Offset do acelerometro no eixo Z.
    int16_t gx; ///< Offset do giroscopio no eixo X.
    int16_t gy; ///< Offset do giroscopio no eixo Y.
    int16_t gz; ///< Offset do giroscopio no eixo Z.
};

/** @brief Driver interno da MPU6050 e estimador de orientacao relativa. */
class IMUSensor
{
  public:
    /** @brief Cria o driver com offsets e angulos zerados. */
    IMUSensor();

    /**
     * @brief Detecta e inicializa a MPU6050.
     * @param wire Barramento I2C ja configurado.
     * @param address Endereco I2C, normalmente 0x68.
     * @return true quando a IMU respondeu.
     */
    bool begin(TwoWire& wire, uint8_t address = 0x68);

    /** @return Nova leitura de aceleracao, giro, temperatura e orientacao. */
    LeituraIMU read();

    /** @brief Atualiza a estimativa interna e descarta o valor retornado. */
    void update()
    {
        read();
    }
    /** @brief Define a orientacao Z atual como zero relativo. */
    void zeroYaw();

    /** @brief Calcula offsets com a IMU imovel. @param durationMs Duracao em milissegundos. */
    void calibrate(uint32_t durationMs = 3000);

    /** @brief Aplica offsets crus. @param offsets Valores previamente obtidos. */
    void setOffsets(const IMUOffsets& offsets)
    {
        offsets_ = offsets;
    }
    /** @return Referencia constante para os offsets atuais. */
    const IMUOffsets& offsets() const
    {
        return offsets_;
    }
    /** @return Ultima leitura sem acessar novamente a IMU. */
    const LeituraIMU& last() const
    {
        return last_;
    }
    /** @return true quando a MPU6050 respondeu ao begin(). */
    bool available() const
    {
        return available_;
    }

  private:
    alignas(MPU6050) uint8_t storage_[sizeof(MPU6050)];
    MPU6050* mpu_;
    IMUOffsets offsets_;
    LeituraIMU last_;
    uint32_t lastMicros_;
    float yawZero_;
    float yawIntegrated_;
    bool available_;
};

} // namespace robos3
