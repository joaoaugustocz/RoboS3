#pragma once

#include <stdint.h>
#include "../types/RoboTypes.h"
#include "IMUSensor.h"

namespace robos3
{

/** @brief Assinatura que identifica um bloco NVS pertencente a RoboS3. */
constexpr uint32_t CALIBRATION_MAGIC = 0x524F424F;
/** @brief Versao do formato persistente RoboCalibration. */
constexpr uint16_t CALIBRATION_SCHEMA = 1;

/**
 * @brief Bloco persistente com todas as calibracoes do robo.
 * @note O checksum nao participa do proprio calculo de checksum.
 */
struct RoboCalibration
{
    uint32_t magic;                         ///< Assinatura CALIBRATION_MAGIC.
    uint16_t schema;                        ///< Versao CALIBRATION_SCHEMA.
    uint16_t size;                          ///< Tamanho do bloco em bytes.
    uint16_t lineMinimum[8];                ///< Minimos QTR na ordem S1...S8.
    uint16_t lineMaximum[8];                ///< Maximos QTR na ordem S1...S8.
    uint16_t lineThreshold;                 ///< Limiar calibrado de 0 a 1000.
    CalibracaoMotor leftMotor;              ///< Ajustes do motor esquerdo.
    CalibracaoMotor rightMotor;             ///< Ajustes do motor direito.
    FaixaHSV colorRanges[2][COR_TOTAL];      ///< Faixas por sensor e cor.
    uint8_t colorGain[2];                   ///< Ganho de cada TCS34725.
    uint8_t colorIntegration[2];            ///< Integracao de cada TCS34725.
    IMUOffsets imuOffsets;                  ///< Offsets da MPU6050.
    uint32_t checksum;                      ///< Verificacao de integridade.
};

/** @brief Persistencia das calibracoes na memoria NVS do ESP32. */
class CalibrationStorage
{
  public:
    /** @param data Bloco completo a persistir. @return true quando foi gravado. */
    bool save(const RoboCalibration& data);
    /** @param data Destino do bloco carregado. @return true quando ele e valido. */
    bool load(RoboCalibration& data);
    /** @return true quando o namespace persistente foi apagado. */
    bool erase();
    /** @param data Bloco a verificar. @return CRC simples do conteudo. */
    static uint32_t checksum(const RoboCalibration& data);
};

} // namespace robos3
