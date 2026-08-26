#pragma once

#include <stdint.h>
#include "../types/RoboTypes.h"

namespace robos3
{

/** @brief Sentido e duty PWM resultantes de um comando de motor. */
struct MotorOutput
{
    bool forward;  ///< true para frente, false para tras.
    uint16_t duty; ///< Duty absoluto de 0 ate pwmMax.
};

/** @param value Comando solicitado. @return value limitado a -100...100. */
int16_t clampCommand(int value);
/**
 * @param command Comando de motor de -100 a 100.
 * @param calibration Ajustes do motor e do sentido.
 * @param pwmMax Maior duty permitido.
 * @return Sentido e duty depois de aplicar os ajustes.
 */
MotorOutput motorOutput(int command, const CalibracaoMotor& calibration, uint16_t pwmMax);
/**
 * @param values Oito leituras calibradas na ordem S1...S8.
 * @param threshold Limiar de 0 a 1000. @param type Polaridade da pista.
 * @return Mascara com S1 no bit 7, S8 no bit 0 e preto igual a 1.
 */
uint8_t lineMask(const uint16_t values[8], uint16_t threshold, TipoLinha type);
/** @brief Converte uma mascara. @param mask Byte de entrada. @param output Buffer char[9]. */
void linePattern(uint8_t mask, char output[9]);
/** @param value Byte a contar. @return Quantidade de bits iguais a 1. */
uint8_t countBits(uint8_t value);
/**
 * @param now Instante atual. @param lastDetection Ultimo instante detectado.
 * @param detected Estado atual. @return Tempo sem deteccao ou zero.
 */
uint32_t elapsedWithoutDetection(uint32_t now, uint32_t lastDetection, bool detected);
/** @param position Posicao medida. @param center Alvo. @return position menos center. */
int16_t lineError(uint16_t position, int16_t center = 3500);
/** @param r Canal cru R. @param g Canal G. @param b Canal B. @param clear Canal clear. @return RGB em 0...255. */
RGB normalizeRgb(uint16_t r, uint16_t g, uint16_t b, uint16_t clear);
/** @param rgb Cor de entrada. @return HSV em graus e porcentagem. */
HSV rgbToHsv(const RGB& rgb);
/** @param rgb Cor de entrada. @return HSL em graus e porcentagem. */
HSL rgbToHsl(const RGB& rgb);
/** @param hue Matiz. @param minimum Limite inicial. @param maximum Limite final. @return true quando pertence ao intervalo circular. */
bool hueInRange(float hue, float minimum, float maximum);
/** @param hsv Cor medida. @param range Faixa configurada. @return true quando pertence a faixa ativa. */
bool hsvInRange(const HSV& hsv, const FaixaHSV& range);

} // namespace robos3
