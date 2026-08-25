#pragma once

#include <stdint.h>
#include "../types/RoboTypes.h"

namespace robos3
{

struct MotorOutput
{
    bool forward;
    uint16_t duty;
};

int16_t clampCommand(int value);
MotorOutput motorOutput(int command, const CalibracaoMotor& calibration, uint16_t pwmMax);
uint8_t lineMask(const uint16_t values[8], uint16_t threshold, TipoLinha type);
void linePattern(uint8_t mask, char output[9]);
uint8_t countBits(uint8_t value);
uint32_t elapsedWithoutDetection(uint32_t now, uint32_t lastDetection, bool detected);
int16_t lineError(uint16_t position, int16_t center = 3500);
RGB normalizeRgb(uint16_t r, uint16_t g, uint16_t b, uint16_t clear);
HSV rgbToHsv(const RGB& rgb);
HSL rgbToHsl(const RGB& rgb);
bool hueInRange(float hue, float minimum, float maximum);
bool hsvInRange(const HSV& hsv, const FaixaHSV& range);

} // namespace robos3
