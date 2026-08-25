#include "RoboMath.h"

#include <math.h>

namespace robos3
{

static float clampf(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

int16_t clampCommand(int value)
{
    return static_cast<int16_t>(value < -100 ? -100 : (value > 100 ? 100 : value));
}

MotorOutput motorOutput(int command, const CalibracaoMotor& cal, uint16_t pwmMax)
{
    int16_t limited = clampCommand(command);
    if (limited == 0 || pwmMax == 0)
    {
        return {true, 0};
    }
    bool forward = limited > 0;
    if (cal.invertido)
    {
        forward = !forward;
    }
    const uint8_t minimum = limited > 0 ? cal.minimoFrente : cal.minimoTras;
    const float compensation = limited > 0 ? cal.compensacaoFrente : cal.compensacaoTras;
    const uint16_t minDuty =
        static_cast<uint16_t>((static_cast<uint32_t>(minimum) * pwmMax) / 100U);
    const uint16_t magnitude = static_cast<uint16_t>(limited > 0 ? limited : -limited);
    float duty = minDuty + (pwmMax - minDuty) * (magnitude - 1U) / 99.0f;
    duty = clampf(duty * compensation, 0.0f, static_cast<float>(pwmMax));
    return {forward, static_cast<uint16_t>(duty + 0.5f)};
}

uint8_t lineMask(const uint16_t values[8], uint16_t threshold, TipoLinha type)
{
    (void)type; // A mascara sempre descreve preto fisico; TipoLinha afeta apenas posicao/deteccao.
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {
        const bool black = values[i] >= threshold;
        if (black)
        {
            mask |= static_cast<uint8_t>(1U << (7U - i));
        }
    }
    return mask;
}

void linePattern(uint8_t mask, char output[9])
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        output[i] = (mask & (1U << (7U - i))) ? 'P' : 'B';
    }
    output[8] = '\0';
}

uint8_t countBits(uint8_t value)
{
    uint8_t count = 0;
    while (value)
    {
        count += value & 1U;
        value >>= 1U;
    }
    return count;
}

uint32_t elapsedWithoutDetection(uint32_t now, uint32_t lastDetection, bool detected)
{
    return detected ? 0 : now - lastDetection;
}

int16_t lineError(uint16_t position, int16_t center)
{
    return static_cast<int16_t>(static_cast<int32_t>(position) - center);
}

RGB normalizeRgb(uint16_t r, uint16_t g, uint16_t b, uint16_t clear)
{
    if (clear == 0)
    {
        return {0, 0, 0};
    }
    return {clampf(255.0f * r / clear, 0, 255), clampf(255.0f * g / clear, 0, 255),
            clampf(255.0f * b / clear, 0, 255)};
}

HSV rgbToHsv(const RGB& rgb)
{
    const float r = clampf(rgb.r / 255.0f, 0, 1);
    const float g = clampf(rgb.g / 255.0f, 0, 1);
    const float b = clampf(rgb.b / 255.0f, 0, 1);
    const float maximum = fmaxf(r, fmaxf(g, b));
    const float minimum = fminf(r, fminf(g, b));
    const float delta = maximum - minimum;
    float hue = 0;
    if (delta > 0.00001f)
    {
        if (maximum == r)
        {
            hue = 60.0f * fmodf((g - b) / delta, 6.0f);
        }
        else if (maximum == g)
        {
            hue = 60.0f * (((b - r) / delta) + 2.0f);
        }
        else
        {
            hue = 60.0f * (((r - g) / delta) + 4.0f);
        }
        if (hue < 0)
        {
            hue += 360.0f;
        }
    }
    return {hue, maximum <= 0 ? 0 : 100.0f * delta / maximum, 100.0f * maximum};
}

HSL rgbToHsl(const RGB& rgb)
{
    const float r = clampf(rgb.r / 255.0f, 0, 1);
    const float g = clampf(rgb.g / 255.0f, 0, 1);
    const float b = clampf(rgb.b / 255.0f, 0, 1);
    const float maximum = fmaxf(r, fmaxf(g, b));
    const float minimum = fminf(r, fminf(g, b));
    const float delta = maximum - minimum;
    const float lightness = (maximum + minimum) * 0.5f;
    float hue = 0;
    if (delta > 0.00001f)
    {
        if (maximum == r)
        {
            hue = 60.0f * fmodf((g - b) / delta, 6.0f);
        }
        else if (maximum == g)
        {
            hue = 60.0f * (((b - r) / delta) + 2.0f);
        }
        else
        {
            hue = 60.0f * (((r - g) / delta) + 4.0f);
        }
        if (hue < 0)
        {
            hue += 360.0f;
        }
    }
    const float saturation =
        delta <= 0.00001f ? 0 : delta / (1.0f - fabsf(2.0f * lightness - 1.0f));
    return {hue, saturation * 100.0f, lightness * 100.0f};
}

bool hueInRange(float hue, float minimum, float maximum)
{
    while (hue < 0)
    {
        hue += 360.0f;
    }
    while (hue >= 360)
    {
        hue -= 360.0f;
    }
    if (minimum <= maximum)
    {
        return hue >= minimum && hue <= maximum;
    }
    return hue >= minimum || hue <= maximum;
}

bool hsvInRange(const HSV& hsv, const FaixaHSV& range)
{
    return range.ativa && hueInRange(hsv.h, range.hMin, range.hMax) && hsv.s >= range.sMin &&
           hsv.s <= range.sMax && hsv.v >= range.vMin && hsv.v <= range.vMax;
}

} // namespace robos3
