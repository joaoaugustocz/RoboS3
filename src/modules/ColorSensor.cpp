#include "ColorSensor.h"

#include <Arduino.h>
#include "RoboMath.h"

namespace robos3
{

ColorSensor::ColorSensor()
    : sensor_(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X), gain_(GANHO_4X),
      integration_(INTEGRACAO_50MS), ranges_{}, available_(false)
{
    loadDefaultRanges();
}

void ColorSensor::loadDefaultRanges()
{
    ranges_[COR_PRETA] = {0, 360, 0, 100, 0, 18, true};
    ranges_[COR_BRANCA] = {0, 360, 0, 18, 70, 100, true};
    ranges_[COR_VERMELHA] = {345, 15, 35, 100, 20, 100, true};
    ranges_[COR_LARANJA] = {16, 44, 35, 100, 20, 100, true};
    ranges_[COR_AMARELA] = {45, 75, 30, 100, 20, 100, true};
    ranges_[COR_VERDE] = {76, 165, 25, 100, 15, 100, true};
    ranges_[COR_AZUL] = {166, 260, 25, 100, 15, 100, true};
    ranges_[COR_ROXA] = {261, 344, 25, 100, 15, 100, true};
}

tcs34725Gain_t ColorSensor::nativeGain(GanhoCor gain)
{
    const tcs34725Gain_t values[] = {TCS34725_GAIN_1X, TCS34725_GAIN_4X, TCS34725_GAIN_16X,
                                     TCS34725_GAIN_60X};
    return values[static_cast<uint8_t>(gain) > 3 ? 0 : static_cast<uint8_t>(gain)];
}

uint8_t ColorSensor::nativeIntegration(TempoIntegracaoCor time)
{
    const uint8_t values[] = {TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_INTEGRATIONTIME_24MS,
                              TCS34725_INTEGRATIONTIME_50MS,  TCS34725_INTEGRATIONTIME_101MS,
                              TCS34725_INTEGRATIONTIME_154MS, TCS34725_INTEGRATIONTIME_600MS};
    return values[static_cast<uint8_t>(time) > 5 ? 2 : static_cast<uint8_t>(time)];
}

bool ColorSensor::begin(TwoWire& wire)
{
    available_ = sensor_.begin(TCS34725_ADDRESS, &wire);
    if (available_)
    {
        sensor_.setGain(nativeGain(gain_));
        sensor_.setIntegrationTime(nativeIntegration(integration_));
    }
    return available_;
}

void ColorSensor::setGain(GanhoCor gain)
{
    gain_ = gain;
    if (available_)
    {
        sensor_.setGain(nativeGain(gain));
    }
}

void ColorSensor::setIntegrationTime(TempoIntegracaoCor time)
{
    integration_ = time;
    if (available_)
    {
        sensor_.setIntegrationTime(nativeIntegration(time));
    }
}

void ColorSensor::resetConfiguration()
{
    loadDefaultRanges();
    setGain(GANHO_4X);
    setIntegrationTime(INTEGRACAO_50MS);
}

LeituraCor ColorSensor::read()
{
    LeituraCor reading{};
    reading.timestampMs = millis();
    reading.valida = available_;
    reading.cor = COR_DESCONHECIDA;
    if (!available_)
    {
        return reading;
    }
    sensor_.getRawData(&reading.rRaw, &reading.gRaw, &reading.bRaw, &reading.clear);
    reading.rgb = normalizeRgb(reading.rRaw, reading.gRaw, reading.bRaw, reading.clear);
    reading.hsv = rgbToHsv(reading.rgb);
    reading.hsl = rgbToHsl(reading.rgb);
    reading.cor = classify(reading);
    return reading;
}

void ColorSensor::setRange(CorDetectada color, const FaixaHSV& range)
{
    if (color > COR_DESCONHECIDA && color < COR_TOTAL)
    {
        ranges_[color] = range;
    }
}

const FaixaHSV& ColorSensor::range(CorDetectada color) const
{
    return ranges_[color < COR_TOTAL ? color : COR_DESCONHECIDA];
}

CorDetectada ColorSensor::classify(const LeituraCor& reading) const
{
    if (!reading.valida)
    {
        return COR_DESCONHECIDA;
    }
    if (hsvInRange(reading.hsv, ranges_[COR_PRETA]))
    {
        return COR_PRETA;
    }
    if (hsvInRange(reading.hsv, ranges_[COR_BRANCA]))
    {
        return COR_BRANCA;
    }
    for (uint8_t c = COR_VERMELHA; c < COR_TOTAL; ++c)
    {
        if (hsvInRange(reading.hsv, ranges_[c]))
        {
            return static_cast<CorDetectada>(c);
        }
    }
    return COR_DESCONHECIDA;
}

} // namespace robos3
