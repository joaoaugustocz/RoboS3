#pragma once

#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include "../types/RoboTypes.h"

namespace robos3
{

class ColorSensor
{
  public:
    ColorSensor();
    bool begin(TwoWire& wire);
    void setGain(GanhoCor gain);
    void setIntegrationTime(TempoIntegracaoCor time);
    GanhoCor gain() const
    {
        return gain_;
    }
    TempoIntegracaoCor integrationTime() const
    {
        return integration_;
    }
    LeituraCor read();
    void setRange(CorDetectada color, const FaixaHSV& range);
    const FaixaHSV& range(CorDetectada color) const;
    CorDetectada classify(const LeituraCor& reading) const;
    bool available() const
    {
        return available_;
    }

  private:
    static tcs34725Gain_t nativeGain(GanhoCor gain);
    static uint8_t nativeIntegration(TempoIntegracaoCor time);
    void loadDefaultRanges();

    Adafruit_TCS34725 sensor_;
    GanhoCor gain_;
    TempoIntegracaoCor integration_;
    FaixaHSV ranges_[COR_TOTAL];
    bool available_;
};

} // namespace robos3
