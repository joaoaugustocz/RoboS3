#pragma once

#include <QTRSensors.h>
#include "../types/RoboTypes.h"

namespace robos3
{

class LineSensor
{
  public:
    LineSensor();
    bool begin(const uint8_t pins[8], TipoQTR type, uint8_t emitterPin = 0xFF);
    void setTypeLine(TipoLinha type)
    {
        lineType_ = type;
    }
    void setThreshold(uint16_t threshold)
    {
        threshold_ = threshold > 1000 ? 1000 : threshold;
    }
    uint16_t threshold() const
    {
        return threshold_;
    }
    LeituraLinha read();
    void readRaw(uint16_t output[8]);
    void readCalibrated(uint16_t output[8]);
    void calibrateMs(uint32_t durationMs);
    void calibrateSamples(uint16_t samples);
    bool getCalibration(uint16_t minimum[8], uint16_t maximum[8]) const;
    bool setCalibration(const uint16_t minimum[8], const uint16_t maximum[8]);
    void setHistoryCapacity(uint8_t capacity);
    void clearHistory();
    uint8_t historyCapacity() const
    {
        return historyCapacity_;
    }
    uint8_t historyCount() const
    {
        return historyCount_;
    }
    uint8_t copyRecentPatterns(PadraoLinha output[], uint8_t maximum) const;
    uint8_t copyHistory(CasoLinha output[], uint8_t maximum) const;
    const LeituraLinha& last() const
    {
        return last_;
    }
    uint32_t timeWithoutLine() const;
    bool available() const
    {
        return begun_;
    }

  private:
    void calibrateRaw(const uint16_t raw[8], uint16_t output[8]) const;
    uint16_t positionFrom(const uint16_t calibrated[8], bool& valid);
    void updateHistory(PadraoLinha pattern, uint32_t nowMs);
    uint8_t historyIndexFromNewest(uint8_t offset) const;

    QTRSensors qtr_;
    TipoLinha lineType_;
    uint16_t threshold_;
    LeituraLinha last_;
    uint32_t lastDetectionMs_;
    uint16_t lastValidPosition_;
    bool begun_;
    CasoLinha history_[MAX_HISTORICO_LINHA];
    uint8_t historyCapacity_;
    uint8_t historyCount_;
    uint8_t historyNext_;
};

} // namespace robos3
