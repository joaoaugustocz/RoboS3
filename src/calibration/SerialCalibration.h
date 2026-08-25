#pragma once

#include <Arduino.h>

namespace robos3
{
class CalibrationManager;

class SerialCalibration
{
  public:
    explicit SerialCalibration(CalibrationManager& manager);
    void begin(Stream& stream = Serial);
    void update();

  private:
    void execute(char* line);
    void help();
    CalibrationManager& manager_;
    Stream* stream_;
    char buffer_[128];
    uint8_t length_;
};

} // namespace robos3
