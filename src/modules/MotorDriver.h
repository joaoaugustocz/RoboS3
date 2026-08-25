#pragma once

#include <stdint.h>
#include "../config/BoardConfig.h"
#include "../types/RoboTypes.h"

namespace robos3
{

class MotorDriver
{
  public:
    MotorDriver();
    bool begin(const MotorPins& pins, uint32_t frequency, uint8_t resolution);
    void move(int left, int right);
    void stop();
    void brake();
    void setCalibration(Motor motor, const CalibracaoMotor& calibration);
    const CalibracaoMotor& calibration(Motor motor) const;
    int leftCommand() const
    {
        return leftCommand_;
    }
    int rightCommand() const
    {
        return rightCommand_;
    }
    uint16_t pwmMax() const
    {
        return pwmMax_;
    }

  private:
    void drive(bool left, int command);
    MotorPins pins_;
    CalibracaoMotor calibration_[2];
    uint16_t pwmMax_;
    int leftCommand_;
    int rightCommand_;
    bool begun_;
};

} // namespace robos3
