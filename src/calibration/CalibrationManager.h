#pragma once

#include <Arduino.h>
#include "../types/RoboTypes.h"

class RoboS3;

namespace robos3
{

class CalibrationManager
{
  public:
    explicit CalibrationManager(RoboS3& robo);
    RoboS3& robo()
    {
        return robo_;
    }
    void update();
    void testMotors(int left, int right, uint32_t timeoutMs = 1500);
    void stopMotors(bool brake = false);
    bool save();
    bool load();
    bool reset();
    void calibrateLine(uint32_t durationMs);
    void calibrateImu(uint32_t durationMs);

  private:
    RoboS3& robo_;
    uint32_t motorDeadlineMs_;
    bool motorTestActive_;
};

} // namespace robos3
