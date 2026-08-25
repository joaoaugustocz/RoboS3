#pragma once

#include <stdint.h>
#include "../types/RoboTypes.h"
#include "IMUSensor.h"

namespace robos3
{

constexpr uint32_t CALIBRATION_MAGIC = 0x524F424F;
constexpr uint16_t CALIBRATION_SCHEMA = 1;

struct RoboCalibration
{
    uint32_t magic;
    uint16_t schema;
    uint16_t size;
    uint16_t lineMinimum[8];
    uint16_t lineMaximum[8];
    uint16_t lineThreshold;
    CalibracaoMotor leftMotor;
    CalibracaoMotor rightMotor;
    FaixaHSV colorRanges[2][COR_TOTAL];
    uint8_t colorGain[2];
    uint8_t colorIntegration[2];
    IMUOffsets imuOffsets;
    uint32_t checksum;
};

class CalibrationStorage
{
  public:
    bool save(const RoboCalibration& data);
    bool load(RoboCalibration& data);
    bool erase();
    static uint32_t checksum(const RoboCalibration& data);
};

} // namespace robos3
