#pragma once

#include <MPU6050.h>
#include <Wire.h>
#include "../types/RoboTypes.h"

namespace robos3
{

struct IMUOffsets
{
    int16_t ax, ay, az, gx, gy, gz;
};

class IMUSensor
{
  public:
    IMUSensor();
    bool begin(TwoWire& wire, uint8_t address = 0x68);
    LeituraIMU read();
    void update()
    {
        read();
    }
    void zeroYaw();
    void calibrate(uint32_t durationMs = 3000);
    void setOffsets(const IMUOffsets& offsets)
    {
        offsets_ = offsets;
    }
    const IMUOffsets& offsets() const
    {
        return offsets_;
    }
    const LeituraIMU& last() const
    {
        return last_;
    }
    bool available() const
    {
        return available_;
    }

  private:
    alignas(MPU6050) uint8_t storage_[sizeof(MPU6050)];
    MPU6050* mpu_;
    IMUOffsets offsets_;
    LeituraIMU last_;
    uint32_t lastMicros_;
    float yawZero_;
    float yawIntegrated_;
    bool available_;
};

} // namespace robos3
