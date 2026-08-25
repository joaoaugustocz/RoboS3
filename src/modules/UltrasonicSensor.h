#pragma once

#include <HCSR04.h>
#include <stdint.h>

namespace robos3
{

class UltrasonicSensor
{
  public:
    UltrasonicSensor();
    bool begin(uint8_t triggerPin, uint8_t echoPin, uint16_t maximumCm = 400);
    float distanceCm();
    bool hasObstacle(float maximumCm);
    bool available() const
    {
        return begun_;
    }

  private:
    alignas(UltraSonicDistanceSensor) uint8_t storage_[sizeof(UltraSonicDistanceSensor)];
    UltraSonicDistanceSensor* sensor_;
    bool begun_;
};

} // namespace robos3
