#include "UltrasonicSensor.h"

#include <new>
#include <math.h>

namespace robos3
{

UltrasonicSensor::UltrasonicSensor() : storage_{}, sensor_(nullptr), begun_(false)
{
}

bool UltrasonicSensor::begin(uint8_t triggerPin, uint8_t echoPin, uint16_t maximumCm)
{
    sensor_ = new (storage_) UltraSonicDistanceSensor(triggerPin, echoPin, maximumCm);
    begun_ = sensor_ != nullptr;
    return begun_;
}

float UltrasonicSensor::distanceCm()
{
    if (!begun_)
    {
        return NAN;
    }
    const float value = sensor_->measureDistanceCm();
    return value > 0 && isfinite(value) ? value : NAN;
}

bool UltrasonicSensor::hasObstacle(float maximumCm)
{
    const float value = distanceCm();
    return isfinite(value) && value <= maximumCm;
}

} // namespace robos3
