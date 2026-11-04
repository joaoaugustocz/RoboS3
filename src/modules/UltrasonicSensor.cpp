#include "UltrasonicSensor.h"

#include <new>
#include <math.h>

namespace robos3
{

namespace
{
constexpr uint32_t MIN_INTERVALO_MEDICAO_MS = 60;
}

UltrasonicSensor::UltrasonicSensor()
    : storage_{}, sensor_(nullptr), begun_(false), lastDistanceCm_(NAN),
      lastMeasurementMs_(0), hasMeasurement_(false)
{
}

bool UltrasonicSensor::begin(uint8_t triggerPin, uint8_t echoPin, uint16_t maximumCm)
{
    sensor_ = new (storage_) UltraSonicDistanceSensor(triggerPin, echoPin, maximumCm);
    begun_ = sensor_ != nullptr;
    lastDistanceCm_ = NAN;
    lastMeasurementMs_ = 0;
    hasMeasurement_ = false;
    return begun_;
}

float UltrasonicSensor::distanceCm()
{
    if (!begun_)
    {
        return NAN;
    }

    const uint32_t now = millis();
    if (hasMeasurement_ && now - lastMeasurementMs_ < MIN_INTERVALO_MEDICAO_MS)
    {
        return lastDistanceCm_;
    }

    const float value = sensor_->measureDistanceCm();
    lastDistanceCm_ = value > 0 && isfinite(value) ? value : NAN;
    lastMeasurementMs_ = millis();
    hasMeasurement_ = true;
    return lastDistanceCm_;
}

bool UltrasonicSensor::hasObstacle(float maximumCm)
{
    const float value = distanceCm();
    return isfinite(value) && value <= maximumCm;
}

} // namespace robos3
