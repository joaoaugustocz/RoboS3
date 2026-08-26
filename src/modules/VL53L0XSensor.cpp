#include "VL53L0XSensor.h"

namespace robos3
{

VL53L0XSensor::VL53L0XSensor() : sensor_{}, available_(false)
{
}

bool VL53L0XSensor::begin(TwoWire& wire, uint16_t timeoutMs)
{
    sensor_.setBus(&wire);
    sensor_.setTimeout(timeoutMs);
    available_ = sensor_.init();
    if (available_)
    {
        sensor_.startContinuous();
    }
    return available_;
}

uint16_t VL53L0XSensor::distanceMm(bool& timeout)
{
    timeout = false;
    if (!available_)
    {
        return 0;
    }

    const uint16_t value = sensor_.readRangeContinuousMillimeters();
    timeout = sensor_.timeoutOccurred();
    if (timeout || value == 0 || value == UINT16_MAX)
    {
        return 0;
    }
    return value;
}

} // namespace robos3
