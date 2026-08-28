#include "CalibrationStorage.h"

#include <Preferences.h>

namespace robos3
{

uint32_t CalibrationStorage::checksum(const RoboCalibration& data)
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < sizeof(RoboCalibration) - sizeof(data.checksum); ++i)
    {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}

bool CalibrationStorage::save(const RoboCalibration& input)
{
    RoboCalibration data = input;
    data.magic = CALIBRATION_MAGIC;
    data.schema = CALIBRATION_SCHEMA;
    data.size = sizeof(RoboCalibration);
    data.checksum = checksum(data);
    Preferences preferences;
    if (!preferences.begin("robos3", false))
    {
        return false;
    }
    const size_t written = preferences.putBytes("calibration", &data, sizeof(data));
    preferences.end();
    return written == sizeof(data);
}

bool CalibrationStorage::load(RoboCalibration& data)
{
    Preferences preferences;
    if (!preferences.begin("robos3", true))
    {
        return false;
    }
    if (preferences.getBytesLength("calibration") != sizeof(data))
    {
        preferences.end();
        return false;
    }
    const size_t read = preferences.getBytes("calibration", &data, sizeof(data));
    preferences.end();
    const bool schemaSuportado =
        data.schema == CALIBRATION_SCHEMA || data.schema == CALIBRATION_SCHEMA_QTR_10_BITS;
    return read == sizeof(data) && data.magic == CALIBRATION_MAGIC && schemaSuportado &&
           data.size == sizeof(data) && data.checksum == checksum(data);
}

bool CalibrationStorage::erase()
{
    Preferences preferences;
    if (!preferences.begin("robos3", false))
    {
        return false;
    }
    const bool result = preferences.clear();
    preferences.end();
    return result;
}

} // namespace robos3
