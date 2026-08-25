#include "IMUSensor.h"

#include <Arduino.h>
#include <math.h>
#include <new>

namespace robos3
{

IMUSensor::IMUSensor()
    : storage_{}, mpu_(nullptr), offsets_{}, last_{}, lastMicros_(0), yawZero_(0),
      yawIntegrated_(0), available_(false)
{
}

bool IMUSensor::begin(TwoWire& wire, uint8_t address)
{
    mpu_ = new (storage_) MPU6050(address, &wire);
    mpu_->initialize();
    available_ = mpu_->testConnection();
    lastMicros_ = micros();
    return available_;
}

LeituraIMU IMUSensor::read()
{
    LeituraIMU reading = last_;
    reading.timestampMs = millis();
    reading.valida = available_;
    if (!available_)
    {
        last_ = reading;
        return reading;
    }
    int16_t ax, ay, az, gx, gy, gz;
    mpu_->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax -= offsets_.ax;
    ay -= offsets_.ay;
    az -= offsets_.az;
    gx -= offsets_.gx;
    gy -= offsets_.gy;
    gz -= offsets_.gz;
    constexpr float gravity = 9.80665f;
    reading.accX = ax * gravity / 16384.0f;
    reading.accY = ay * gravity / 16384.0f;
    reading.accZ = az * gravity / 16384.0f;
    reading.gyroX = gx / 131.0f;
    reading.gyroY = gy / 131.0f;
    reading.gyroZ = gz / 131.0f;
    const float accRoll = atan2f(reading.accY, reading.accZ) * 180.0f / PI;
    const float accPitch =
        atan2f(-reading.accX, sqrtf(reading.accY * reading.accY + reading.accZ * reading.accZ)) *
        180.0f / PI;
    const uint32_t now = micros();
    float dt = (now - lastMicros_) / 1000000.0f;
    lastMicros_ = now;
    if (dt <= 0 || dt > 0.25f)
    {
        dt = 0;
    }
    reading.roll = 0.98f * (last_.roll + reading.gyroX * dt) + 0.02f * accRoll;
    reading.pitch = 0.98f * (last_.pitch + reading.gyroY * dt) + 0.02f * accPitch;
    yawIntegrated_ += reading.gyroZ * dt;
    reading.yawRelativo = yawIntegrated_ - yawZero_;
    reading.temperatura = mpu_->getTemperature() / 340.0f + 36.53f;
    last_ = reading;
    return reading;
}

void IMUSensor::zeroYaw()
{
    yawZero_ = yawIntegrated_;
    last_.yawRelativo = 0;
}

void IMUSensor::calibrate(uint32_t durationMs)
{
    if (!available_)
    {
        return;
    }
    int64_t sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
    uint32_t count = 0, start = millis();
    while (millis() - start < durationMs)
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu_->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sax += ax;
        say += ay;
        saz += az - 16384;
        sgx += gx;
        sgy += gy;
        sgz += gz;
        ++count;
        delay(2);
    }
    if (count)
    {
        offsets_ = {static_cast<int16_t>(sax / count), static_cast<int16_t>(say / count),
                    static_cast<int16_t>(saz / count), static_cast<int16_t>(sgx / count),
                    static_cast<int16_t>(sgy / count), static_cast<int16_t>(sgz / count)};
    }
    yawIntegrated_ = yawZero_ = 0;
}

} // namespace robos3
