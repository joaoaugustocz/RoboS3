#include "MotorDriver.h"

#include <Arduino.h>
#include "RoboMath.h"

namespace robos3
{

MotorDriver::MotorDriver()
    : pins_{}, calibration_{{1, 1, 18, 18, false}, {1, 1, 18, 18, false}}, pwmMax_(0),
      leftCommand_(0), rightCommand_(0), begun_(false)
{
}

bool MotorDriver::begin(const MotorPins& pins, uint32_t frequency, uint8_t resolution)
{
    pins_ = pins;
    pinMode(pins_.in1, OUTPUT);
    pinMode(pins_.in2, OUTPUT);
    pinMode(pins_.in3, OUTPUT);
    pinMode(pins_.in4, OUTPUT);
    digitalWrite(pins_.in1, LOW);
    digitalWrite(pins_.in2, LOW);
    digitalWrite(pins_.in3, LOW);
    digitalWrite(pins_.in4, LOW);
    resolution = resolution > 14 ? 14 : resolution;
    pwmMax_ = static_cast<uint16_t>((1UL << resolution) - 1UL);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    const bool leftOk = ledcAttach(pins_.ena, frequency, resolution);
    const bool rightOk = ledcAttach(pins_.enb, frequency, resolution);
#else
    ledcSetup(0, frequency, resolution);
    ledcAttachPin(pins_.ena, 0);
    ledcSetup(1, frequency, resolution);
    ledcAttachPin(pins_.enb, 1);
    const bool leftOk = true, rightOk = true;
#endif
    begun_ = leftOk && rightOk;
    stop();
    return begun_;
}

void MotorDriver::drive(bool left, int command)
{
    const MotorOutput out = motorOutput(command, calibration_[left ? 0 : 1], pwmMax_);
    const uint8_t inA = left ? pins_.in1 : pins_.in3;
    const uint8_t inB = left ? pins_.in2 : pins_.in4;
    const uint8_t enable = left ? pins_.ena : pins_.enb;
    digitalWrite(inA, out.forward ? HIGH : LOW);
    digitalWrite(inB, out.forward ? LOW : HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(enable, out.duty);
#else
    ledcWrite(left ? 0 : 1, out.duty);
#endif
}

void MotorDriver::move(int left, int right)
{
    if (!begun_)
    {
        return;
    }
    leftCommand_ = clampCommand(left);
    rightCommand_ = clampCommand(right);
    drive(true, leftCommand_);
    drive(false, rightCommand_);
}

void MotorDriver::stop()
{
    leftCommand_ = rightCommand_ = 0;
    if (!begun_)
    {
        return;
    }
    digitalWrite(pins_.in1, LOW);
    digitalWrite(pins_.in2, LOW);
    digitalWrite(pins_.in3, LOW);
    digitalWrite(pins_.in4, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pins_.ena, 0);
    ledcWrite(pins_.enb, 0);
#else
    ledcWrite(0, 0);
    ledcWrite(1, 0);
#endif
}

void MotorDriver::brake()
{
    leftCommand_ = rightCommand_ = 0;
    if (!begun_)
    {
        return;
    }
    digitalWrite(pins_.in1, HIGH);
    digitalWrite(pins_.in2, HIGH);
    digitalWrite(pins_.in3, HIGH);
    digitalWrite(pins_.in4, HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pins_.ena, pwmMax_);
    ledcWrite(pins_.enb, pwmMax_);
#else
    ledcWrite(0, pwmMax_);
    ledcWrite(1, pwmMax_);
#endif
}

void MotorDriver::setCalibration(Motor motor, const CalibracaoMotor& calibration)
{
    calibration_[motor == MOTOR_ESQUERDO ? 0 : 1] = calibration;
}

const CalibracaoMotor& MotorDriver::calibration(Motor motor) const
{
    return calibration_[motor == MOTOR_ESQUERDO ? 0 : 1];
}

} // namespace robos3
