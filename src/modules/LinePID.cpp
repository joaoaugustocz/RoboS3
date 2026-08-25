#include "LinePID.h"

#include <Arduino.h>

namespace robos3
{

LinePID::LinePID()
    : pid_(0.02f, 0.0f, 0.05f), kp_(0.02f), ki_(0), kd_(0.05f), minimum_(-100), maximum_(100),
      lastMillis_(0)
{
    pid_.setConstrain(-100, 100);
    pid_.setWindUP(-1000, 1000);
    pid_.setSmoothingDerivate(0.25f);
    pid_.setDampingFactor(0.5f);
}

void LinePID::configure(float kp, float ki, float kd, float minimum, float maximum)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    minimum_ = minimum;
    maximum_ = maximum;
    reset();
}

void LinePID::reset()
{
    pid_ = PID(kp_, ki_, kd_);
    pid_.setConstrain(static_cast<int>(minimum_), static_cast<int>(maximum_));
    pid_.setWindUP(-1000, 1000);
    pid_.setSmoothingDerivate(0.25f);
    pid_.setDampingFactor(0.5f);
    lastMillis_ = millis();
}

float LinePID::compute(float error)
{
    const unsigned long now = millis();
    unsigned long dt = lastMillis_ == 0 ? 1 : now - lastMillis_;
    if (dt == 0)
    {
        dt = 1;
    }
    lastMillis_ = now;
    return pid_.compute(error, dt);
}

} // namespace robos3
