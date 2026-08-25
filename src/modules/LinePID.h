#pragma once

#include <PIDEasy.h>

namespace robos3
{

class LinePID
{
  public:
    LinePID();
    void configure(float kp, float ki, float kd, float minimum = -100, float maximum = 100);
    void reset();
    float compute(float error);

  private:
    PID pid_;
    float kp_, ki_, kd_, minimum_, maximum_;
    unsigned long lastMillis_;
};

} // namespace robos3
