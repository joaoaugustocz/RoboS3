#pragma once

#include <Arduino.h>
#include "../types/RoboTypes.h"

#ifndef ROBOS3_DEBUG_LEVEL
#ifdef ROBOS3_DEBUG
#define ROBOS3_DEBUG_LEVEL 2
#else
#define ROBOS3_DEBUG_LEVEL 0
#endif
#endif

namespace robos3
{

class RoboDebug
{
  public:
    RoboDebug() : output_(&Serial), level_(static_cast<DebugLevel>(ROBOS3_DEBUG_LEVEL))
    {
    }
    void setOutput(Print& output)
    {
        output_ = &output;
    }
    void setLevel(DebugLevel level)
    {
        level_ = level;
    }
    DebugLevel level() const
    {
        return level_;
    }
    void log(DebugLevel level, const char* message) const;
    void module(const char* name, bool ok) const;

  private:
    Print* output_;
    DebugLevel level_;
};

} // namespace robos3
