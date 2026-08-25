#include "RoboDebug.h"

namespace robos3
{

void RoboDebug::log(DebugLevel level, const char* message) const
{
    if (level_ < level || !output_)
    {
        return;
    }
    output_->print(F("[RoboS3] "));
    output_->println(message);
}

void RoboDebug::module(const char* name, bool ok) const
{
    if (level_ < DEBUG_INFO || !output_)
    {
        return;
    }
    output_->print(ok ? F("[OK] ") : F("[ERRO] "));
    output_->println(name);
}

} // namespace robos3
