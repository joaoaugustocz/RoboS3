#include "RoboDebug.h"

namespace robos3
{

void RoboDebug::log(DebugLevel level, const char* message) const
{
    if (level_ < level || !output_)
    {
        return;
    }
    output_->print(F("[RoboS3]["));
    switch (level)
    {
        case DEBUG_ERROR:
            output_->print(F("ERRO"));
            break;
        case DEBUG_INFO:
            output_->print(F("INFO"));
            break;
        case DEBUG_VERBOSE:
            output_->print(F("DETALHE"));
            break;
        default:
            output_->print(F("DEBUG"));
            break;
    }
    output_->print(F("] "));
    output_->println(message);
}

void RoboDebug::module(const char* name, bool ok) const
{
    if (level_ < DEBUG_INFO || !output_)
    {
        return;
    }
    output_->print(ok ? F("[RoboS3][OK] ") : F("[RoboS3][ERRO] "));
    output_->println(name);
}

} // namespace robos3
