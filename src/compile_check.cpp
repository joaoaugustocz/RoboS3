#ifdef ROBOS3_COMPILE_CHECK
#include "RoboS3.h"

RoboS3 robos3CompileCheck;

void setup()
{
    robos3CompileCheck.begin();
}

void loop()
{
    robos3CompileCheck.atualizar();
}
#endif
