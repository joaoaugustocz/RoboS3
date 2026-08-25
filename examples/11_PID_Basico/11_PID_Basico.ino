#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.configurarPID(0.02f, 0.0f, 0.05f);
    robo.begin();
}
void loop()
{
    robo.seguirLinha(60);
}
