#define ROBOS3_DEBUG_LEVEL 3
#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    robo.lerLinha();
    robo.printLinha();
    delay(250);
}
