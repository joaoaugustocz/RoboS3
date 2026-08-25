#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
    robo.zerarAnguloZ();
}
void loop()
{
    LeituraIMU i = robo.lerIMU();
    Serial.printf("Roll %.1f Pitch %.1f Z relativo %.1f\n", i.roll, i.pitch, i.yawRelativo);
    delay(20);
}
