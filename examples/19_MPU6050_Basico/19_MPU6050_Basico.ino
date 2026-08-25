#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    LeituraIMU i = robo.lerIMU();
    Serial.printf("A %.2f %.2f %.2f G %.2f %.2f %.2f\n", i.accX, i.accY, i.accZ, i.gyroX, i.gyroY,
                  i.gyroZ);
    delay(20);
}
