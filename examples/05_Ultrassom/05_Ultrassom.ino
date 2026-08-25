#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    float d = robo.distanciaCM();
    Serial.println(d);
    if (robo.temObstaculo(20))
    {
        robo.parar();
    }
    delay(100);
}
