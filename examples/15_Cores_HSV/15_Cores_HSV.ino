#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    HSV c = robo.lerHSV(COR_ESQUERDA);
    Serial.printf("H %.1f S %.1f V %.1f\n", c.h, c.s, c.v);
    delay(100);
}
