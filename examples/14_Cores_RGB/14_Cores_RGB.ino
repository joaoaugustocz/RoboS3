#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    RGB c = robo.lerRGB(COR_ESQUERDA);
    Serial.printf("R %.1f G %.1f B %.1f\n", c.r, c.g, c.b);
    delay(100);
}
