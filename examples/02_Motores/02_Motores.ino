#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.begin();
}
void loop()
{
    robo.frente(50);
    delay(1000);
    robo.parar();
    delay(500);
    robo.tras(50);
    delay(1000);
    robo.frear();
    delay(1000);
}
