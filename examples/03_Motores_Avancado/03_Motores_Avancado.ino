#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.inverterMotores(false, false);
    robo.begin();
}
void loop()
{
    robo.mover(30, 70);
    delay(800);
    robo.mover(-40, 40);
    delay(500);
    robo.frear();
    delay(1000);
}
