#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.begin();
}
void loop()
{
    if (robo.lerCor(COR_ESQUERDA) == COR_VERDE)
    {
        robo.frear();
    }
}
