#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.definePinsQTR(4, 5, 6, 7, 15, 16, 17, 18);
    robo.definePinsPonteH(0, 36, 39, 38, 37, 35);
    robo.definePinsUltrassom(9, 46);
    robo.definePinsI2C0(1, 42);
    robo.definePinsI2C1(41, 40);
    robo.begin();
}
void loop()
{
}
