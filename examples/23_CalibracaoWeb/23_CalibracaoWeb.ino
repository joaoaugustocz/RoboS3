// PlatformIO: use o ambiente robos3_web ou -DROBOS3_CALIBRATION_WEB.
#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    robo.atualizar();
}
