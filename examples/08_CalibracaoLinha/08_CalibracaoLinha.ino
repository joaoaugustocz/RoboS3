#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
    Serial.println("Mova o robo sobre branco e preto");
    robo.calibrarLinha(5000);
    robo.salvarCalibracao();
}
void loop()
{
    robo.lerLinha();
    robo.printLinha();
    delay(200);
}
