#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
    Serial.println("Posicione o verde");
    delay(2000);
    robo.calibrarCor(COR_ESQUERDA, COR_VERDE, 30);
    robo.salvarCalibracao();
}
void loop()
{
}
