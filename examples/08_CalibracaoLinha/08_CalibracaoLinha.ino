#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
    Serial.println("Mova o robo sobre branco e preto");
    if (!robo.calibrarLinha(5000))
    {
        Serial.println("Calibracao QTR invalida; dados nao salvos.");
        return;
    }
    if (!robo.salvarCalibracao())
    {
        Serial.println("Falha ao salvar calibracao QTR.");
    }
}
void loop()
{
    robo.lerLinha();
    robo.printLinha();
    delay(200);
}
