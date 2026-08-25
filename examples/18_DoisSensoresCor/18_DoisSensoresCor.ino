#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    LeituraCor e = robo.lerDadosCor(COR_ESQUERDA), d = robo.lerDadosCor(COR_DIREITA);
    Serial.printf("E=%u D=%u\n", e.cor, d.cor);
    delay(100);
}
