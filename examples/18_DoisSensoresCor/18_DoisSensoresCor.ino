#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);

    // Use true se o TCS esquerdo foi conectado no canal do direito e vice-versa.
    // Esta configuracao precisa ser feita antes de robo.begin().
    robo.trocarSensoresCor(false);
    robo.begin();
}
void loop()
{
    LeituraCor e = robo.lerDadosCor(COR_ESQUERDA), d = robo.lerDadosCor(COR_DIREITA);
    Serial.printf("E=%u D=%u\n", e.cor, d.cor);
    delay(100);
}
