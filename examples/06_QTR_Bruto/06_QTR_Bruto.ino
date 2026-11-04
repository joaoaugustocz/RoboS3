#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    // Descomente se a barra estiver invertida (S1 a direita e S8 a esquerda).
    // robo.espelharSensoresLinha();
    robo.begin();
}
void loop()
{
    // Leitura bruta e util para conferir ligacao, ordem S1-S8 e contraste da pista.
    uint16_t v[8];
    robo.lerLinhaBruta(v);

    // Os valores seguem a ordem logica S1, S2, ... S8.
    for (auto x : v)
    {
        Serial.print(x);
        Serial.print(' ');
    }
    Serial.println();
    delay(50);
}
