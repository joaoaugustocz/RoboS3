#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    // A leitura calibrada normaliza cada sensor para a faixa de 0 a 1000.
    // Execute primeiro o exemplo 08 para obter bons limites de branco e preto.
    uint16_t v[8];
    robo.lerLinhaCalibrada(v);
    for (auto x : v)
    {
        Serial.print(x);
        Serial.print(' ');
    }
    Serial.println();
    delay(50);
}
