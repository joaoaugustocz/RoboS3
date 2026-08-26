#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    // Cada bit representa um sensor: S1 e o bit 7 e S8 e o bit 0.
    // O bit 1 significa preto fisico, mesmo quando a pista usa linha branca.
    uint8_t m = robo.lerLinhaBinaria();

    // Exemplo: 10011000 corresponde ao padrao PBBPPBBB.
    for (int i = 7; i >= 0; --i)
    {
        Serial.print((m >> i) & 1);
    }
    Serial.println();
    delay(50);
}
