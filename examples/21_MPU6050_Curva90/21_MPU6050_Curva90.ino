#include <RoboS3.h>
RoboS3 robo;

void setup()
{
    Serial.begin(115200);
    robo.begin();

    // Se o angulo medido crescer com sinal contrario na sua montagem, use:
    // robo.inverterAnguloZ(true);

    // Positivo gira para a direita; negativo gira para a esquerda.
    if (!robo.girarAngulo(40, 90))
    {
        Serial.println("Nao foi possivel completar o giro para a direita.");
    }

    delay(1000);

    if (!robo.girarAngulo(40, -90))
    {
        Serial.println("Nao foi possivel completar o giro para a esquerda.");
    }
}

void loop()
{
}
