#include <RoboS3.h>

RoboS3 robo;

void setup()
{
    // Inicializa todos os modulos e deixa os motores parados.
    robo.begin();
}

void loop()
{
    // Velocidade em porcentagem: 50 significa metade da potencia configurada.
    robo.frente(50);
}
