#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    // Use true somente se os conectores esquerdo e direito foram trocados.
    robo.trocarMotores(false);

    // Estas opcoes corrigem a polaridade de cada motor, nao trocam os lados.
    robo.inverterMotores(false, false);
    robo.begin();
}
void loop()
{
    robo.mover(30, 70);
    delay(800);
    robo.mover(-40, 40);
    delay(500);
    robo.frear();
    delay(1000);
}
