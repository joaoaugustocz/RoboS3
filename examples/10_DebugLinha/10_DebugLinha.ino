// Nivel 3 habilita mensagens detalhadas da biblioteca durante o diagnostico.
#define ROBOS3_DEBUG_LEVEL 3
#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    // Atualiza a leitura antes de imprimir bruto, calibrado, mascara e posicao.
    robo.lerLinha();
    robo.printLinha();
    delay(250);
}
