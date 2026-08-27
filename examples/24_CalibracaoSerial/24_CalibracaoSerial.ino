#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();

    // Usa a porta e a velocidade que o proprio sketch inicializou.
    // Nao e necessario adicionar nenhuma build_flag ao platformio.ini.
    robo.iniciarCalibracaoSerial(Serial);
}
void loop()
{
    // Processa comandos sem bloquear o restante do programa.
    robo.atualizar();
}
