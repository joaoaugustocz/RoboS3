#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();

    // Cria a rede RoboS3-Calibracao-XXXX e inicia o painel HTTP.
    // Nao e necessario adicionar nenhuma build_flag ao platformio.ini.
    if (!robo.iniciarCalibracaoWeb())
    {
        Serial.println("Nao foi possivel iniciar a calibracao Web.");
    }
}
void loop()
{
    // Mantem o servidor Web, a IMU e os filtros dos botoes atualizados.
    robo.atualizar();
}
