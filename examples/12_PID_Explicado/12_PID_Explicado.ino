#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.begin();
    robo.configurarPID(0.02f, 0.0f, 0.05f);
}
void loop()
{
    // SENSOR -> POSICAO -> ERRO -> PID -> CORRECAO -> MOTORES
    LeituraLinha linha = robo.lerLinha();
    if (!linha.detectada)
    {
        robo.parar();
        return;
    }
    int erro = (int)linha.posicao - 3500;
    float correcao = robo.calcularPID(erro);
    int base = 60;
    robo.mover(base + correcao, base - correcao);
}
