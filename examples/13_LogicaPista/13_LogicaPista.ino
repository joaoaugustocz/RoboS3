#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.begin();
}
void loop()
{
    LeituraLinha linha = robo.lerLinha();
    if (linha.mascara == 0)
    {
        if (robo.tempoSemLinha() > 100)
        {
            // Estrategia do aluno para confirmar perda de linha.
        }
        return;
    }
    if (linha.quantidadePretos >= 6)
    {
        // Uma leitura isolada nao confirma uma encruzilhada.
        return;
    }
    float c = robo.calcularPID((int)linha.posicao - 3500);
    robo.mover(60 + c, 60 - c);
}
