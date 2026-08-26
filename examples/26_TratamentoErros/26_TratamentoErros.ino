#include <RoboS3.h>

RoboS3 robo;

void setup()
{
    Serial.begin(115200);
    delay(500);

    // begin() tenta iniciar todos os modulos, mesmo quando um deles falha.
    // Se houver falha, ele ja imprime automaticamente o diagnostico na Serial.
    bool tudoPronto = robo.begin();

    if (!tudoPronto)
    {
        // status() permite que o programa trate cada modulo separadamente.
        const StatusRobo& status = robo.status();

        if (!status.imu)
        {
            Serial.println("O programa continuara sem curvas controladas pela IMU.");
        }

        if (!status.corEsquerda || !status.corDireita)
        {
            Serial.println("O programa continuara sem a logica dos marcadores de cor.");
        }

        if (!status.ultrassom)
        {
            Serial.println("O programa continuara sem detectar obstaculos.");
        }

        // Sem motores ou linha, este exemplo nao consegue executar sua funcao
        // basica. Mantemos o robo parado, mas o Monitor Serial continua ativo.
        if (!robo.operacaoBasicaDisponivel())
        {
            robo.parar();
            Serial.println("Motores ou QTR indisponiveis. Execucao interrompida.");

            while (true)
            {
                delay(1000);
            }
        }
    }

    Serial.println("Inicializacao tratada. O programa pode continuar.");
}

void loop()
{
    robo.atualizar();

    // Antes de usar um modulo opcional, confirme se ele esta disponivel.
    if (robo.imuDisponivel())
    {
        Serial.print("Angulo Z: ");
        Serial.println(robo.anguloZ());
    }

    if (robo.ultrassomDisponivel())
    {
        Serial.print("Distancia: ");
        Serial.print(robo.distanciaCM());
        Serial.println(" cm");
    }

    delay(500);
}
