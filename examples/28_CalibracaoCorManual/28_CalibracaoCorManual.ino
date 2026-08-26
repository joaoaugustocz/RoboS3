#include <RoboS3.h>

RoboS3 robo;

void setup()
{
    Serial.begin(115200);
    delay(500);

    // Se algum modulo falhar, begin() informa automaticamente pela Serial.
    robo.begin();

    if (!robo.corDisponivel(COR_ESQUERDA))
    {
        Serial.println("Sensor de cor esquerdo indisponivel.");
        while (true)
        {
            delay(1000);
        }
    }

    // Primeiro observe H, S e V no Monitor Serial sobre cada cor da pista.
    // Depois substitua os limites abaixo pelos valores medidos no seu ambiente.
    robo.definirFaixaHSV(COR_ESQUERDA, COR_VERDE, 75, 165, 25, 100, 15, 100);
    robo.definirFaixaHSV(COR_ESQUERDA, COR_VERMELHA, 345, 15, 35, 100, 15, 100);
    robo.definirFaixaHSV(COR_ESQUERDA, COR_AZUL, 190, 260, 25, 100, 15, 100);

    // Ganho e tempo de integracao tambem dependem da iluminacao e distancia.
    robo.setGanhoCor(COR_ESQUERDA, GANHO_4X);
    robo.setTempoIntegracaoCor(COR_ESQUERDA, INTEGRACAO_50MS);

    // Salva as faixas, ganho e integracao na memoria do ESP32.
    if (robo.salvarCalibracao())
    {
        Serial.println("Calibracao manual salva.");
    }
    else
    {
        Serial.println("Nao foi possivel salvar a calibracao.");
    }
}

void loop()
{
    LeituraCor leitura = robo.lerDadosCor(COR_ESQUERDA);

    // Estes valores ajudam a refinar manualmente as faixas acima.
    Serial.printf("H=%.1f S=%.1f V=%.1f cor=%u valida=%s\n", leitura.hsv.h, leitura.hsv.s,
                  leitura.hsv.v, leitura.cor, leitura.valida ? "sim" : "nao");
    delay(200);
}
