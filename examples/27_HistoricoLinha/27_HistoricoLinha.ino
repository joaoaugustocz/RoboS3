#include <RoboS3.h>

RoboS3 robo;

uint32_t proximaImpressaoMs = 0;

void imprimirHistorico()
{
    // A lista simples contem apenas os ultimos padroes, do mais novo ao mais antigo.
    PadraoLinha ultimosCasos[10];
    uint8_t quantidade = robo.copiarUltimosCasosLinha(ultimosCasos, 10);

    Serial.println("Ultimos casos:");
    for (uint8_t i = 0; i < quantidade; ++i)
    {
        char texto[9];
        RoboS3::padraoLinhaTexto(ultimosCasos[i], texto);
        Serial.printf("  %u: %s\n", i, texto);
    }

    // A lista detalhada usa o mesmo buffer e acrescenta inicio e duracao.
    CasoLinha historico[10];
    quantidade = robo.copiarHistoricoLinha(historico, 10);

    Serial.println("Historico com tempo:");
    for (uint8_t i = 0; i < quantidade; ++i)
    {
        char texto[9];
        RoboS3::padraoLinhaTexto(historico[i].padrao, texto);
        Serial.printf("  %u: %s, inicio=%lu ms, duracao=%lu ms\n", i, texto,
                      static_cast<unsigned long>(historico[i].inicioMs),
                      static_cast<unsigned long>(historico[i].duracaoMs));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    // Aceita de 0 a 32 casos. Zero desativa o historico e economiza processamento.
    // Alterar o tamanho limpa os registros anteriores. O tamanho inicial e 8.
    robo.configurarHistoricoLinha(10);
    robo.begin();
}

void loop()
{
    LeituraLinha linha = robo.lerLinha();

    // O campo caso permite nomes P/B diretamente no if ou switch.
    if (linha.caso == BBBPPBBB)
    {
        Serial.println("Linha centralizada: BBBPPBBB");
    }

    // O historico registra mudancas de padrao, nao todas as amostras repetidas.
    if (millis() >= proximaImpressaoMs)
    {
        proximaImpressaoMs = millis() + 1000;
        imprimirHistorico();
    }

    delay(20);
}
