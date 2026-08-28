#include <RoboS3.h>

RoboS3 robo;

// O ID e apenas um nome numerico escolhido pela equipe.
constexpr uint8_t BOTAO_INICIAR = 1;
constexpr uint8_t BOTAO_MODO_1 = 2;
constexpr uint8_t BOTAO_MODO_2 = 3;
bool avisoPressionamentoLongo = false;
uint32_t ultimaImpressaoBotoesMs = 0;

void setup()
{
    Serial.begin(115200);

    // Botao digital: ligue um terminal ao GPIO35 e o outro ao GND.
    // O pull-up interno e o debounce de 30 ms sao configurados pela biblioteca.
    robo.adicionarBotao(BOTAO_INICIAR, 35);

    // Teclas analogicas: todas compartilham o GPIO4, mas cada resistor produz
    // uma faixa diferente. Meça valorBruto e ajuste estes limites na sua placa.
    // A leitura usa 12 bits (0...4095). Estes numeros sao apenas exemplos.
    robo.adicionarBotaoAnalogico(BOTAO_MODO_1, 4, 480, 880);
    robo.adicionarBotaoAnalogico(BOTAO_MODO_2, 4, 1720, 2160);

    robo.begin();
}

void loop()
{
    // atualizar() cuida do debounce. Chame-a frequentemente e evite delays longos.
    robo.atualizar();

    if (robo.botaoFoiApertado(BOTAO_INICIAR))
    {
        Serial.println("Botao iniciar foi apertado uma vez.");
    }

    if (robo.botaoFoiSolto(BOTAO_INICIAR))
    {
        Serial.println("Botao iniciar foi solto.");
    }

    if (robo.botaoPressionadoPor(BOTAO_INICIAR, 2000) && !avisoPressionamentoLongo)
    {
        avisoPressionamentoLongo = true;
        Serial.println("Botao iniciar esta pressionado ha pelo menos 2 segundos.");
    }
    if (!robo.botaoPressionado(BOTAO_INICIAR))
    {
        avisoPressionamentoLongo = false;
    }

    if (robo.botaoFoiApertado(BOTAO_MODO_1))
    {
        Serial.println("Modo analogico 1 ativo.");
    }

    // A estrutura completa ajuda a descobrir as faixas do teclado resistivo.
    // O intervalo evita inundar o Monitor Serial e atrasar o restante do loop.
    if (millis() - ultimaImpressaoBotoesMs >= 250)
    {
        ultimaImpressaoBotoesMs = millis();
        LeituraBotao leitura = robo.lerBotao(BOTAO_MODO_2);
        Serial.printf("ADC GPIO%u = %u\n", leitura.pino, leitura.valorBruto);
    }
}
