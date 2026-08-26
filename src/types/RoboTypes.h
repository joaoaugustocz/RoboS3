#pragma once

#include <stdint.h>

/** @brief Identifica um dos dois motores da base. */
enum Motor : uint8_t
{
    MOTOR_ESQUERDO = 0, ///< Motor do lado esquerdo.
    MOTOR_DIREITO = 1  ///< Motor do lado direito.
};
/** @brief Identifica o sentido usado nas calibracoes de motor. */
enum SentidoMotor : uint8_t
{
    FRENTE = 0, ///< Movimento para frente.
    TRAS = 1    ///< Movimento para tras.
};
/** @brief Seleciona a interface eletrica do conjunto QTR. */
enum TipoQTR : uint8_t
{
    QTR_8A = 0, ///< Oito saidas analogicas.
    QTR_8RC = 1 ///< Oito saidas RC temporizadas.
};
/** @brief Seleciona a polaridade visual da pista seguida. */
enum TipoLinha : uint8_t
{
    LINHA_PRETA = 0, ///< Linha escura sobre fundo claro.
    LINHA_BRANCA = 1 ///< Linha clara sobre fundo escuro.
};
/**
 * @brief Padrao binario dos oito sensores de linha.
 *
 * Cada nome segue a ordem fisica S1 ate S8. P significa preto (bit 1) e
 * B significa branco (bit 0). Exemplo: BBBPPBBB equivale a 0b00011000.
 * Todos os 256 padroes possiveis estao disponiveis para uso em if ou switch.
 */
enum PadraoLinha : uint8_t
{
    BBBBBBBB = 0b00000000,
    BBBBBBBP = 0b00000001,
    BBBBBBPB = 0b00000010,
    BBBBBBPP = 0b00000011,
    BBBBBPBB = 0b00000100,
    BBBBBPBP = 0b00000101,
    BBBBBPPB = 0b00000110,
    BBBBBPPP = 0b00000111,
    BBBBPBBB = 0b00001000,
    BBBBPBBP = 0b00001001,
    BBBBPBPB = 0b00001010,
    BBBBPBPP = 0b00001011,
    BBBBPPBB = 0b00001100,
    BBBBPPBP = 0b00001101,
    BBBBPPPB = 0b00001110,
    BBBBPPPP = 0b00001111,
    BBBPBBBB = 0b00010000,
    BBBPBBBP = 0b00010001,
    BBBPBBPB = 0b00010010,
    BBBPBBPP = 0b00010011,
    BBBPBPBB = 0b00010100,
    BBBPBPBP = 0b00010101,
    BBBPBPPB = 0b00010110,
    BBBPBPPP = 0b00010111,
    BBBPPBBB = 0b00011000,
    BBBPPBBP = 0b00011001,
    BBBPPBPB = 0b00011010,
    BBBPPBPP = 0b00011011,
    BBBPPPBB = 0b00011100,
    BBBPPPBP = 0b00011101,
    BBBPPPPB = 0b00011110,
    BBBPPPPP = 0b00011111,
    BBPBBBBB = 0b00100000,
    BBPBBBBP = 0b00100001,
    BBPBBBPB = 0b00100010,
    BBPBBBPP = 0b00100011,
    BBPBBPBB = 0b00100100,
    BBPBBPBP = 0b00100101,
    BBPBBPPB = 0b00100110,
    BBPBBPPP = 0b00100111,
    BBPBPBBB = 0b00101000,
    BBPBPBBP = 0b00101001,
    BBPBPBPB = 0b00101010,
    BBPBPBPP = 0b00101011,
    BBPBPPBB = 0b00101100,
    BBPBPPBP = 0b00101101,
    BBPBPPPB = 0b00101110,
    BBPBPPPP = 0b00101111,
    BBPPBBBB = 0b00110000,
    BBPPBBBP = 0b00110001,
    BBPPBBPB = 0b00110010,
    BBPPBBPP = 0b00110011,
    BBPPBPBB = 0b00110100,
    BBPPBPBP = 0b00110101,
    BBPPBPPB = 0b00110110,
    BBPPBPPP = 0b00110111,
    BBPPPBBB = 0b00111000,
    BBPPPBBP = 0b00111001,
    BBPPPBPB = 0b00111010,
    BBPPPBPP = 0b00111011,
    BBPPPPBB = 0b00111100,
    BBPPPPBP = 0b00111101,
    BBPPPPPB = 0b00111110,
    BBPPPPPP = 0b00111111,
    BPBBBBBB = 0b01000000,
    BPBBBBBP = 0b01000001,
    BPBBBBPB = 0b01000010,
    BPBBBBPP = 0b01000011,
    BPBBBPBB = 0b01000100,
    BPBBBPBP = 0b01000101,
    BPBBBPPB = 0b01000110,
    BPBBBPPP = 0b01000111,
    BPBBPBBB = 0b01001000,
    BPBBPBBP = 0b01001001,
    BPBBPBPB = 0b01001010,
    BPBBPBPP = 0b01001011,
    BPBBPPBB = 0b01001100,
    BPBBPPBP = 0b01001101,
    BPBBPPPB = 0b01001110,
    BPBBPPPP = 0b01001111,
    BPBPBBBB = 0b01010000,
    BPBPBBBP = 0b01010001,
    BPBPBBPB = 0b01010010,
    BPBPBBPP = 0b01010011,
    BPBPBPBB = 0b01010100,
    BPBPBPBP = 0b01010101,
    BPBPBPPB = 0b01010110,
    BPBPBPPP = 0b01010111,
    BPBPPBBB = 0b01011000,
    BPBPPBBP = 0b01011001,
    BPBPPBPB = 0b01011010,
    BPBPPBPP = 0b01011011,
    BPBPPPBB = 0b01011100,
    BPBPPPBP = 0b01011101,
    BPBPPPPB = 0b01011110,
    BPBPPPPP = 0b01011111,
    BPPBBBBB = 0b01100000,
    BPPBBBBP = 0b01100001,
    BPPBBBPB = 0b01100010,
    BPPBBBPP = 0b01100011,
    BPPBBPBB = 0b01100100,
    BPPBBPBP = 0b01100101,
    BPPBBPPB = 0b01100110,
    BPPBBPPP = 0b01100111,
    BPPBPBBB = 0b01101000,
    BPPBPBBP = 0b01101001,
    BPPBPBPB = 0b01101010,
    BPPBPBPP = 0b01101011,
    BPPBPPBB = 0b01101100,
    BPPBPPBP = 0b01101101,
    BPPBPPPB = 0b01101110,
    BPPBPPPP = 0b01101111,
    BPPPBBBB = 0b01110000,
    BPPPBBBP = 0b01110001,
    BPPPBBPB = 0b01110010,
    BPPPBBPP = 0b01110011,
    BPPPBPBB = 0b01110100,
    BPPPBPBP = 0b01110101,
    BPPPBPPB = 0b01110110,
    BPPPBPPP = 0b01110111,
    BPPPPBBB = 0b01111000,
    BPPPPBBP = 0b01111001,
    BPPPPBPB = 0b01111010,
    BPPPPBPP = 0b01111011,
    BPPPPPBB = 0b01111100,
    BPPPPPBP = 0b01111101,
    BPPPPPPB = 0b01111110,
    BPPPPPPP = 0b01111111,
    PBBBBBBB = 0b10000000,
    PBBBBBBP = 0b10000001,
    PBBBBBPB = 0b10000010,
    PBBBBBPP = 0b10000011,
    PBBBBPBB = 0b10000100,
    PBBBBPBP = 0b10000101,
    PBBBBPPB = 0b10000110,
    PBBBBPPP = 0b10000111,
    PBBBPBBB = 0b10001000,
    PBBBPBBP = 0b10001001,
    PBBBPBPB = 0b10001010,
    PBBBPBPP = 0b10001011,
    PBBBPPBB = 0b10001100,
    PBBBPPBP = 0b10001101,
    PBBBPPPB = 0b10001110,
    PBBBPPPP = 0b10001111,
    PBBPBBBB = 0b10010000,
    PBBPBBBP = 0b10010001,
    PBBPBBPB = 0b10010010,
    PBBPBBPP = 0b10010011,
    PBBPBPBB = 0b10010100,
    PBBPBPBP = 0b10010101,
    PBBPBPPB = 0b10010110,
    PBBPBPPP = 0b10010111,
    PBBPPBBB = 0b10011000,
    PBBPPBBP = 0b10011001,
    PBBPPBPB = 0b10011010,
    PBBPPBPP = 0b10011011,
    PBBPPPBB = 0b10011100,
    PBBPPPBP = 0b10011101,
    PBBPPPPB = 0b10011110,
    PBBPPPPP = 0b10011111,
    PBPBBBBB = 0b10100000,
    PBPBBBBP = 0b10100001,
    PBPBBBPB = 0b10100010,
    PBPBBBPP = 0b10100011,
    PBPBBPBB = 0b10100100,
    PBPBBPBP = 0b10100101,
    PBPBBPPB = 0b10100110,
    PBPBBPPP = 0b10100111,
    PBPBPBBB = 0b10101000,
    PBPBPBBP = 0b10101001,
    PBPBPBPB = 0b10101010,
    PBPBPBPP = 0b10101011,
    PBPBPPBB = 0b10101100,
    PBPBPPBP = 0b10101101,
    PBPBPPPB = 0b10101110,
    PBPBPPPP = 0b10101111,
    PBPPBBBB = 0b10110000,
    PBPPBBBP = 0b10110001,
    PBPPBBPB = 0b10110010,
    PBPPBBPP = 0b10110011,
    PBPPBPBB = 0b10110100,
    PBPPBPBP = 0b10110101,
    PBPPBPPB = 0b10110110,
    PBPPBPPP = 0b10110111,
    PBPPPBBB = 0b10111000,
    PBPPPBBP = 0b10111001,
    PBPPPBPB = 0b10111010,
    PBPPPBPP = 0b10111011,
    PBPPPPBB = 0b10111100,
    PBPPPPBP = 0b10111101,
    PBPPPPPB = 0b10111110,
    PBPPPPPP = 0b10111111,
    PPBBBBBB = 0b11000000,
    PPBBBBBP = 0b11000001,
    PPBBBBPB = 0b11000010,
    PPBBBBPP = 0b11000011,
    PPBBBPBB = 0b11000100,
    PPBBBPBP = 0b11000101,
    PPBBBPPB = 0b11000110,
    PPBBBPPP = 0b11000111,
    PPBBPBBB = 0b11001000,
    PPBBPBBP = 0b11001001,
    PPBBPBPB = 0b11001010,
    PPBBPBPP = 0b11001011,
    PPBBPPBB = 0b11001100,
    PPBBPPBP = 0b11001101,
    PPBBPPPB = 0b11001110,
    PPBBPPPP = 0b11001111,
    PPBPBBBB = 0b11010000,
    PPBPBBBP = 0b11010001,
    PPBPBBPB = 0b11010010,
    PPBPBBPP = 0b11010011,
    PPBPBPBB = 0b11010100,
    PPBPBPBP = 0b11010101,
    PPBPBPPB = 0b11010110,
    PPBPBPPP = 0b11010111,
    PPBPPBBB = 0b11011000,
    PPBPPBBP = 0b11011001,
    PPBPPBPB = 0b11011010,
    PPBPPBPP = 0b11011011,
    PPBPPPBB = 0b11011100,
    PPBPPPBP = 0b11011101,
    PPBPPPPB = 0b11011110,
    PPBPPPPP = 0b11011111,
    PPPBBBBB = 0b11100000,
    PPPBBBBP = 0b11100001,
    PPPBBBPB = 0b11100010,
    PPPBBBPP = 0b11100011,
    PPPBBPBB = 0b11100100,
    PPPBBPBP = 0b11100101,
    PPPBBPPB = 0b11100110,
    PPPBBPPP = 0b11100111,
    PPPBPBBB = 0b11101000,
    PPPBPBBP = 0b11101001,
    PPPBPBPB = 0b11101010,
    PPPBPBPP = 0b11101011,
    PPPBPPBB = 0b11101100,
    PPPBPPBP = 0b11101101,
    PPPBPPPB = 0b11101110,
    PPPBPPPP = 0b11101111,
    PPPPBBBB = 0b11110000,
    PPPPBBBP = 0b11110001,
    PPPPBBPB = 0b11110010,
    PPPPBBPP = 0b11110011,
    PPPPBPBB = 0b11110100,
    PPPPBPBP = 0b11110101,
    PPPPBPPB = 0b11110110,
    PPPPBPPP = 0b11110111,
    PPPPPBBB = 0b11111000,
    PPPPPBBP = 0b11111001,
    PPPPPBPB = 0b11111010,
    PPPPPBPP = 0b11111011,
    PPPPPPBB = 0b11111100,
    PPPPPPBP = 0b11111101,
    PPPPPPPB = 0b11111110,
    PPPPPPPP = 0b11111111
};

/** Quantidade maxima de mudancas de padrao mantidas sem alocacao dinamica. */
constexpr uint8_t MAX_HISTORICO_LINHA = 32;

/** @brief Identifica um dos dois sensores TCS34725 da RoboS3. */
enum SensorCor : uint8_t
{
    COR_ESQUERDA = 0, ///< Sensor instalado no lado esquerdo.
    COR_DIREITA = 1   ///< Sensor instalado no lado direito.
};

/** @brief Seleciona um dos dois controladores I2C do ESP32-S3. */
enum BarramentoI2C : uint8_t
{
    BARRAMENTO_I2C0 = 0, ///< Controlador Wire, configurado por definePinsI2C0().
    BARRAMENTO_I2C1 = 1  ///< Controlador Wire1, configurado por definePinsI2C1().
};

/** @brief Resultado possivel da classificacao HSV. */
enum CorDetectada : uint8_t
{
    COR_DESCONHECIDA = 0, ///< Nenhuma faixa ativa correspondeu.
    COR_PRETA,            ///< Preto.
    COR_BRANCA,           ///< Branco.
    COR_VERMELHA,         ///< Vermelho.
    COR_VERDE,            ///< Verde.
    COR_AZUL,             ///< Azul.
    COR_AMARELA,          ///< Amarelo.
    COR_LARANJA,          ///< Laranja.
    COR_ROXA,             ///< Roxo.
    COR_TOTAL             ///< Sentinela com a quantidade de classificacoes.
};

/** @brief Ganhos analogicos suportados pelo TCS34725. */
enum GanhoCor : uint8_t
{
    GANHO_1X = 0, ///< Ganho 1x.
    GANHO_4X,     ///< Ganho 4x.
    GANHO_16X,    ///< Ganho 16x.
    GANHO_60X     ///< Ganho 60x.
};
/** @brief Tempos de integracao suportados pelo TCS34725. */
enum TempoIntegracaoCor : uint8_t
{
    INTEGRACAO_2_4MS = 0, ///< 2,4 ms.
    INTEGRACAO_24MS,      ///< 24 ms.
    INTEGRACAO_50MS,      ///< 50 ms.
    INTEGRACAO_101MS,     ///< 101 ms.
    INTEGRACAO_154MS,     ///< 154 ms.
    INTEGRACAO_600MS      ///< 600 ms.
};

/** @brief Severidade minima das mensagens internas de diagnostico. */
enum DebugLevel : uint8_t
{
    DEBUG_OFF = 0, ///< Desativa mensagens internas.
    DEBUG_ERROR,   ///< Exibe somente falhas.
    DEBUG_INFO,    ///< Exibe falhas e eventos principais.
    DEBUG_VERBOSE  ///< Exibe todos os detalhes disponiveis.
};

/** @brief Ajustes independentes de um motor para frente e para tras. */
struct CalibracaoMotor
{
    float compensacaoFrente; ///< Multiplicador do comando para frente.
    float compensacaoTras;   ///< Multiplicador do comando para tras.
    uint8_t minimoFrente;    ///< Menor comando util para frente, 0...100.
    uint8_t minimoTras;      ///< Menor comando util para tras, 0...100.
    bool invertido;          ///< Inverte a polaridade eletrica quando true.
};

/** @brief Cor RGB normalizada, com cada canal na faixa 0...255. */
struct RGB
{
    float r; ///< Canal vermelho.
    float g; ///< Canal verde.
    float b; ///< Canal azul.
};
/** @brief Cor HSV com H em graus e S/V em porcentagem. */
struct HSV
{
    float h; ///< Matiz de 0 a 360 graus.
    float s; ///< Saturacao de 0 a 100 por cento.
    float v; ///< Valor de 0 a 100 por cento.
};
/** @brief Cor HSL com H em graus e S/L em porcentagem. */
struct HSL
{
    float h; ///< Matiz de 0 a 360 graus.
    float s; ///< Saturacao de 0 a 100 por cento.
    float l; ///< Luminosidade de 0 a 100 por cento.
};

/** @brief Intervalo HSV usado para classificar uma cor. */
struct FaixaHSV
{
    float hMin; ///< Menor matiz; pode formar intervalo circular com hMax.
    float hMax; ///< Maior matiz; pode ser menor que hMin ao cruzar 360 graus.
    float sMin; ///< Saturacao minima em porcentagem.
    float sMax; ///< Saturacao maxima em porcentagem.
    float vMin; ///< Valor minimo em porcentagem.
    float vMax; ///< Valor maximo em porcentagem.
    bool ativa; ///< Define se a faixa participa da classificacao.
};

/** @brief Resultado completo de uma aquisicao do TCS34725. */
struct LeituraCor
{
    RGB rgb;               ///< Canais normalizados em 0...255.
    HSV hsv;               ///< Hue 0...360 e saturacao/valor 0...100.
    HSL hsl;               ///< Hue 0...360 e saturacao/luz 0...100.
    uint16_t rRaw;         ///< Canal vermelho cru entregue pelo sensor.
    uint16_t gRaw;         ///< Canal verde cru entregue pelo sensor.
    uint16_t bRaw;         ///< Canal azul cru entregue pelo sensor.
    uint16_t clear;        ///< Canal clear cru usado na normalizacao.
    CorDetectada cor;      ///< Classificacao pelas faixas configuradas.
    bool valida;           ///< false quando o sensor nao esta disponivel.
    uint32_t timestampMs;  ///< Instante da leitura segundo millis().
};

/** @brief Resultado completo derivado de uma unica aquisicao dos oito QTR. */
struct LeituraLinha
{
    uint16_t bruto[8];          ///< Valores fisicos na ordem S1...S8.
    uint16_t calibrado[8];      ///< Valores normalizados para 0...1000.
    uint8_t mascara;            ///< S1 no bit 7, S8 no bit 0; preto vale 1.
    PadraoLinha caso;           ///< Mesmo valor da mascara com nome P/B.
    char padrao[9];             ///< Texto P/B terminado em zero, como BBBPPBBB.
    uint8_t quantidadePretos;   ///< Quantidade de bits pretos na mascara.
    uint16_t posicao;           ///< Posicao ponderada de 0 a 7000; centro 3500.
    bool detectada;             ///< true quando existe posicao valida de linha.
    bool posicaoValida;         ///< true quando houve contraste suficiente.
    uint32_t timestampMs;       ///< Instante da leitura segundo millis().
    uint32_t tempoSemLinhaMs;   ///< Tempo desde a ultima deteccao valida.
    uint16_t ultimaPosicaoValida; ///< Posicao anterior usada durante perda.

    /** @brief Retorna true quando nenhum sensor esta sobre preto fisico. */
    bool semPreto() const
    {
        return mascara == 0;
    }
    /** @brief Retorna true quando os oito sensores estao sobre preto fisico. */
    bool todosPretos() const
    {
        return mascara == 0xFF;
    }
    /** @brief Testa preto em S1, S2 ou S3. */
    bool temPretoEsquerda() const
    {
        return (mascara & 0xE0) != 0;
    }
    /** @brief Testa preto em S4 ou S5. */
    bool temPretoCentro() const
    {
        return (mascara & 0x18) != 0;
    }
    /** @brief Testa preto em S6, S7 ou S8. */
    bool temPretoDireita() const
    {
        return (mascara & 0x07) != 0;
    }
};

/** @brief Aceleracao, giroscopio, temperatura e orientacao estimada. */
struct LeituraIMU
{
    float accX;            ///< Aceleracao no eixo X em metros por segundo ao quadrado.
    float accY;            ///< Aceleracao no eixo Y em metros por segundo ao quadrado.
    float accZ;            ///< Aceleracao no eixo Z em metros por segundo ao quadrado.
    float gyroX;           ///< Velocidade angular X em graus por segundo.
    float gyroY;           ///< Velocidade angular Y em graus por segundo.
    float gyroZ;           ///< Velocidade angular Z em graus por segundo.
    float roll;            ///< Inclinacao lateral estimada em graus.
    float pitch;           ///< Inclinacao longitudinal estimada em graus.
    float yawRelativo;     ///< Rotacao Z integrada e relativa em graus.
    float temperatura;     ///< Temperatura interna da IMU em graus Celsius.
    bool valida;           ///< false quando a IMU nao esta disponivel.
    uint32_t timestampMs;  ///< Instante da leitura segundo millis().
};

/** @brief Disponibilidade individual dos modulos apos begin(). */
struct StatusRobo
{
    bool motores;       ///< Ponte H configurada e pronta.
    bool linha;         ///< Conjunto QTR configurado e pronto.
    bool ultrassom;     ///< Driver ultrassonico configurado.
    bool i2c0;          ///< Barramento I2C0 iniciado.
    bool i2c1;          ///< Barramento I2C1 iniciado.
    bool corEsquerda;   ///< TCS34725 esquerdo detectado.
    bool corDireita;    ///< TCS34725 direito detectado.
    bool imu;           ///< MPU6050 detectada.

    /**
     * Ausencia de calibracao salva nao e falha de hardware. A biblioteca usa
     * valores padrao ate que o usuario execute e salve uma calibracao.
     */
    bool calibracaoCarregada; ///< Existe calibracao persistida valida e aplicada.

    /** @return true quando todo o hardware previsto respondeu ao begin(). */
    bool inicializacaoCompleta() const
    {
        return motores && linha && ultrassom && i2c0 && i2c1 && corEsquerda && corDireita && imu;
    }

    /** @return true quando motores e linha permitem um seguidor simples. */
    bool operacaoBasicaDisponivel() const
    {
        return motores && linha;
    }
};

/**
 * @brief Uma permanencia do robo em determinado padrao de linha.
 *
 * inicioMs usa millis(). duracaoMs informa quanto tempo o padrao permaneceu
 * ativo ate a mudanca seguinte ou ate a consulta mais recente.
 */
struct CasoLinha
{
    PadraoLinha padrao;  ///< Caso P/B observado.
    uint32_t inicioMs;   ///< Inicio da permanencia segundo millis().
    uint32_t duracaoMs;  ///< Duracao ate a mudanca ou consulta mais recente.
};
