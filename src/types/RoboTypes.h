#pragma once

#include <stdint.h>

enum Motor : uint8_t
{
    MOTOR_ESQUERDO = 0,
    MOTOR_DIREITO = 1
};
enum SentidoMotor : uint8_t
{
    FRENTE = 0,
    TRAS = 1
};
enum TipoQTR : uint8_t
{
    QTR_8A = 0,
    QTR_8RC = 1
};
enum TipoLinha : uint8_t
{
    LINHA_PRETA = 0,
    LINHA_BRANCA = 1
};
enum SensorCor : uint8_t
{
    COR_ESQUERDA = 0,
    COR_DIREITA = 1
};

enum CorDetectada : uint8_t
{
    COR_DESCONHECIDA = 0,
    COR_PRETA,
    COR_BRANCA,
    COR_VERMELHA,
    COR_VERDE,
    COR_AZUL,
    COR_AMARELA,
    COR_LARANJA,
    COR_ROXA,
    COR_TOTAL
};

enum GanhoCor : uint8_t
{
    GANHO_1X = 0,
    GANHO_4X,
    GANHO_16X,
    GANHO_60X
};
enum TempoIntegracaoCor : uint8_t
{
    INTEGRACAO_2_4MS = 0,
    INTEGRACAO_24MS,
    INTEGRACAO_50MS,
    INTEGRACAO_101MS,
    INTEGRACAO_154MS,
    INTEGRACAO_600MS
};

enum DebugLevel : uint8_t
{
    DEBUG_OFF = 0,
    DEBUG_ERROR,
    DEBUG_INFO,
    DEBUG_VERBOSE
};

struct CalibracaoMotor
{
    float compensacaoFrente;
    float compensacaoTras;
    uint8_t minimoFrente;
    uint8_t minimoTras;
    bool invertido;
};

struct RGB
{
    float r, g, b;
};
struct HSV
{
    float h, s, v;
};
struct HSL
{
    float h, s, l;
};

struct FaixaHSV
{
    float hMin, hMax;
    float sMin, sMax;
    float vMin, vMax;
    bool ativa;
};

struct LeituraCor
{
    RGB rgb;
    HSV hsv;
    HSL hsl;
    uint16_t rRaw, gRaw, bRaw, clear;
    CorDetectada cor;
    bool valida;
    uint32_t timestampMs;
};

struct LeituraLinha
{
    uint16_t bruto[8];
    uint16_t calibrado[8];
    uint8_t mascara;
    char padrao[9];
    uint8_t quantidadePretos;
    uint16_t posicao;
    bool detectada;
    bool posicaoValida;
    uint32_t timestampMs;
    uint32_t tempoSemLinhaMs;
    uint16_t ultimaPosicaoValida;

    bool semPreto() const
    {
        return mascara == 0;
    }
    bool todosPretos() const
    {
        return mascara == 0xFF;
    }
    bool temPretoEsquerda() const
    {
        return (mascara & 0xE0) != 0;
    }
    bool temPretoCentro() const
    {
        return (mascara & 0x18) != 0;
    }
    bool temPretoDireita() const
    {
        return (mascara & 0x07) != 0;
    }
};

struct LeituraIMU
{
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yawRelativo;
    float temperatura;
    bool valida;
    uint32_t timestampMs;
};

struct StatusRobo
{
    // Cada campo informa se aquele modulo foi iniciado e pode ser usado.
    bool motores;
    bool linha;
    bool ultrassom;
    bool i2c0;
    bool i2c1;
    bool corEsquerda;
    bool corDireita;
    bool imu;

    // Ausencia de calibracao salva nao e falha de hardware. A biblioteca usa
    // valores padrao ate que o usuario execute e salve uma calibracao.
    bool calibracaoCarregada;

    // Retorna true quando todo o hardware previsto na PCB respondeu ao begin().
    bool inicializacaoCompleta() const
    {
        return motores && linha && ultrassom && i2c0 && i2c1 && corEsquerda && corDireita && imu;
    }

    // Motores e linha formam o conjunto minimo para um seguidor de linha simples.
    bool operacaoBasicaDisponivel() const
    {
        return motores && linha;
    }
};
