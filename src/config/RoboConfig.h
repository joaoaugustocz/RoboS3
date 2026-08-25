#pragma once

#include "BoardConfig.h"

namespace robos3
{

struct RoboConfig
{
    BoardConfig board;
    TipoLinha tipoLinha;
    uint16_t limiarLinha;
    float kp;
    float ki;
    float kd;
    int16_t limitePID;
    CalibracaoMotor motorEsquerdo;
    CalibracaoMotor motorDireito;

    static RoboConfig padrao()
    {
        RoboConfig c{};
        c.board = BoardConfig::padrao();
        c.tipoLinha = LINHA_PRETA;
        c.limiarLinha = 500;
        c.kp = 0.02f;
        c.ki = 0.0f;
        c.kd = 0.05f;
        c.limitePID = 100;
        c.motorEsquerdo = {1.0f, 1.0f, 18, 18, false};
        c.motorDireito = {1.0f, 1.0f, 18, 18, false};
        return c;
    }
};

} // namespace robos3
