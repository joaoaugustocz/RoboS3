#pragma once

#include "BoardConfig.h"

namespace robos3
{

/** @brief Parametros logicos e eletricos usados pela interface RoboS3. */
struct RoboConfig
{
    BoardConfig board;                ///< Pinagem e parametros eletricos.
    TipoLinha tipoLinha;              ///< Linha preta ou branca.
    uint16_t limiarLinha;             ///< Limiar QTR calibrado de 0 a 1000.
    float kp;                         ///< Ganho proporcional do PID.
    float ki;                         ///< Ganho integral do PID.
    float kd;                         ///< Ganho derivativo do PID.
    int16_t limitePID;                ///< Limite absoluto da correcao.
    CalibracaoMotor motorEsquerdo;    ///< Ajustes do motor esquerdo.
    CalibracaoMotor motorDireito;     ///< Ajustes do motor direito.

    /** @return Configuracao recomendada para a PCB ESP_FB V1. */
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
