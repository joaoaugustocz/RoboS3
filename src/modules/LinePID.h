#pragma once

#include <PIDEasy.h>

namespace robos3
{

/**
 * @brief Controlador PID usado no seguimento de linha.
 *
 * Encapsula o PIDEasy e calcula automaticamente o intervalo entre chamadas.
 * A saida e limitada pelos valores definidos em configure().
 */
class LinePID
{
  public:
    /** @brief Cria um controlador com os ganhos padrao da biblioteca. */
    LinePID();

    /**
     * @brief Define ganhos e limites da correcao PID.
     * @param kp Ganho proporcional.
     * @param ki Ganho integral.
     * @param kd Ganho derivativo.
     * @param minimum Menor valor permitido para a saida.
     * @param maximum Maior valor permitido para a saida.
     */
    void configure(float kp, float ki, float kd, float minimum = -100, float maximum = 100);

    /** @brief Limpa os estados integral, derivativo e temporal do controlador. */
    void reset();

    /**
     * @brief Calcula uma nova correcao para o erro informado.
     * @param error Diferenca entre a posicao medida e a posicao desejada.
     * @return Correcao limitada ao intervalo configurado.
     */
    float compute(float error);

  private:
    PID pid_;
    float kp_, ki_, kd_, minimum_, maximum_;
    unsigned long lastMillis_;
};

} // namespace robos3
