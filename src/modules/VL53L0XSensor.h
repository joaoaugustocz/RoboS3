#pragma once

#include <VL53L0X.h>
#include <Wire.h>
#include <stdint.h>

namespace robos3
{

/** @brief Adaptador interno para a biblioteca VL53L0X da Pololu. */
class VL53L0XSensor
{
  public:
    /** @brief Cria um driver ainda nao associado a um barramento. */
    VL53L0XSensor();

    /**
     * @brief Inicializa o sensor e inicia medicoes continuas.
     * @param wire Barramento Wire ou Wire1 ja iniciado.
     * @param timeoutMs Limite de espera de cada leitura, em milissegundos.
     * @return true quando o VL53L0X respondeu e foi configurado.
     */
    bool begin(TwoWire& wire, uint16_t timeoutMs = 100);

    /**
     * @brief Le a ultima faixa em modo continuo.
     * @param timeout Recebe true quando a espera excede o limite configurado.
     * @return Distancia em milimetros; 0 quando indisponivel ou por timeout.
     */
    uint16_t distanceMm(bool& timeout);

    /** @return true quando begin() detectou e configurou o sensor. */
    bool available() const
    {
        return available_;
    }

  private:
    VL53L0X sensor_;
    bool available_;
};

} // namespace robos3
