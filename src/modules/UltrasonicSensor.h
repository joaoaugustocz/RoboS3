#pragma once

#include <HCSR04.h>
#include <stdint.h>

namespace robos3
{

/** @brief Driver interno para um sensor ultrassonico compativel com HC-SR04. */
class UltrasonicSensor
{
  public:
    /** @brief Cria o driver ainda nao associado a GPIOs. */
    UltrasonicSensor();

    /**
     * @brief Associa os GPIOs e prepara o sensor.
     * @param triggerPin GPIO conectado ao sinal TRIG.
     * @param echoPin GPIO conectado ao sinal ECHO.
     * @param maximumCm Maior distancia considerada, em centimetros.
     * @return true quando o driver foi configurado.
     */
    bool begin(uint8_t triggerPin, uint8_t echoPin, uint16_t maximumCm = 400);

    /**
     * @brief Realiza uma medicao de distancia.
     * @return Distancia em centimetros ou NAN quando nao ha medida valida.
     */
    float distanceCm();

    /**
     * @brief Verifica se existe obstaculo dentro do limite informado.
     * @param maximumCm Distancia limite em centimetros.
     * @return true somente para uma medida valida menor ou igual ao limite.
     */
    bool hasObstacle(float maximumCm);

    /** @return true depois que begin() configurou o driver. */
    bool available() const
    {
        return begun_;
    }

  private:
    alignas(UltraSonicDistanceSensor) uint8_t storage_[sizeof(UltraSonicDistanceSensor)];
    UltraSonicDistanceSensor* sensor_;
    bool begun_;
};

} // namespace robos3
