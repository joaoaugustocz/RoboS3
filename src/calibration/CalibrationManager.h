#pragma once

#include <Arduino.h>
#include "../types/RoboTypes.h"

class RoboS3;

namespace robos3
{

/**
 * @brief Coordenador interno das rotinas de calibracao e teste de bancada.
 *
 * E compartilhado pelas interfaces Serial e Web quando elas sao habilitadas.
 */
class CalibrationManager
{
  public:
    /** @brief Associa o coordenador. @param robo Instancia principal controlada. */
    explicit CalibrationManager(RoboS3& robo);

    /** @return Instancia RoboS3 controlada por este coordenador. */
    RoboS3& robo()
    {
        return robo_;
    }
    /** @brief Processa o prazo de seguranca dos testes de motor. */
    void update();

    /**
     * @brief Aciona os motores temporariamente e agenda parada automatica.
     * @param left Comando esquerdo. @param right Comando direito.
     * @param timeoutMs Prazo de seguranca em milissegundos.
     */
    void testMotors(int left, int right, uint32_t timeoutMs = 1500);

    /** @brief Encerra um teste. @param brake true para frenagem ativa. */
    void stopMotors(bool brake = false);

    /** @return Resultado de RoboS3::salvarCalibracao(). */
    bool save();
    /** @return Resultado de RoboS3::carregarCalibracao(). */
    bool load();
    /** @return Resultado de RoboS3::apagarCalibracao(). */
    bool reset();
    /** @brief Calibra a linha. @param durationMs Duracao em milissegundos. */
    void calibrateLine(uint32_t durationMs);
    /** @brief Calibra a IMU. @param durationMs Duracao em milissegundos. */
    void calibrateImu(uint32_t durationMs);

  private:
    RoboS3& robo_;
    uint32_t motorDeadlineMs_;
    bool motorTestActive_;
};

} // namespace robos3
