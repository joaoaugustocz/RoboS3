#pragma once

#include <stdint.h>
#include "../config/BoardConfig.h"
#include "../types/RoboTypes.h"

namespace robos3
{

/** @brief Driver interno dos dois motores ligados a uma ponte H L298N. */
class MotorDriver
{
  public:
    /** @brief Cria o driver com comandos zerados e calibracao neutra. */
    MotorDriver();

    /**
     * @brief Configura GPIOs e PWM dos dois motores.
     * @param pins Pinagem eletrica da ponte H.
     * @param frequency Frequencia PWM em hertz.
     * @param resolution Resolucao PWM em bits.
     * @return true quando os canais foram configurados.
     */
    bool begin(const MotorPins& pins, uint32_t frequency, uint8_t resolution);

    /**
     * @brief Aplica comandos independentes aos motores.
     * @param left Comando esquerdo de -100 a 100.
     * @param right Comando direito de -100 a 100.
     */
    void move(int left, int right);

    /**
     * @brief Troca o canal fisico acionado pelos comandos esquerdo e direito.
     * @param swapped true quando os conectores dos motores estao trocados.
     */
    void setSwapped(bool swapped)
    {
        swapped_ = swapped;
    }

    /** @return true quando os canais fisicos esquerdo e direito estao trocados. */
    bool swapped() const
    {
        return swapped_;
    }

    /** @brief Desliga os enables e deixa os motores em parada livre. */
    void stop();

    /** @brief Aplica frenagem ativa aos dois motores. */
    void brake();

    /**
     * @brief Substitui a calibracao de um motor.
     * @param motor Motor selecionado. @param calibration Novos ajustes.
     */
    void setCalibration(Motor motor, const CalibracaoMotor& calibration);

    /** @param motor Motor selecionado. @return Referencia para sua calibracao. */
    const CalibracaoMotor& calibration(Motor motor) const;

    /** @return Ultimo comando solicitado ao motor esquerdo, de -100 a 100. */
    int leftCommand() const
    {
        return leftCommand_;
    }
    /** @return Ultimo comando solicitado ao motor direito, de -100 a 100. */
    int rightCommand() const
    {
        return rightCommand_;
    }
    /** @return Maior duty PWM representavel pela resolucao configurada. */
    uint16_t pwmMax() const
    {
        return pwmMax_;
    }

  private:
    void drive(bool physicalLeft, int command, const CalibracaoMotor& calibration);
    MotorPins pins_;
    CalibracaoMotor calibration_[2];
    uint16_t pwmMax_;
    int leftCommand_;
    int rightCommand_;
    bool begun_;
    bool swapped_;
};

} // namespace robos3
