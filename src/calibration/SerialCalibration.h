#pragma once

#include <Arduino.h>

namespace robos3
{
class CalibrationManager;

/** @brief Console textual de calibracao acessivel por qualquer Stream Arduino. */
class SerialCalibration
{
  public:
    /** @brief Associa o console. @param manager Coordenador compartilhado. */
    explicit SerialCalibration(CalibrationManager& manager);

    /**
     * @brief Inicia a recepcao de comandos.
     * @param stream Porta de entrada e saida; Serial por padrao.
     */
    void begin(Stream& stream = Serial);

    /** @brief Le e executa comandos completos sem bloquear o loop principal. */
    void update();

  private:
    void execute(char* line);
    void help();
    CalibrationManager& manager_;
    Stream* stream_;
    char buffer_[128];
    uint8_t length_;
};

} // namespace robos3
