#pragma once

#include <WebServer.h>

namespace robos3
{
class CalibrationManager;

/** @brief Painel HTTP de calibracao servido por um ponto de acesso do ESP32. */
class WebCalibration
{
  public:
    /** @brief Associa o painel. @param manager Coordenador compartilhado. */
    explicit WebCalibration(CalibrationManager& manager);

    /** @return true quando o ponto de acesso e o servidor foram iniciados. */
    bool begin();

    /** @brief Processa requisicoes HTTP pendentes; chame continuamente. */
    void update();

    /** @return Nome do ponto de acesso criado pela biblioteca. */
    const char* ssid() const
    {
        return ssid_;
    }

  private:
    void routes();
    void root();
    void apiStatus();
    void apiLine();
    void apiColors();
    void apiImu();
    void apiMotors();
    void apiStop();
    void apiSave();
    void apiLoad();
    void apiReset();
    void apiMotorConfig();
    void apiColorConfig();
    void apiColorRange();
    CalibrationManager& manager_;
    WebServer server_;
    char ssid_[32];
};

} // namespace robos3
