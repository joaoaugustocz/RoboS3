#pragma once

#ifdef ROBOS3_CALIBRATION_WEB
#include <WebServer.h>

namespace robos3
{
class CalibrationManager;

class WebCalibration
{
  public:
    explicit WebCalibration(CalibrationManager& manager);
    bool begin();
    void update();
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
#endif
