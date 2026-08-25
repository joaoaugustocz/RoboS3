#if defined(ROBOS3_CALIBRATION_WEB) || defined(ROBOS3_CALIBRATION_SERIAL)
#include "CalibrationManager.h"
#include "../RoboS3.h"

namespace robos3
{

CalibrationManager::CalibrationManager(RoboS3& r)
    : robo_(r), motorDeadlineMs_(0), motorTestActive_(false)
{
}
void CalibrationManager::update()
{
    if (motorTestActive_ && static_cast<int32_t>(millis() - motorDeadlineMs_) >= 0)
    {
        stopMotors(true);
    }
}
void CalibrationManager::testMotors(int left, int right, uint32_t timeout)
{
    robo_.mover(left, right);
    motorDeadlineMs_ = millis() + (timeout > 2000 ? 2000 : timeout);
    motorTestActive_ = true;
}
void CalibrationManager::stopMotors(bool brake)
{
    brake ? robo_.frear() : robo_.parar();
    motorTestActive_ = false;
}
bool CalibrationManager::save()
{
    return robo_.salvarCalibracao();
}
bool CalibrationManager::load()
{
    return robo_.carregarCalibracao();
}
bool CalibrationManager::reset()
{
    stopMotors(true);
    return robo_.apagarCalibracao();
}
void CalibrationManager::calibrateLine(uint32_t ms)
{
    stopMotors(true);
    robo_.calibrarLinha(ms);
}
void CalibrationManager::calibrateImu(uint32_t ms)
{
    stopMotors(true);
    robo_.calibrarIMU(ms);
}

} // namespace robos3
#endif
