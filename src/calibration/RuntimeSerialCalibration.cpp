#include "../RoboS3.h"
#include "CalibrationManager.h"
#include "SerialCalibration.h"
#include <new>

namespace robos3
{
/** @brief Estado alocado somente quando o console Serial e solicitado. */
struct RuntimeSerialCalibration
{
    explicit RuntimeSerialCalibration(RoboS3& robo) : manager(robo), interface(manager)
    {
    }

    CalibrationManager manager;
    SerialCalibration interface;
};
} // namespace robos3

bool RoboS3::iniciarCalibracaoSerial(Stream& stream)
{
    if (!begun_)
    {
        return false;
    }
    auto* runtime = static_cast<robos3::RuntimeSerialCalibration*>(interfacesCalibracao_[0]);
    if (!runtime)
    {
        runtime = new (std::nothrow) robos3::RuntimeSerialCalibration(*this);
        if (!runtime)
        {
            return false;
        }
    }
    runtime->interface.begin(stream);
    interfacesCalibracao_[0] = runtime;
    atualizadoresCalibracao_[0] = [](void* contexto)
    {
        auto* serial = static_cast<robos3::RuntimeSerialCalibration*>(contexto);
        serial->manager.update();
        serial->interface.update();
    };
    return true;
}
