#include "../RoboS3.h"
#include "CalibrationManager.h"
#include "WebCalibration.h"
#include <new>

namespace robos3
{
/** @brief Estado alocado somente quando o painel Web e solicitado. */
struct RuntimeWebCalibration
{
    explicit RuntimeWebCalibration(RoboS3& robo) : manager(robo), interface(manager)
    {
    }

    CalibrationManager manager;
    WebCalibration interface;
};
} // namespace robos3

bool RoboS3::iniciarCalibracaoWeb()
{
    if (!begun_)
    {
        return false;
    }
    auto* runtime = static_cast<robos3::RuntimeWebCalibration*>(interfacesCalibracao_[1]);
    if (!runtime)
    {
        runtime = new (std::nothrow) robos3::RuntimeWebCalibration(*this);
        if (!runtime)
        {
            return false;
        }
        if (!runtime->interface.begin())
        {
            delete runtime;
            return false;
        }
    }
    interfacesCalibracao_[1] = runtime;
    atualizadoresCalibracao_[1] = [](void* contexto)
    {
        auto* web = static_cast<robos3::RuntimeWebCalibration*>(contexto);
        web->manager.update();
        web->interface.update();
    };
    return true;
}
