#include "RoboS3.h"
#include "modules/RoboMath.h"

#include <Wire.h>
#include <math.h>
#include <string.h>

using namespace robos3;

namespace
{
constexpr const char* PRINT_RULE =
    "======================================================================";
constexpr const char* PRINT_DIVIDER =
    "----------------------------------------------------------------------";

void printReportStart(Print& out, const __FlashStringHelper* title)
{
    out.println();
    out.println(PRINT_RULE);
    out.print(F("RoboS3 | "));
    out.println(title);
    out.println(PRINT_RULE);
}

void printReportEnd(Print& out)
{
    out.println(PRINT_RULE);
}

void printSection(Print& out, const __FlashStringHelper* title)
{
    out.println();
    out.print(F("[ "));
    out.print(title);
    out.println(F(" ]"));
    out.println(PRINT_DIVIDER);
}

void printModuleStatus(Print& out, const char* name, bool enabled, bool ready)
{
    out.print(F("  "));
    out.print(name);
    const size_t nameLength = strlen(name);
    for (size_t i = nameLength; i < 25; ++i)
    {
        out.print(' ');
    }
    out.print(F(" | "));
    out.println(!enabled ? F("DESATIVADO") : ready ? F("OK") : F("FALHA"));
}

void printVector8(Print& out, const char* label, const uint16_t values[8])
{
    out.print(F("  "));
    out.print(label);
    out.print(F(": ["));
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (i)
        {
            out.print(F(", "));
        }
        out.print(values[i]);
    }
    out.println(']');
}

const char* colorName(CorDetectada color)
{
    static const char* names[COR_TOTAL] = {
        "desconhecida", "preta", "branca", "vermelha", "verde",
        "azul", "amarela", "laranja", "roxa"};
    const uint8_t index = static_cast<uint8_t>(color);
    return index < COR_TOTAL ? names[index] : "invalida";
}

void printMotorCalibration(Print& out, const char* name, const CalibracaoMotor& motor,
                           uint8_t explicitFields)
{
    out.print(F("  Motor "));
    out.println(name);
    out.print(F("    Frente : compensacao="));
    out.print(motor.compensacaoFrente, 3);
    out.print(F(" | minimo="));
    out.print(motor.minimoFrente);
    out.println(F("%"));
    out.print(F("    Re     : compensacao="));
    out.print(motor.compensacaoTras, 3);
    out.print(F(" | minimo="));
    out.print(motor.minimoTras);
    out.println(F("%"));
    out.print(F("    Invertido: "));
    out.println(motor.invertido ? F("sim") : F("nao"));
    out.print(F("    Sobrescrito no sketch: "));
    if (explicitFields == 0)
    {
        out.println(F("nenhum"));
        return;
    }

    bool separator = false;
    const auto printOverride = [&out, &separator, explicitFields](uint8_t bit,
                                                                    const __FlashStringHelper* label)
    {
        if (explicitFields & bit)
        {
            if (separator)
            {
                out.print(F(", "));
            }
            out.print(label);
            separator = true;
        }
    };
    printOverride(AJUSTE_MOTOR_INVERSAO, F("inversao"));
    printOverride(AJUSTE_MOTOR_COMPENSACAO_FRENTE, F("compensacao frente"));
    printOverride(AJUSTE_MOTOR_COMPENSACAO_TRAS, F("compensacao re"));
    printOverride(AJUSTE_MOTOR_MINIMO_FRENTE, F("minimo frente"));
    printOverride(AJUSTE_MOTOR_MINIMO_TRAS, F("minimo re"));
    out.println();
}

template <typename BusConfig>
void printI2CBus(Print& out, const char* name, const BusConfig& bus, bool ready)
{
    out.print(F("  "));
    out.print(name);
    out.print(F(" | "));
    out.print(ready ? F("OK") : F("FALHA"));
    out.print(F(" | SDA="));
    out.print(bus.sda);
    out.print(F(" SCL="));
    out.print(bus.scl);
    out.print(F(" | "));
    out.print(bus.frequency / 1000);
    out.println(F(" kHz"));
}
} // namespace

RoboS3::RoboS3()
    : config_(RoboConfig::padrao()), status_{}, motors_{}, line_{}, pid_{},
      ultrasonic_{}, vl53_{}, colors_{}, imu_{}, storage_{}, buttons_{}, debug_{},
      interfacesCalibracao_{nullptr, nullptr}, atualizadoresCalibracao_{nullptr, nullptr},
      ajustesMotorExplicitos_{0, 0}, begun_(false), anguloZInvertido_(false),
      sensoresCorTrocados_(false), calibracaoAlterada_(false)
{
}

bool RoboS3::begin(Print& diagnostico)
{
    status_ = {};
    status_.motores = motors_.begin(config_.board.motors, config_.board.pwmFrequency,
                                    config_.board.pwmResolution);
    motors_.setCalibration(MOTOR_ESQUERDO, config_.motorEsquerdo);
    motors_.setCalibration(MOTOR_DIREITO, config_.motorDireito);
    motors_.stop();
    status_.linha =
        line_.begin(config_.board.qtrPins, config_.board.qtrType, config_.board.qtrEmitterPin);
    line_.setTypeLine(config_.tipoLinha);
    line_.setThreshold(config_.limiarLinha);
    status_.ultrassom1Habilitado = config_.board.ultrassom[0].habilitado;
    status_.ultrassom2Habilitado = config_.board.ultrassom[1].habilitado;
    status_.corEsquerdaHabilitada = corHabilitada(COR_ESQUERDA);
    status_.corDireitaHabilitada = corHabilitada(COR_DIREITA);
    status_.vl53_1Habilitado = config_.board.vl53[0].habilitado;
    status_.vl53_2Habilitado = config_.board.vl53[1].habilitado;
    if (status_.ultrassom1Habilitado)
    {
        const auto& ultra = config_.board.ultrassom[0];
        status_.ultrassom = ultrasonic_[0].begin(ultra.trigger, ultra.echo, ultra.maximoCm);
    }
    if (status_.ultrassom2Habilitado)
    {
        const auto& ultra = config_.board.ultrassom[1];
        status_.ultrassom2 = ultrasonic_[1].begin(ultra.trigger, ultra.echo, ultra.maximoCm);
    }
    status_.i2c0 =
        Wire.begin(config_.board.i2c0.sda, config_.board.i2c0.scl, config_.board.i2c0.frequency);
    status_.i2c1 =
        Wire1.begin(config_.board.i2c1.sda, config_.board.i2c1.scl, config_.board.i2c1.frequency);
    auto dispositivosNoEndereco29 = [this](BarramentoI2C barramento)
    {
        uint8_t quantidade = 0;
        quantidade += corHabilitada(COR_ESQUERDA) && barramentoCor(COR_ESQUERDA) == barramento;
        quantidade += corHabilitada(COR_DIREITA) && barramentoCor(COR_DIREITA) == barramento;
        quantidade += config_.board.vl53[0].habilitado &&
                      config_.board.vl53[0].barramento == barramento;
        quantidade += config_.board.vl53[1].habilitado &&
                      config_.board.vl53[1].barramento == barramento;
        return quantidade;
    };
    if (corHabilitada(COR_ESQUERDA) &&
        dispositivosNoEndereco29(barramentoCor(COR_ESQUERDA)) == 1)
    {
        status_.corEsquerda = i2cDisponivel(barramentoCor(COR_ESQUERDA)) &&
                              colors_[0].begin(wire(barramentoCor(COR_ESQUERDA)));
    }
    if (corHabilitada(COR_DIREITA) &&
        dispositivosNoEndereco29(barramentoCor(COR_DIREITA)) == 1)
    {
        status_.corDireita = i2cDisponivel(barramentoCor(COR_DIREITA)) &&
                            colors_[1].begin(wire(barramentoCor(COR_DIREITA)));
    }
    if (config_.board.vl53[0].habilitado &&
        dispositivosNoEndereco29(config_.board.vl53[0].barramento) == 1)
    {
        const auto& tof = config_.board.vl53[0];
        status_.vl53_1 = i2cDisponivel(tof.barramento) &&
                         vl53_[0].begin(wire(tof.barramento), tof.timeoutMs);
    }
    if (config_.board.vl53[1].habilitado &&
        dispositivosNoEndereco29(config_.board.vl53[1].barramento) == 1)
    {
        const auto& tof = config_.board.vl53[1];
        status_.vl53_2 = i2cDisponivel(tof.barramento) &&
                         vl53_[1].begin(wire(tof.barramento), tof.timeoutMs);
    }
    status_.imu = i2cDisponivel(config_.board.imuI2C) &&
                  imu_.begin(wire(config_.board.imuI2C), config_.board.mpuAddress);
    pid_.configure(config_.kp, config_.ki, config_.kd, -config_.limitePID, config_.limitePID);
    status_.calibracaoCarregada = carregarCalibracao();
    begun_ = true;
    printCalibracao(diagnostico);
#ifdef ROBOS3_CALIBRATION_SERIAL
    iniciarCalibracaoSerial(Serial);
#endif
#ifdef ROBOS3_CALIBRATION_WEB
    iniciarCalibracaoWeb();
#endif
    // A inicializacao de cada modulo e independente. Em caso de falha, begin()
    // ja informa o defeito no destino Print recebido (Serial por padrao).
    const bool complete = status_.inicializacaoCompleta();
    if (!complete)
    {
        printErrosInicializacao(diagnostico);
    }
    else if (debug_.level() >= DEBUG_INFO)
    {
        printStatus(diagnostico);
    }
    return complete;
}

void RoboS3::atualizar()
{
    buttons_.update();
    if (status_.imu)
    {
        imu_.update();
    }
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (interfacesCalibracao_[i] && atualizadoresCalibracao_[i])
        {
            atualizadoresCalibracao_[i](interfacesCalibracao_[i]);
        }
    }
}

void RoboS3::definePinsQTR(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint8_t s5, uint8_t s6,
                           uint8_t s7, uint8_t s8, uint8_t emitter)
{
    const uint8_t pins[8] = {s1, s2, s3, s4, s5, s6, s7, s8};
    memcpy(config_.board.qtrPins, pins, 8);
    config_.board.qtrEmitterPin = emitter;
}
void RoboS3::setTipoLinha(TipoLinha type)
{
    config_.tipoLinha = type;
    line_.setTypeLine(type);
}
void RoboS3::espelharSensoresLinha(bool espelhados)
{
    line_.setMirrored(espelhados);
}
void RoboS3::definePinsPonteH(uint8_t ena, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                              uint8_t enb)
{
    config_.board.motors = {ena, in1, in2, in3, in4, enb};
}
void RoboS3::definePinsUltrassom(uint8_t trigger, uint8_t echo)
{
    configurarUltrassom(DISTANCIA_ULTRASSOM_1, trigger, echo);
}
bool RoboS3::configurarUltrassom(SensorDistancia sensor, uint8_t trigger, uint8_t echo,
                                 uint16_t maximoCm)
{
    if (sensor != DISTANCIA_ULTRASSOM_1 && sensor != DISTANCIA_ULTRASSOM_2)
    {
        return false;
    }
    if (trigger == echo)
    {
        return false;
    }
    const uint8_t index = sensor == DISTANCIA_ULTRASSOM_2 ? 1 : 0;
    const uint16_t alcanceSeguro = maximoCm == 0 ? static_cast<uint16_t>(1) : maximoCm;
    config_.board.ultrassom[index] = {config_.board.ultrassom[index].id, true, trigger, echo,
                                      alcanceSeguro};
    return true;
}
bool RoboS3::habilitarUltrassom(SensorDistancia sensor, bool habilitado)
{
    if (sensor != DISTANCIA_ULTRASSOM_1 && sensor != DISTANCIA_ULTRASSOM_2)
    {
        return false;
    }
    config_.board.ultrassom[sensor == DISTANCIA_ULTRASSOM_2 ? 1 : 0].habilitado = habilitado;
    return true;
}
void RoboS3::definePinsI2C0(uint8_t sda, uint8_t scl, uint32_t frequency)
{
    config_.board.i2c0 = {sda, scl, frequency};
}
void RoboS3::definePinsI2C1(uint8_t sda, uint8_t scl, uint32_t frequency)
{
    config_.board.i2c1 = {sda, scl, frequency};
}
void RoboS3::definirBarramentoCor(SensorCor sensor, BarramentoI2C barramento)
{
    if (indiceCorFisica(sensor) == 1)
    {
        config_.board.corDireitaI2C = barramento;
    }
    else
    {
        config_.board.corEsquerdaI2C = barramento;
    }
}
void RoboS3::habilitarSensorCor(SensorCor sensor, bool habilitado)
{
    if (indiceCorFisica(sensor) == 1)
    {
        config_.board.corDireitaHabilitada = habilitado;
    }
    else
    {
        config_.board.corEsquerdaHabilitada = habilitado;
    }
}
bool RoboS3::configurarVL53L0X(SensorDistancia sensor, BarramentoI2C barramento,
                               uint16_t timeoutMs)
{
    if (sensor != DISTANCIA_VL53_1 && sensor != DISTANCIA_VL53_2)
    {
        return false;
    }
    const uint8_t index = sensor == DISTANCIA_VL53_2 ? 1 : 0;
    const uint16_t timeoutSeguro = timeoutMs == 0 ? static_cast<uint16_t>(1) : timeoutMs;
    config_.board.vl53[index] = {config_.board.vl53[index].id, true, barramento, timeoutSeguro};
    return true;
}
bool RoboS3::desabilitarVL53L0X(SensorDistancia sensor)
{
    if (sensor != DISTANCIA_VL53_1 && sensor != DISTANCIA_VL53_2)
    {
        return false;
    }
    config_.board.vl53[sensor == DISTANCIA_VL53_2 ? 1 : 0].habilitado = false;
    return true;
}
bool RoboS3::adicionarSensorDistancia(uint8_t id, TipoSensorDistancia tipo, uint8_t trigger,
                                      uint8_t echo, uint16_t maximoCm)
{
    if (tipo != SENSOR_ULTRASSONICO || id == DISTANCIA_DESCONHECIDA || trigger == echo)
    {
        return false;
    }

    int8_t slot = -1;
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (config_.board.ultrassom[i].trigger == trigger && config_.board.ultrassom[i].echo == echo)
        {
            slot = i;
            break;
        }
        if (slot < 0 && !config_.board.ultrassom[i].habilitado)
        {
            slot = i;
        }
    }
    if (slot < 0)
    {
        return false;
    }
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (i != slot && config_.board.ultrassom[i].habilitado &&
            config_.board.ultrassom[i].id == id)
        {
            return false;
        }
        if (config_.board.vl53[i].habilitado && config_.board.vl53[i].id == id)
        {
            return false;
        }
    }
    const uint16_t alcanceSeguro = maximoCm == 0 ? static_cast<uint16_t>(1) : maximoCm;
    config_.board.ultrassom[slot] = {id, true, trigger, echo, alcanceSeguro};
    return true;
}
bool RoboS3::adicionarSensorDistancia(uint8_t id, TipoSensorDistancia tipo,
                                      BarramentoI2C barramento, uint16_t timeoutMs)
{
    if (tipo != SENSOR_TOF_VL53L0X || id == DISTANCIA_DESCONHECIDA)
    {
        return false;
    }

    int8_t slot = -1;
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (!config_.board.vl53[i].habilitado)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        return false;
    }
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (config_.board.ultrassom[i].habilitado && config_.board.ultrassom[i].id == id)
        {
            return false;
        }
        if (config_.board.vl53[i].habilitado && config_.board.vl53[i].id == id)
        {
            return false;
        }
    }
    const uint16_t timeoutSeguro = timeoutMs == 0 ? static_cast<uint16_t>(1) : timeoutMs;
    config_.board.vl53[slot] = {id, true, barramento, timeoutSeguro};
    return true;
}
bool RoboS3::removerSensorDistancia(uint8_t id)
{
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (config_.board.ultrassom[i].habilitado && config_.board.ultrassom[i].id == id)
        {
            config_.board.ultrassom[i].habilitado = false;
            return true;
        }
        if (config_.board.vl53[i].habilitado && config_.board.vl53[i].id == id)
        {
            config_.board.vl53[i].habilitado = false;
            return true;
        }
    }
    return false;
}
bool RoboS3::adicionarBotao(uint8_t id, uint8_t pino, uint16_t debounceMs)
{
    return buttons_.addDigital(id, pino, debounceMs);
}
bool RoboS3::adicionarBotaoAnalogico(uint8_t id, uint8_t pino, uint16_t valorMinimo,
                                     uint16_t valorMaximo, uint16_t debounceMs)
{
    return buttons_.addAnalog(id, pino, valorMinimo, valorMaximo, debounceMs);
}
void RoboS3::atualizarBotoes()
{
    buttons_.update();
}
LeituraBotao RoboS3::lerBotao(uint8_t id)
{
    return buttons_.read(id);
}
bool RoboS3::botaoPressionado(uint8_t id)
{
    return buttons_.pressed(id);
}
bool RoboS3::botaoFoiApertado(uint8_t id)
{
    return buttons_.consumePressed(id);
}
bool RoboS3::botaoFoiSolto(uint8_t id)
{
    return buttons_.consumeReleased(id);
}
bool RoboS3::botaoPressionadoPor(uint8_t id, uint32_t tempoMs)
{
    return buttons_.heldFor(id, tempoMs);
}
bool RoboS3::botaoDisponivel(uint8_t id) const
{
    return buttons_.available(id);
}
bool RoboS3::removerBotao(uint8_t id)
{
    return buttons_.remove(id);
}
void RoboS3::limparBotoes()
{
    buttons_.clear();
}
void RoboS3::definirBarramentoIMU(BarramentoI2C barramento)
{
    config_.board.imuI2C = barramento;
}

void RoboS3::mover(int left, int right)
{
    motors_.move(left, right);
}
void RoboS3::frente(int speed)
{
    mover(abs(speed), abs(speed));
}
void RoboS3::tras(int speed)
{
    mover(-abs(speed), -abs(speed));
}
void RoboS3::girarEsquerda(int speed)
{
    mover(-abs(speed), abs(speed));
}
void RoboS3::girarDireita(int speed)
{
    mover(abs(speed), -abs(speed));
}
bool RoboS3::girarAngulo(int speed, float angleZ, uint32_t timeoutMs)
{
    parar();
    if (!status_.motores || !status_.imu || speed == 0 || timeoutMs == 0 || !isfinite(angleZ))
    {
        return false;
    }

    const float target = fabsf(angleZ);
    if (target < 0.1f)
    {
        return true;
    }

    // Atualiza o instante interno antes de zerar para que uma pausa longa entre
    // leituras nao seja incorporada ao primeiro passo da manobra.
    imu_.read();
    imu_.zeroYaw();

    const bool turnRight = angleZ > 0;
    if (turnRight)
    {
        girarDireita(abs(speed));
    }
    else
    {
        girarEsquerda(abs(speed));
    }

    const uint32_t startMs = millis();
    bool reached = false;
    while (millis() - startMs < timeoutMs)
    {
        const LeituraIMU reading = imu_.read();
        float measured = reading.yawRelativo;
        if (anguloZInvertido_)
        {
            measured = -measured;
        }

        const float progress = turnRight ? measured : -measured;
        if (progress >= target)
        {
            reached = true;
            break;
        }
        delay(2);
    }

    frear();
    delay(30);
    parar();
    return reached;
}
void RoboS3::motorEsquerdo(int speed)
{
    mover(speed, motors_.rightCommand());
}
void RoboS3::motorDireito(int speed)
{
    mover(motors_.leftCommand(), speed);
}
void RoboS3::parar()
{
    motors_.stop();
}
void RoboS3::frear()
{
    motors_.brake();
}

void RoboS3::inverterMotorEsquerdo(bool value)
{
    auto c = motors_.calibration(MOTOR_ESQUERDO);
    c.invertido = value;
    config_.motorEsquerdo = c;
    motors_.setCalibration(MOTOR_ESQUERDO, c);
    ajustesMotorExplicitos_[MOTOR_ESQUERDO] |= AJUSTE_MOTOR_INVERSAO;
    calibracaoAlterada_ = true;
}
void RoboS3::inverterMotorDireito(bool value)
{
    auto c = motors_.calibration(MOTOR_DIREITO);
    c.invertido = value;
    config_.motorDireito = c;
    motors_.setCalibration(MOTOR_DIREITO, c);
    ajustesMotorExplicitos_[MOTOR_DIREITO] |= AJUSTE_MOTOR_INVERSAO;
    calibracaoAlterada_ = true;
}
void RoboS3::inverterMotores(bool left, bool right)
{
    inverterMotorEsquerdo(left);
    inverterMotorDireito(right);
}
void RoboS3::trocarMotores(bool trocados)
{
    motors_.stop();
    motors_.setSwapped(trocados);
}
void RoboS3::configurarCompensacaoMotor(Motor motor, SentidoMotor direction, float factor)
{
    const Motor motorNormalizado = motor == MOTOR_DIREITO ? MOTOR_DIREITO : MOTOR_ESQUERDO;
    auto c = motors_.calibration(motorNormalizado);
    if (direction == FRENTE)
    {
        c.compensacaoFrente = factor;
        ajustesMotorExplicitos_[motorNormalizado] |= AJUSTE_MOTOR_COMPENSACAO_FRENTE;
    }
    else
    {
        c.compensacaoTras = factor;
        ajustesMotorExplicitos_[motorNormalizado] |= AJUSTE_MOTOR_COMPENSACAO_TRAS;
    }
    if (motorNormalizado == MOTOR_ESQUERDO)
    {
        config_.motorEsquerdo = c;
    }
    else
    {
        config_.motorDireito = c;
    }
    motors_.setCalibration(motorNormalizado, c);
    calibracaoAlterada_ = true;
}
void RoboS3::configurarMinimoMovimento(Motor motor, SentidoMotor direction, uint8_t minimum)
{
    const Motor motorNormalizado = motor == MOTOR_DIREITO ? MOTOR_DIREITO : MOTOR_ESQUERDO;
    auto c = motors_.calibration(motorNormalizado);
    minimum = minimum > 100 ? 100 : minimum;
    if (direction == FRENTE)
    {
        c.minimoFrente = minimum;
        ajustesMotorExplicitos_[motorNormalizado] |= AJUSTE_MOTOR_MINIMO_FRENTE;
    }
    else
    {
        c.minimoTras = minimum;
        ajustesMotorExplicitos_[motorNormalizado] |= AJUSTE_MOTOR_MINIMO_TRAS;
    }
    if (motorNormalizado == MOTOR_ESQUERDO)
    {
        config_.motorEsquerdo = c;
    }
    else
    {
        config_.motorDireito = c;
    }
    motors_.setCalibration(motorNormalizado, c);
    calibracaoAlterada_ = true;
}

LeituraLinha RoboS3::lerLinha()
{
    return line_.read();
}
void RoboS3::lerLinhaBruta(uint16_t out[8])
{
    line_.readRaw(out);
}
void RoboS3::lerLinhaCalibrada(uint16_t out[8])
{
    line_.readCalibrated(out);
}
uint8_t RoboS3::lerLinhaBinaria()
{
    return line_.read().mascara;
}
PadraoLinha RoboS3::lerPadraoLinha()
{
    return line_.read().caso;
}
const LeituraLinha& RoboS3::ultimaLeituraLinha() const
{
    return line_.last();
}
uint32_t RoboS3::tempoSemLinha() const
{
    return line_.timeWithoutLine();
}
void RoboS3::configurarHistoricoLinha(uint8_t tamanho)
{
    line_.setHistoryCapacity(tamanho);
}
void RoboS3::limparHistoricoLinha()
{
    line_.clearHistory();
}
uint8_t RoboS3::capacidadeHistoricoLinha() const
{
    return line_.historyCapacity();
}
uint8_t RoboS3::quantidadeHistoricoLinha() const
{
    return line_.historyCount();
}
uint8_t RoboS3::copiarUltimosCasosLinha(PadraoLinha saida[], uint8_t maximo) const
{
    return line_.copyRecentPatterns(saida, maximo);
}
uint8_t RoboS3::copiarHistoricoLinha(CasoLinha saida[], uint8_t maximo) const
{
    return line_.copyHistory(saida, maximo);
}
void RoboS3::padraoLinhaTexto(PadraoLinha padrao, char saida[9])
{
    linePattern(static_cast<uint8_t>(padrao), saida);
}
bool RoboS3::calibrarLinha(uint32_t ms)
{
    parar();
    const bool calibrada = line_.calibrateMs(ms);
    calibracaoAlterada_ = calibracaoAlterada_ || calibrada;
    return calibrada;
}
bool RoboS3::calibrarLinhaAmostras(uint16_t samples)
{
    parar();
    const bool calibrada = line_.calibrateSamples(samples);
    calibracaoAlterada_ = calibracaoAlterada_ || calibrada;
    return calibrada;
}
void RoboS3::configurarLimiarLinha(uint16_t threshold)
{
    config_.limiarLinha = threshold;
    line_.setThreshold(threshold);
    calibracaoAlterada_ = true;
}
int16_t RoboS3::erroLinha(int16_t center) const
{
    return lineError(line_.last().posicao, center);
}
void RoboS3::configurarPID(float kp, float ki, float kd)
{
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
    pid_.configure(kp, ki, kd, -config_.limitePID, config_.limitePID);
}
void RoboS3::resetPID()
{
    pid_.reset();
}
float RoboS3::calcularPID(float error)
{
    return pid_.compute(error);
}
void RoboS3::seguirLinha(int base)
{
    LeituraLinha reading = lerLinha();
    if (!reading.detectada)
    {
        parar();
        return;
    }
    float correction = calcularPID(static_cast<int>(reading.posicao) - 3500);
    mover(base + correction, base - correction);
}

float RoboS3::distanciaCM()
{
    return distanciaCM(DISTANCIA_ULTRASSOM_1);
}
LeituraDistancia RoboS3::lerDistancia(SensorDistancia sensor)
{
    LeituraDistancia leitura{};
    if (sensor == DISTANCIA_ULTRASSOM_1 || sensor == DISTANCIA_ULTRASSOM_2)
    {
        leitura.id = config_.board.ultrassom[sensor == DISTANCIA_ULTRASSOM_2 ? 1 : 0].id;
    }
    else if (sensor == DISTANCIA_VL53_1 || sensor == DISTANCIA_VL53_2)
    {
        leitura.id = config_.board.vl53[sensor == DISTANCIA_VL53_2 ? 1 : 0].id;
    }
    leitura.sensor = sensor;
    if (sensor == DISTANCIA_VL53_1 || sensor == DISTANCIA_VL53_2)
    {
        leitura.tipo = SENSOR_TOF_VL53L0X;
    }
    else if (sensor == DISTANCIA_ULTRASSOM_1 || sensor == DISTANCIA_ULTRASSOM_2)
    {
        leitura.tipo = SENSOR_ULTRASSONICO;
    }
    else
    {
        leitura.tipo = SENSOR_DISTANCIA_DESCONHECIDO;
    }
    leitura.centimetros = NAN;
    leitura.timestampMs = millis();

    if (sensor == DISTANCIA_ULTRASSOM_1 || sensor == DISTANCIA_ULTRASSOM_2)
    {
        const uint8_t index = sensor == DISTANCIA_ULTRASSOM_2 ? 1 : 0;
        if (!distanciaDisponivel(sensor))
        {
            return leitura;
        }
        leitura.centimetros = ultrasonic_[index].distanceCm();
        leitura.timestampMs = ultrasonic_[index].lastMeasurementMs();
        leitura.valida = isfinite(leitura.centimetros);
        if (leitura.valida)
        {
            const float mm = leitura.centimetros * 10.0f;
            leitura.milimetros = mm >= UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(mm + 0.5f);
        }
        return leitura;
    }

    if (sensor == DISTANCIA_VL53_1 || sensor == DISTANCIA_VL53_2)
    {
        const uint8_t index = sensor == DISTANCIA_VL53_2 ? 1 : 0;
        if (!distanciaDisponivel(sensor))
        {
            return leitura;
        }
        leitura.milimetros = vl53_[index].distanceMm(leitura.timeout);
        leitura.valida = !leitura.timeout && leitura.milimetros > 0;
        if (leitura.valida)
        {
            leitura.centimetros = leitura.milimetros / 10.0f;
        }
    }
    return leitura;
}
LeituraDistancia RoboS3::lerDistancia(uint8_t id)
{
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (config_.board.ultrassom[i].habilitado && config_.board.ultrassom[i].id == id)
        {
            return lerDistancia(i == 0 ? DISTANCIA_ULTRASSOM_1 : DISTANCIA_ULTRASSOM_2);
        }
        if (config_.board.vl53[i].habilitado && config_.board.vl53[i].id == id)
        {
            return lerDistancia(i == 0 ? DISTANCIA_VL53_1 : DISTANCIA_VL53_2);
        }
    }
    LeituraDistancia leitura{};
    leitura.id = id;
    leitura.sensor = DISTANCIA_DESCONHECIDA;
    leitura.tipo = SENSOR_DISTANCIA_DESCONHECIDO;
    leitura.centimetros = NAN;
    leitura.timestampMs = millis();
    return leitura;
}
float RoboS3::distanciaCM(SensorDistancia sensor)
{
    return lerDistancia(sensor).centimetros;
}
float RoboS3::distanciaCM(uint8_t id)
{
    return lerDistancia(id).centimetros;
}
bool RoboS3::temObstaculo(float cm)
{
    return temObstaculo(DISTANCIA_ULTRASSOM_1, cm);
}
bool RoboS3::temObstaculo(SensorDistancia sensor, float cm)
{
    const LeituraDistancia leitura = lerDistancia(sensor);
    return leitura.valida && leitura.centimetros <= cm;
}
bool RoboS3::temObstaculo(uint8_t id, float cm)
{
    const LeituraDistancia leitura = lerDistancia(id);
    return leitura.valida && leitura.centimetros <= cm;
}
ColorSensor& RoboS3::color(SensorCor s)
{
    return colors_[s == COR_DIREITA ? 1 : 0];
}
const ColorSensor& RoboS3::color(SensorCor s) const
{
    return colors_[s == COR_DIREITA ? 1 : 0];
}
TwoWire& RoboS3::wire(BarramentoI2C barramento)
{
    return barramento == BARRAMENTO_I2C1 ? Wire1 : Wire;
}
bool RoboS3::i2cDisponivel(BarramentoI2C barramento) const
{
    return barramento == BARRAMENTO_I2C1 ? status_.i2c1 : status_.i2c0;
}
uint8_t RoboS3::indiceCorFisica(SensorCor sensor) const
{
    const uint8_t indiceLogico = sensor == COR_DIREITA ? 1 : 0;
    return sensoresCorTrocados_ ? indiceLogico ^ 1U : indiceLogico;
}
BarramentoI2C RoboS3::barramentoCor(SensorCor sensor) const
{
    return indiceCorFisica(sensor) == 1 ? config_.board.corDireitaI2C
                                        : config_.board.corEsquerdaI2C;
}
bool RoboS3::corHabilitada(SensorCor sensor) const
{
    return indiceCorFisica(sensor) == 1 ? config_.board.corDireitaHabilitada
                                        : config_.board.corEsquerdaHabilitada;
}
LeituraCor RoboS3::lerDadosCor(SensorCor s)
{
    return color(s).read();
}
RGB RoboS3::lerRGB(SensorCor s)
{
    return lerDadosCor(s).rgb;
}
HSV RoboS3::lerHSV(SensorCor s)
{
    return lerDadosCor(s).hsv;
}
HSL RoboS3::lerHSL(SensorCor s)
{
    return lerDadosCor(s).hsl;
}
CorDetectada RoboS3::lerCor(SensorCor s)
{
    return lerDadosCor(s).cor;
}
void RoboS3::definirFaixaH(SensorCor s, CorDetectada c, float hMin, float hMax)
{
    FaixaHSV r = color(s).range(c);
    r.hMin = hMin;
    r.hMax = hMax;
    r.ativa = true;
    color(s).setRange(c, r);
    calibracaoAlterada_ = true;
}
void RoboS3::definirFaixaHSV(SensorCor s, CorDetectada c, float hMin, float hMax, float sMin,
                             float sMax, float vMin, float vMax)
{
    color(s).setRange(c, {hMin, hMax, sMin, sMax, vMin, vMax, true});
    calibracaoAlterada_ = true;
}
void RoboS3::calibrarCor(SensorCor s, CorDetectada c, uint16_t samples)
{
    if (!corDisponivel(s) || samples == 0)
    {
        return;
    }
    float hMin = 360, hMax = 0, lowMax = 0, highMin = 360, sMin = 100, sMax = 0, vMin = 100,
          vMax = 0;
    for (uint16_t i = 0; i < samples; ++i)
    {
        HSV h = lerHSV(s);
        hMin = fminf(hMin, h.h);
        hMax = fmaxf(hMax, h.h);
        if (h.h <= 180)
        {
            lowMax = fmaxf(lowMax, h.h);
        }
        else
        {
            highMin = fminf(highMin, h.h);
        }
        sMin = fminf(sMin, h.s);
        sMax = fmaxf(sMax, h.s);
        vMin = fminf(vMin, h.v);
        vMax = fmaxf(vMax, h.v);
        delay(5);
    }
    if (hMax - hMin > 180 && highMin < 360)
    {
        hMin = fmaxf(0, highMin - 5);
        hMax = fminf(360, lowMax + 5);
    }
    else
    {
        hMin = fmaxf(0, hMin - 5);
        hMax = fminf(360, hMax + 5);
    }
    definirFaixaHSV(s, c, hMin, hMax, fmaxf(0, sMin - 5), fminf(100, sMax + 5), fmaxf(0, vMin - 5),
                    fminf(100, vMax + 5));
}
void RoboS3::setGanhoCor(SensorCor s, GanhoCor gain)
{
    color(s).setGain(gain);
    calibracaoAlterada_ = true;
}
void RoboS3::setTempoIntegracaoCor(SensorCor s, TempoIntegracaoCor time)
{
    color(s).setIntegrationTime(time);
    calibracaoAlterada_ = true;
}
bool RoboS3::trocarSensoresCor(bool trocados)
{
    if (begun_)
    {
        return false;
    }
    sensoresCorTrocados_ = trocados;
    return true;
}

LeituraIMU RoboS3::lerIMU()
{
    return imu_.read();
}
void RoboS3::atualizarIMU()
{
    imu_.update();
}
float RoboS3::roll() const
{
    return imu_.last().roll;
}
float RoboS3::pitch() const
{
    return imu_.last().pitch;
}
float RoboS3::anguloZ() const
{
    return imu_.last().yawRelativo;
}
void RoboS3::zerarAnguloZ()
{
    imu_.zeroYaw();
}
bool RoboS3::estaInclinado(float degrees) const
{
    return fabsf(pitch()) >= fabsf(degrees) || fabsf(roll()) >= fabsf(degrees);
}
void RoboS3::calibrarIMU(uint32_t ms)
{
    parar();
    imu_.calibrate(ms);
    calibracaoAlterada_ = calibracaoAlterada_ || status_.imu;
}

RoboCalibration RoboS3::makeCalibration() const
{
    RoboCalibration d{};
    line_.getCalibration(d.lineMinimum, d.lineMaximum);
    d.lineThreshold = line_.threshold();
    d.leftMotor = motors_.calibration(MOTOR_ESQUERDO);
    d.rightMotor = motors_.calibration(MOTOR_DIREITO);
    for (uint8_t s = 0; s < 2; ++s)
    {
        for (uint8_t c = 0; c < COR_TOTAL; ++c)
        {
            d.colorRanges[s][c] = colors_[s].range(static_cast<CorDetectada>(c));
        }
        d.colorGain[s] = colors_[s].gain();
        d.colorIntegration[s] = colors_[s].integrationTime();
    }
    d.imuOffsets = imu_.offsets();
    return d;
}
void RoboS3::applyCalibration(const RoboCalibration& d)
{
    line_.setCalibration(d.lineMinimum, d.lineMaximum);
    configurarLimiarLinha(d.lineThreshold);
    const CalibracaoMotor motorEsquerdo = mesclarCalibracaoMotor(
        d.leftMotor, config_.motorEsquerdo, ajustesMotorExplicitos_[MOTOR_ESQUERDO]);
    const CalibracaoMotor motorDireito = mesclarCalibracaoMotor(
        d.rightMotor, config_.motorDireito, ajustesMotorExplicitos_[MOTOR_DIREITO]);
    motors_.setCalibration(MOTOR_ESQUERDO, motorEsquerdo);
    motors_.setCalibration(MOTOR_DIREITO, motorDireito);
    config_.motorEsquerdo = motorEsquerdo;
    config_.motorDireito = motorDireito;
    for (uint8_t s = 0; s < 2; ++s)
    {
        for (uint8_t c = 0; c < COR_TOTAL; ++c)
        {
            colors_[s].setRange(static_cast<CorDetectada>(c), d.colorRanges[s][c]);
        }
        colors_[s].setGain(static_cast<GanhoCor>(d.colorGain[s]));
        colors_[s].setIntegrationTime(static_cast<TempoIntegracaoCor>(d.colorIntegration[s]));
    }
    imu_.setOffsets(d.imuOffsets);
}
bool RoboS3::salvarCalibracao()
{
    const bool salvou = storage_.save(makeCalibration());
    if (salvou)
    {
        status_.calibracaoCarregada = true;
        calibracaoAlterada_ = false;
        ajustesMotorExplicitos_[MOTOR_ESQUERDO] = 0;
        ajustesMotorExplicitos_[MOTOR_DIREITO] = 0;
    }
    return salvou;
}
bool RoboS3::carregarCalibracao()
{
    RoboCalibration d{};
    if (!storage_.load(d))
    {
        status_.calibracaoCarregada = false;
        return false;
    }
    if (d.schema == CALIBRATION_SCHEMA_QTR_10_BITS && config_.board.qtrType == QTR_8A)
    {
        // Ate o schema 1, o backend analogico era limitado a 0...1023 pela
        // QTRSensors. Converte somente extremos validos para preservar a NVS.
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (d.lineMinimum[i] < d.lineMaximum[i] && d.lineMaximum[i] <= 1023)
            {
                d.lineMinimum[i] = static_cast<uint16_t>(
                    (static_cast<uint32_t>(d.lineMinimum[i]) * 4095U + 511U) / 1023U);
                d.lineMaximum[i] = static_cast<uint16_t>(
                    (static_cast<uint32_t>(d.lineMaximum[i]) * 4095U + 511U) / 1023U);
            }
        }
        d.schema = CALIBRATION_SCHEMA;
    }
    applyCalibration(d);
    status_.calibracaoCarregada = true;
    calibracaoAlterada_ = ajustesMotorExplicitos_[MOTOR_ESQUERDO] != 0 ||
                          ajustesMotorExplicitos_[MOTOR_DIREITO] != 0;
    return true;
}
bool RoboS3::apagarCalibracao()
{
    const bool apagouNvs = storage_.erase();
    const RoboConfig padrao = RoboConfig::padrao();
    line_.resetCalibration();
    configurarLimiarLinha(padrao.limiarLinha);
    config_.motorEsquerdo = padrao.motorEsquerdo;
    config_.motorDireito = padrao.motorDireito;
    motors_.setCalibration(MOTOR_ESQUERDO, padrao.motorEsquerdo);
    motors_.setCalibration(MOTOR_DIREITO, padrao.motorDireito);
    ajustesMotorExplicitos_[MOTOR_ESQUERDO] = 0;
    ajustesMotorExplicitos_[MOTOR_DIREITO] = 0;
    colors_[0].resetConfiguration();
    colors_[1].resetConfiguration();
    imu_.setOffsets({});
    imu_.zeroYaw();
    status_.calibracaoCarregada = false;
    calibracaoAlterada_ = !apagouNvs;
    return apagouNvs;
}
bool RoboS3::corDisponivel(SensorCor s) const
{
    return s == COR_ESQUERDA ? status_.corEsquerda : status_.corDireita;
}

void RoboS3::printStatus(Print& out) const
{
    printReportStart(out, F("STATUS DO ROBO"));
    out.print(F("Resultado da inicializacao: "));
    out.println(status_.inicializacaoCompleta() ? F("COMPLETA") : F("COM FALHAS"));
    out.print(F("Calibracao aplicada: "));
    out.println(status_.calibracaoCarregada ? F("NVS") : F("PADRAO"));

    printSection(out, F("MODULOS"));
    out.println(F("  MODULO                    | ESTADO"));
    printModuleStatus(out, "Motores", true, status_.motores);
    printModuleStatus(out, "Sensores QTR", true, status_.linha);
    printModuleStatus(out, "Ultrassom 1", status_.ultrassom1Habilitado, status_.ultrassom);
    printModuleStatus(out, "Ultrassom 2", status_.ultrassom2Habilitado, status_.ultrassom2);
    printModuleStatus(out, "Sensor de cor esquerdo", status_.corEsquerdaHabilitada,
                      status_.corEsquerda);
    printModuleStatus(out, "Sensor de cor direito", status_.corDireitaHabilitada,
                      status_.corDireita);
    printModuleStatus(out, "MPU6050", true, status_.imu);
    printModuleStatus(out, "VL53L0X 1", status_.vl53_1Habilitado, status_.vl53_1);
    printModuleStatus(out, "VL53L0X 2", status_.vl53_2Habilitado, status_.vl53_2);

    printSection(out, F("BARRAMENTOS I2C"));
    printI2CBus(out, "I2C0", config_.board.i2c0, status_.i2c0);
    printI2CBus(out, "I2C1", config_.board.i2c1, status_.i2c1);
    out.print(F("  MPU6050 | barramento=I2C"));
    out.print(config_.board.imuI2C == BARRAMENTO_I2C1 ? 1 : 0);
    out.print(F(" | endereco=0x"));
    out.println(config_.board.mpuAddress, HEX);

    printSection(out, F("IDS DOS SENSORES DE DISTANCIA"));
    out.print(F("  Ultrassom 1: "));
    out.println(config_.board.ultrassom[0].id);
    out.print(F("  Ultrassom 2: "));
    out.println(config_.board.ultrassom[1].id);
    if (status_.vl53_1Habilitado)
    {
        out.print(F("  VL53L0X 1  : "));
        out.println(config_.board.vl53[0].id);
    }
    if (status_.vl53_2Habilitado)
    {
        out.print(F("  VL53L0X 2  : "));
        out.println(config_.board.vl53[1].id);
    }

    printReportEnd(out);
}
bool RoboS3::distanciaDisponivel(SensorDistancia sensor) const
{
    switch (sensor)
    {
        case DISTANCIA_ULTRASSOM_1:
            return status_.ultrassom;
        case DISTANCIA_ULTRASSOM_2:
            return status_.ultrassom2;
        case DISTANCIA_VL53_1:
            return status_.vl53_1;
        case DISTANCIA_VL53_2:
            return status_.vl53_2;
        default:
            return false;
    }
}
bool RoboS3::distanciaDisponivel(uint8_t id) const
{
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (config_.board.ultrassom[i].habilitado && config_.board.ultrassom[i].id == id)
        {
            return i == 0 ? status_.ultrassom : status_.ultrassom2;
        }
        if (config_.board.vl53[i].habilitado && config_.board.vl53[i].id == id)
        {
            return i == 0 ? status_.vl53_1 : status_.vl53_2;
        }
    }
    return false;
}

void RoboS3::printErrosInicializacao(Print& out) const
{
    printReportStart(out, F("DIAGNOSTICO DE INICIALIZACAO"));
    if (status_.inicializacaoCompleta())
    {
        out.println(F("[OK] Todos os modulos previstos foram inicializados."));
        printReportEnd(out);
        return;
    }

    out.println(F("Falhas detectadas durante begin():"));
    printSection(out, F("FALHAS"));
    if (!status_.motores)
    {
        out.println(F("  [ERRO] Motores | falha ao configurar ponte H ou PWM."));
    }
    if (!status_.linha)
    {
        out.println(F("  [ERRO] QTR | falha ao configurar os sensores de linha."));
    }
    if (status_.ultrassom1Habilitado && !status_.ultrassom)
    {
        out.println(F("  [ERRO] Ultrassom 1 | falha ao configurar TRIGGER/ECHO."));
    }
    if (status_.ultrassom2Habilitado && !status_.ultrassom2)
    {
        out.println(F("  [ERRO] Ultrassom 2 | falha ao configurar TRIGGER/ECHO."));
    }
    if (!status_.i2c0)
    {
        out.println(F("  [ERRO] I2C0 | falha ao iniciar o controlador."));
        printI2CBus(out, "I2C0", config_.board.i2c0, false);
    }
    if (!status_.i2c1)
    {
        out.println(F("  [ERRO] I2C1 | falha ao iniciar o controlador."));
        printI2CBus(out, "I2C1", config_.board.i2c1, false);
    }
    auto quantidadeEndereco29 = [this](BarramentoI2C barramento)
    {
        uint8_t quantidade = 0;
        quantidade += corHabilitada(COR_ESQUERDA) && barramentoCor(COR_ESQUERDA) == barramento;
        quantidade += corHabilitada(COR_DIREITA) && barramentoCor(COR_DIREITA) == barramento;
        quantidade += config_.board.vl53[0].habilitado &&
                      config_.board.vl53[0].barramento == barramento;
        quantidade += config_.board.vl53[1].habilitado &&
                      config_.board.vl53[1].barramento == barramento;
        return quantidade;
    };
    const bool conflitoI2C0 = quantidadeEndereco29(BARRAMENTO_I2C0) > 1;
    const bool conflitoI2C1 = quantidadeEndereco29(BARRAMENTO_I2C1) > 1;
    if (conflitoI2C0)
    {
        out.println(F("  [ERRO] I2C0 | mais de um TCS34725/VL53L0X no endereco 0x29."));
    }
    if (conflitoI2C1)
    {
        out.println(F("  [ERRO] I2C1 | mais de um TCS34725/VL53L0X no endereco 0x29."));
    }
    if (status_.corEsquerdaHabilitada && !status_.corEsquerda &&
        !(barramentoCor(COR_ESQUERDA) == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("  [ERRO] Cor esquerda | TCS34725 nao respondeu no "));
        out.println(barramentoCor(COR_ESQUERDA) == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (status_.corDireitaHabilitada && !status_.corDireita &&
        !(barramentoCor(COR_DIREITA) == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("  [ERRO] Cor direita | TCS34725 nao respondeu no "));
        out.println(barramentoCor(COR_DIREITA) == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (status_.vl53_1Habilitado && !status_.vl53_1 &&
        !(config_.board.vl53[0].barramento == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("  [ERRO] VL53L0X 1 | sensor nao respondeu no "));
        out.println(config_.board.vl53[0].barramento == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (status_.vl53_2Habilitado && !status_.vl53_2 &&
        !(config_.board.vl53[1].barramento == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("  [ERRO] VL53L0X 2 | sensor nao respondeu no "));
        out.println(config_.board.vl53[1].barramento == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (!status_.imu)
    {
        out.print(F("  [ERRO] MPU6050 | sem resposta em I2C"));
        out.print(config_.board.imuI2C == BARRAMENTO_I2C1 ? 1 : 0);
        out.print(F(" no endereco 0x"));
        out.println(config_.board.mpuAddress, HEX);
        out.println(F("         Confira SDA/SCL, alimentacao, pull-ups e AD0 (0x68/0x69)."));
    }
    printReportEnd(out);
}
void RoboS3::printLinha(Print& out) const
{
    const auto& r = line_.last();
    printReportStart(out, F("LEITURA DOS SENSORES DE LINHA"));
    out.print(F("  Instante da leitura: "));
    out.print(r.timestampMs);
    out.println(F(" ms"));
    out.print(F("  Linha detectada    : "));
    out.println(r.detectada ? F("sim") : F("nao"));
    out.print(F("  Posicao valida     : "));
    out.println(r.posicaoValida ? F("sim") : F("nao"));
    out.print(F("  Padrao S1..S8      : "));
    out.println(r.padrao);
    out.print(F("  Mascara            : 0b"));
    for (int i = 7; i >= 0; --i)
    {
        out.print((r.mascara >> i) & 1);
    }
    out.println();
    out.print(F("  Sensores pretos    : "));
    out.print(r.quantidadePretos);
    out.println(F(" de 8"));
    out.print(F("  Posicao            : "));
    out.print(r.posicao);
    out.println(F(" / 7000"));
    out.print(F("  Ultima posicao     : "));
    out.println(r.ultimaPosicaoValida);
    out.print(F("  Tempo sem linha    : "));
    out.print(r.tempoSemLinhaMs);
    out.println(F(" ms"));

    printSection(out, F("AMOSTRAS POR SENSOR (S1..S8)"));
    printVector8(out, "Bruto", r.bruto);
    printVector8(out, "Calibrado (0..1000)", r.calibrado);
    printReportEnd(out);
}
void RoboS3::printCores(Print& out)
{
    printReportStart(out, F("LEITURAS DOS SENSORES DE COR"));
    for (uint8_t s = 0; s < 2; ++s)
    {
        auto r = lerDadosCor(static_cast<SensorCor>(s));
        printSection(out, s == 0 ? F("SENSOR ESQUERDO") : F("SENSOR DIREITO"));
        out.print(F("  Estado      : "));
        out.println(r.valida ? F("valida") : F("indisponivel"));
        out.print(F("  Timestamp   : "));
        out.print(r.timestampMs);
        out.println(F(" ms"));
        out.print(F("  RGB (0..255): R="));
        out.print(r.rgb.r);
        out.print(F(" G="));
        out.print(r.rgb.g);
        out.print(F(" B="));
        out.println(r.rgb.b);
        out.print(F("  HSV         : H="));
        out.print(r.hsv.h, 1);
        out.print(F(" deg S="));
        out.print(r.hsv.s, 1);
        out.print(F("% V="));
        out.print(r.hsv.v, 1);
        out.println('%');
        out.print(F("  HSL         : H="));
        out.print(r.hsl.h, 1);
        out.print(F(" deg S="));
        out.print(r.hsl.s, 1);
        out.print(F("% L="));
        out.print(r.hsl.l, 1);
        out.println('%');
        out.print(F("  Bruto       : R="));
        out.print(r.rRaw);
        out.print(F(" G="));
        out.print(r.gRaw);
        out.print(F(" B="));
        out.print(r.bRaw);
        out.print(F(" Clear="));
        out.println(r.clear);
        out.print(F("  Classificada: "));
        out.println(colorName(r.cor));
    }
    printReportEnd(out);
}
void RoboS3::printIMU(Print& out) const
{
    const auto& r = imu_.last();
    printReportStart(out, F("LEITURA DA IMU (MPU6050)"));
    out.print(F("  Estado   : "));
    out.println(r.valida ? F("valida") : F("indisponivel"));
    out.print(F("  Timestamp: "));
    out.print(r.timestampMs);
    out.println(F(" ms"));
    out.print(F("  Barramento/endereco: I2C"));
    out.print(config_.board.imuI2C == BARRAMENTO_I2C1 ? 1 : 0);
    out.print(F(" / 0x"));
    out.println(config_.board.mpuAddress, HEX);

    printSection(out, F("ACELERACAO"));
    out.print(F("  X="));
    out.print(r.accX, 3);
    out.print(F("  Y="));
    out.print(r.accY, 3);
    out.print(F("  Z="));
    out.print(r.accZ, 3);
    out.println(F(" m/s^2"));

    printSection(out, F("VELOCIDADE ANGULAR"));
    out.print(F("  X="));
    out.print(r.gyroX, 3);
    out.print(F("  Y="));
    out.print(r.gyroY, 3);
    out.print(F("  Z="));
    out.print(r.gyroZ, 3);
    out.println(F(" deg/s"));

    printSection(out, F("ORIENTACAO E TEMPERATURA"));
    out.print(F("  Roll="));
    out.print(r.roll, 2);
    out.print(F(" deg | Pitch="));
    out.print(r.pitch, 2);
    out.print(F(" deg | Yaw relativo="));
    out.print(r.yawRelativo, 2);
    out.println(F(" deg"));
    out.print(F("  Temperatura: "));
    out.print(r.temperatura, 2);
    out.println(F(" C"));
    printReportEnd(out);
}
void RoboS3::printMotores(Print& out) const
{
    printReportStart(out, F("ESTADO E CALIBRACAO DOS MOTORES"));
    out.print(F("  Estado do driver: "));
    out.println(status_.motores ? F("pronto") : F("falha"));
    out.print(F("  Comando esquerdo: "));
    out.print(motors_.leftCommand());
    out.println('%');
    out.print(F("  Comando direito : "));
    out.print(motors_.rightCommand());
    out.println('%');

    printSection(out, F("CALIBRACAO APLICADA"));
    printMotorCalibration(out, "esquerdo", motors_.calibration(MOTOR_ESQUERDO),
                          ajustesMotorExplicitos_[MOTOR_ESQUERDO]);
    printMotorCalibration(out, "direito", motors_.calibration(MOTOR_DIREITO),
                          ajustesMotorExplicitos_[MOTOR_DIREITO]);
    out.print(F("  Mapeamento dos motores trocado: "));
    out.println(motors_.swapped() ? F("sim") : F("nao"));
    printReportEnd(out);
}

void RoboS3::printCalibracao(Print& out) const
{
    const bool possuiAjustesExplicitos =
        ajustesMotorExplicitos_[MOTOR_ESQUERDO] != 0 ||
        ajustesMotorExplicitos_[MOTOR_DIREITO] != 0;
    printReportStart(out, F("RELATORIO DE CALIBRACAO APLICADA"));
    out.print(F("  Origem base dos dados: "));
    out.println(status_.calibracaoCarregada ? F("NVS (calibracao salva)") : F("valores padrao"));
    out.print(F("  Alterada desde salvar/carregar: "));
    out.println(calibracaoAlterada_ ? F("sim; valores atuais ainda nao salvos") : F("nao"));
    out.print(F("  Ajustes de motor no sketch: "));
    out.println(possuiAjustesExplicitos ? F("sim; detalhados por motor") : F("nenhum"));

    uint16_t minimos[8]{};
    uint16_t maximos[8]{};
    const bool qtrCalibrado = line_.getCalibration(minimos, maximos);
    printSection(out, F("LINHA (QTR)"));
    out.print(F("  Calibracao dos extremos: "));
    out.println(qtrCalibrado ? F("disponivel") : F("ainda nao realizada"));
    out.print(F("  Limiar de deteccao (0..1000): "));
    out.println(line_.threshold());
    if (qtrCalibrado)
    {
        printVector8(out, "Minimos S1..S8", minimos);
        printVector8(out, "Maximos S1..S8", maximos);
    }

    printSection(out, F("MOTORES"));
    for (uint8_t i = 0; i < 2; ++i)
    {
        const CalibracaoMotor& motor =
            motors_.calibration(i == 0 ? MOTOR_ESQUERDO : MOTOR_DIREITO);
        printMotorCalibration(out, i == 0 ? "esquerdo" : "direito", motor,
                              ajustesMotorExplicitos_[i]);
    }
    out.print(F("  Mapeamento trocado dos motores: "));
    out.print(motors_.swapped() ? F("sim") : F("nao"));
    out.println();
    out.print(F("  Mapeamento trocado dos sensores de cor: "));
    out.println(sensoresCorTrocados_ ? F("sim") : F("nao"));

    printSection(out, F("SENSORES DE COR (TCS34725)"));
    static const uint8_t ganhos[] = {1, 4, 16, 60};
    static const char* integracoes[] = {"2.4 ms", "24 ms", "50 ms", "101 ms", "154 ms", "600 ms"};
    for (uint8_t sensor = 0; sensor < 2; ++sensor)
    {
        const uint8_t ganho = static_cast<uint8_t>(colors_[sensor].gain());
        const uint8_t integracao = static_cast<uint8_t>(colors_[sensor].integrationTime());
        out.print(F("  Sensor "));
        out.println(sensor == 0 ? F("esquerdo") : F("direito"));
        out.print(F("    Estado     : "));
        const bool colorEnabled = sensor == 0 ? status_.corEsquerdaHabilitada
                                              : status_.corDireitaHabilitada;
        const bool colorReady = sensor == 0 ? status_.corEsquerda : status_.corDireita;
        out.println(!colorEnabled ? F("desativado") : colorReady ? F("pronto") : F("falha"));
        out.print(F("    Ganho      : "));
        out.print(ganho < 4 ? ganhos[ganho] : 0);
        out.println('x');
        out.print(F("    Integracao : "));
        out.println(integracao < 6 ? integracoes[integracao] : "invalida");
        bool possuiFaixaAtiva = false;
        for (uint8_t cor = 0; cor < COR_TOTAL; ++cor)
        {
            const FaixaHSV& faixa = colors_[sensor].range(static_cast<CorDetectada>(cor));
            if (!faixa.ativa)
            {
                continue;
            }
            possuiFaixaAtiva = true;
            const char* nome = colorName(static_cast<CorDetectada>(cor));
            out.print(F("    "));
            out.print(nome);
            for (size_t i = strlen(nome); i < 12; ++i)
            {
                out.print(' ');
            }
            out.print(F(" | H="));
            out.print(faixa.hMin, 1);
            out.print(F(".."));
            out.print(faixa.hMax, 1);
            out.print(F(" deg | S="));
            out.print(faixa.sMin, 1);
            out.print(F(".."));
            out.print(faixa.sMax, 1);
            out.print(F("% | V="));
            out.print(faixa.vMin, 1);
            out.print(F(".."));
            out.print(faixa.vMax, 1);
            out.println('%');
        }
        if (!possuiFaixaAtiva)
        {
            out.println(F("    Faixas HSV ativas: nenhuma"));
        }
    }

    printSection(out, F("IMU (MPU6050)"));
    out.print(F("  Estado do sensor: "));
    out.println(status_.imu ? F("pronto") : F("falha/indisponivel"));
    out.print(F("  Barramento     : I2C"));
    out.println(config_.board.imuI2C == BARRAMENTO_I2C1 ? 1 : 0);
    out.print(F("  Endereco       : 0x"));
    out.println(config_.board.mpuAddress, HEX);
    const robos3::IMUOffsets& offsets = imu_.offsets();
    out.print(F("  Offsets do acelerometro (LSB): X="));
    out.print(offsets.ax);
    out.print(F(" Y="));
    out.print(offsets.ay);
    out.print(F(" Z="));
    out.println(offsets.az);
    out.print(F("  Offsets do giroscopio (LSB) : X="));
    out.print(offsets.gx);
    out.print(F(" Y="));
    out.print(offsets.gy);
    out.print(F(" Z="));
    out.println(offsets.gz);
    printReportEnd(out);
}
