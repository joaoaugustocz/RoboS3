#include "RoboS3.h"
#include "modules/RoboMath.h"

#include <Wire.h>
#include <math.h>
#include <string.h>

using namespace robos3;

RoboS3::RoboS3()
    : config_(RoboConfig::padrao()), status_{}, motors_{}, line_{}, pid_{},
      ultrasonic_{}, vl53_{}, colors_{}, imu_{}, storage_{}, buttons_{}, debug_{},
      interfacesCalibracao_{nullptr, nullptr}, atualizadoresCalibracao_{nullptr, nullptr},
      ajustesMotorExplicitos_{0, 0}, begun_(false), anguloZInvertido_(false),
      sensoresCorTrocados_(false)
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
}
void RoboS3::inverterMotorDireito(bool value)
{
    auto c = motors_.calibration(MOTOR_DIREITO);
    c.invertido = value;
    config_.motorDireito = c;
    motors_.setCalibration(MOTOR_DIREITO, c);
    ajustesMotorExplicitos_[MOTOR_DIREITO] |= AJUSTE_MOTOR_INVERSAO;
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
void RoboS3::calibrarLinha(uint32_t ms)
{
    parar();
    line_.calibrateMs(ms);
}
void RoboS3::calibrarLinhaAmostras(uint16_t samples)
{
    parar();
    line_.calibrateSamples(samples);
}
void RoboS3::configurarLimiarLinha(uint16_t threshold)
{
    config_.limiarLinha = threshold;
    line_.setThreshold(threshold);
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
}
void RoboS3::definirFaixaHSV(SensorCor s, CorDetectada c, float hMin, float hMax, float sMin,
                             float sMax, float vMin, float vMax)
{
    color(s).setRange(c, {hMin, hMax, sMin, sMax, vMin, vMax, true});
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
}
void RoboS3::setTempoIntegracaoCor(SensorCor s, TempoIntegracaoCor time)
{
    color(s).setIntegrationTime(time);
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
    return apagouNvs;
}
bool RoboS3::corDisponivel(SensorCor s) const
{
    return s == COR_ESQUERDA ? status_.corEsquerda : status_.corDireita;
}

void RoboS3::printStatus(Print& out) const
{
    out.println(F("[RoboS3] Status"));
    const char* names[] = {"Motores", "QTR", "ULTRA1", "ULTRA2", "I2C0", "I2C1",
                           "TCS esquerdo", "TCS direito", "MPU6050", "VL53 #1", "VL53 #2"};
    const bool enabled[] = {true, true, status_.ultrassom1Habilitado,
                            status_.ultrassom2Habilitado, true, true,
                            status_.corEsquerdaHabilitada, status_.corDireitaHabilitada,
                            true, status_.vl53_1Habilitado, status_.vl53_2Habilitado};
    const bool values[] = {status_.motores, status_.linha, status_.ultrassom,
                           status_.ultrassom2, status_.i2c0, status_.i2c1,
                           status_.corEsquerda, status_.corDireita, status_.imu,
                           status_.vl53_1, status_.vl53_2};
    for (uint8_t i = 0; i < 11; ++i)
    {
        out.print(!enabled[i] ? F("[DESATIVADO] ") : values[i] ? F("[OK] ") : F("[ERRO] "));
        out.println(names[i]);
    }
    out.print(F("IDs distancia: ULTRA1="));
    out.print(config_.board.ultrassom[0].id);
    out.print(F(", ULTRA2="));
    out.print(config_.board.ultrassom[1].id);
    if (status_.vl53_1Habilitado)
    {
        out.print(F(", VL53#1="));
        out.print(config_.board.vl53[0].id);
    }
    if (status_.vl53_2Habilitado)
    {
        out.print(F(", VL53#2="));
        out.print(config_.board.vl53[1].id);
    }
    out.println();
    out.print(status_.calibracaoCarregada ? F("[OK] ") : F("[PADRAO] "));
    out.println(F("Calibracao"));
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
    if (status_.inicializacaoCompleta())
    {
        out.println(F("[RoboS3] Nenhum erro de inicializacao."));
        return;
    }

    out.println(F("[RoboS3] Erros de inicializacao:"));
    if (!status_.motores)
    {
        out.println(F("- Motores: falha ao configurar a ponte H/PWM."));
    }
    if (!status_.linha)
    {
        out.println(F("- QTR: falha ao configurar os sensores de linha."));
    }
    if (status_.ultrassom1Habilitado && !status_.ultrassom)
    {
        out.println(F("- ULTRA1: falha ao configurar TRIGGER/ECHO."));
    }
    if (status_.ultrassom2Habilitado && !status_.ultrassom2)
    {
        out.println(F("- ULTRA2: falha ao configurar TRIGGER/ECHO."));
    }
    if (!status_.i2c0)
    {
        out.println(F("- I2C0: falha ao iniciar SDA/SCL do barramento 0."));
    }
    if (!status_.i2c1)
    {
        out.println(F("- I2C1: falha ao iniciar SDA/SCL do barramento 1."));
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
        out.println(F("- I2C0: mais de um TCS34725/VL53L0X usa o endereco 0x29."));
    }
    if (conflitoI2C1)
    {
        out.println(F("- I2C1: mais de um TCS34725/VL53L0X usa o endereco 0x29."));
    }
    if (status_.corEsquerdaHabilitada && !status_.corEsquerda &&
        !(barramentoCor(COR_ESQUERDA) == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("- Cor esquerda: TCS34725 nao respondeu no "));
        out.println(barramentoCor(COR_ESQUERDA) == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (status_.corDireitaHabilitada && !status_.corDireita &&
        !(barramentoCor(COR_DIREITA) == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("- Cor direita: TCS34725 nao respondeu no "));
        out.println(barramentoCor(COR_DIREITA) == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (status_.vl53_1Habilitado && !status_.vl53_1 &&
        !(config_.board.vl53[0].barramento == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("- VL53 #1: sensor nao respondeu no "));
        out.println(config_.board.vl53[0].barramento == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (status_.vl53_2Habilitado && !status_.vl53_2 &&
        !(config_.board.vl53[1].barramento == BARRAMENTO_I2C0 ? conflitoI2C0 : conflitoI2C1))
    {
        out.print(F("- VL53 #2: sensor nao respondeu no "));
        out.println(config_.board.vl53[1].barramento == BARRAMENTO_I2C1 ? F("I2C1.") : F("I2C0."));
    }
    if (!status_.imu)
    {
        out.print(F("- IMU: MPU6050 nao respondeu no "));
        out.println(config_.board.imuI2C == BARRAMENTO_I2C1 ? F("I2C1/endereco configurado.")
                                                             : F("I2C0/endereco configurado."));
    }
}
void RoboS3::printLinha(Print& out) const
{
    const auto& r = line_.last();
    out.print(F("BRUTO: "));
    for (auto v : r.bruto)
    {
        out.print(v);
        out.print(' ');
    }
    out.print(F("\nCALIBRADO: "));
    for (auto v : r.calibrado)
    {
        out.print(v);
        out.print(' ');
    }
    out.print(F("\nPADRAO: "));
    out.println(r.padrao);
    out.print(F("MASCARA: 0b"));
    for (int i = 7; i >= 0; --i)
    {
        out.print((r.mascara >> i) & 1);
    }
    out.print(F("\nPOSICAO: "));
    out.println(r.posicao);
    out.print(F("TEMPO SEM LINHA: "));
    out.println(r.tempoSemLinhaMs);
}
void RoboS3::printCores(Print& out)
{
    for (uint8_t s = 0; s < 2; ++s)
    {
        auto r = lerDadosCor(static_cast<SensorCor>(s));
        out.print(s ? F("D: ") : F("E: "));
        out.print(r.rgb.r);
        out.print(' ');
        out.print(r.rgb.g);
        out.print(' ');
        out.print(r.rgb.b);
        out.print(F(" HSV "));
        out.print(r.hsv.h);
        out.print(' ');
        out.print(r.hsv.s);
        out.print(' ');
        out.println(r.hsv.v);
    }
}
void RoboS3::printIMU(Print& out) const
{
    const auto& r = imu_.last();
    out.print(F("ACC "));
    out.print(r.accX);
    out.print(' ');
    out.print(r.accY);
    out.print(' ');
    out.println(r.accZ);
    out.print(F("GYRO "));
    out.print(r.gyroX);
    out.print(' ');
    out.print(r.gyroY);
    out.print(' ');
    out.println(r.gyroZ);
    out.print(F("ANG "));
    out.print(r.roll);
    out.print(' ');
    out.print(r.pitch);
    out.print(' ');
    out.println(r.yawRelativo);
}
void RoboS3::printMotores(Print& out) const
{
    out.print(F("E="));
    out.print(motors_.leftCommand());
    out.print(F(" D="));
    out.println(motors_.rightCommand());
}

void RoboS3::printCalibracao(Print& out) const
{
    const bool possuiAjustesExplicitos =
        ajustesMotorExplicitos_[MOTOR_ESQUERDO] != 0 ||
        ajustesMotorExplicitos_[MOTOR_DIREITO] != 0;
    out.print(F("[RoboS3] Calibracao aplicada: "));
    out.print(status_.calibracaoCarregada ? F("SALVA NA NVS") : F("VALORES PADRAO"));
    if (possuiAjustesExplicitos)
    {
        out.print(F(" + AJUSTES DO CODIGO"));
    }
    out.println();

    uint16_t minimos[8]{};
    uint16_t maximos[8]{};
    const bool qtrCalibrado = line_.getCalibration(minimos, maximos);
    out.print(F("  QTR calibrado="));
    out.print(qtrCalibrado ? F("sim") : F("nao"));
    out.print(F(" limiar="));
    out.print(line_.threshold());
    if (qtrCalibrado)
    {
        out.print(F(" min="));
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (i)
            {
                out.print(',');
            }
            out.print(minimos[i]);
        }
        out.print(F(" max="));
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (i)
            {
                out.print(',');
            }
            out.print(maximos[i]);
        }
    }
    out.println();

    for (uint8_t i = 0; i < 2; ++i)
    {
        const CalibracaoMotor& motor =
            motors_.calibration(i == 0 ? MOTOR_ESQUERDO : MOTOR_DIREITO);
        out.print(i == 0 ? F("  Motor E") : F("  Motor D"));
        out.print(F(": compFrente="));
        out.print(motor.compensacaoFrente, 3);
        out.print(F(" compTras="));
        out.print(motor.compensacaoTras, 3);
        out.print(F(" minimoFrente="));
        out.print(motor.minimoFrente);
        out.print(F(" minimoTras="));
        out.print(motor.minimoTras);
        out.print(F(" invertido="));
        out.print(motor.invertido ? F("sim") : F("nao"));
        const uint8_t ajustes = ajustesMotorExplicitos_[i];
        out.print(F(" ajustesCodigo="));
        if (ajustes == 0)
        {
            out.println(F("nenhum"));
            continue;
        }
        bool separador = false;
        if (ajustes & AJUSTE_MOTOR_INVERSAO)
        {
            out.print(F("inversao"));
            separador = true;
        }
        if (ajustes & AJUSTE_MOTOR_COMPENSACAO_FRENTE)
        {
            out.print(separador ? F(",compFrente") : F("compFrente"));
            separador = true;
        }
        if (ajustes & AJUSTE_MOTOR_COMPENSACAO_TRAS)
        {
            out.print(separador ? F(",compTras") : F("compTras"));
            separador = true;
        }
        if (ajustes & AJUSTE_MOTOR_MINIMO_FRENTE)
        {
            out.print(separador ? F(",minimoFrente") : F("minimoFrente"));
            separador = true;
        }
        if (ajustes & AJUSTE_MOTOR_MINIMO_TRAS)
        {
            out.print(separador ? F(",minimoTras") : F("minimoTras"));
        }
        out.println();
    }
    out.print(F("  Mapeamento: motoresTrocados="));
    out.print(motors_.swapped() ? F("sim") : F("nao"));
    out.print(F(" coresTrocadas="));
    out.println(sensoresCorTrocados_ ? F("sim") : F("nao"));

    static const char* nomesCores[COR_TOTAL] =
        {"desconhecida", "preta", "branca", "vermelha", "verde", "azul", "amarela", "laranja", "roxa"};
    static const uint8_t ganhos[] = {1, 4, 16, 60};
    static const char* integracoes[] = {"2.4ms", "24ms", "50ms", "101ms", "154ms", "600ms"};
    for (uint8_t sensor = 0; sensor < 2; ++sensor)
    {
        const uint8_t ganho = static_cast<uint8_t>(colors_[sensor].gain());
        const uint8_t integracao = static_cast<uint8_t>(colors_[sensor].integrationTime());
        out.print(sensor == 0 ? F("  Cor E") : F("  Cor D"));
        out.print(F(": ganho="));
        out.print(ganho < 4 ? ganhos[ganho] : 0);
        out.print('x');
        out.print(F(" integracao="));
        out.println(integracao < 6 ? integracoes[integracao] : "invalida");
        for (uint8_t cor = 0; cor < COR_TOTAL; ++cor)
        {
            const FaixaHSV& faixa = colors_[sensor].range(static_cast<CorDetectada>(cor));
            if (!faixa.ativa)
            {
                continue;
            }
            out.print(F("    "));
            out.print(nomesCores[cor]);
            out.print(F(": H="));
            out.print(faixa.hMin, 1);
            out.print(F(".."));
            out.print(faixa.hMax, 1);
            out.print(F(" S="));
            out.print(faixa.sMin, 1);
            out.print(F(".."));
            out.print(faixa.sMax, 1);
            out.print(F(" V="));
            out.print(faixa.vMin, 1);
            out.print(F(".."));
            out.println(faixa.vMax, 1);
        }
    }

    const robos3::IMUOffsets& offsets = imu_.offsets();
    out.print(F("  IMU offsets: acc="));
    out.print(offsets.ax);
    out.print(',');
    out.print(offsets.ay);
    out.print(',');
    out.print(offsets.az);
    out.print(F(" gyro="));
    out.print(offsets.gx);
    out.print(',');
    out.print(offsets.gy);
    out.print(',');
    out.println(offsets.gz);
}
