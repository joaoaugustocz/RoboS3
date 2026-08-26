#include "RoboS3.h"
#include "modules/RoboMath.h"

#include <Wire.h>
#include <math.h>
#include <string.h>
#if defined(ROBOS3_CALIBRATION_WEB) || defined(ROBOS3_CALIBRATION_SERIAL)
#include "calibration/CalibrationManager.h"
#endif
#ifdef ROBOS3_CALIBRATION_SERIAL
#include "calibration/SerialCalibration.h"
#endif
#ifdef ROBOS3_CALIBRATION_WEB
#include "calibration/WebCalibration.h"
#endif

using namespace robos3;

RoboS3::RoboS3()
    : config_(RoboConfig::padrao()), status_{}, motors_{}, line_{}, pid_{},
      ultrasonic_{}, colors_{}, imu_{}, storage_{}, debug_{}, begun_(false)
#ifdef ROBOS3_CALIBRATION_SERIAL
      ,
      serialCalibration_(nullptr)
#endif
#ifdef ROBOS3_CALIBRATION_WEB
      ,
      webCalibration_(nullptr)
#endif
#if defined(ROBOS3_CALIBRATION_WEB) || defined(ROBOS3_CALIBRATION_SERIAL)
      ,
      calibrationManager_(nullptr)
#endif
{
}

bool RoboS3::begin()
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
    status_.ultrassom =
        ultrasonic_.begin(config_.board.ultrasonicTrig, config_.board.ultrasonicEcho);
    status_.i2c0 =
        Wire.begin(config_.board.i2c0.sda, config_.board.i2c0.scl, config_.board.i2c0.frequency);
    status_.i2c1 =
        Wire1.begin(config_.board.i2c1.sda, config_.board.i2c1.scl, config_.board.i2c1.frequency);
    status_.corEsquerda = status_.i2c0 && colors_[0].begin(Wire);
    status_.corDireita = status_.i2c1 && colors_[1].begin(Wire1);
    status_.imu = status_.i2c0 && imu_.begin(Wire, config_.board.mpuAddress);
    pid_.configure(config_.kp, config_.ki, config_.kd, -config_.limitePID, config_.limitePID);
    status_.calibracaoCarregada = carregarCalibracao();
    begun_ = true;
    if (debug_.level() >= DEBUG_INFO)
    {
        printStatus();
    }
#if defined(ROBOS3_CALIBRATION_WEB) || defined(ROBOS3_CALIBRATION_SERIAL)
    static CalibrationManager manager(*this);
    calibrationManager_ = &manager;
#endif
#ifdef ROBOS3_CALIBRATION_SERIAL
    static SerialCalibration serial(manager);
    serialCalibration_ = &serial;
    serial.begin();
#endif
#ifdef ROBOS3_CALIBRATION_WEB
    static WebCalibration web(manager);
    webCalibration_ = &web;
    web.begin();
#endif
    // A inicializacao de cada modulo e independente. Assim o programa pode
    // consultar status() e decidir se consegue trabalhar com o hardware restante.
    return status_.inicializacaoCompleta();
}

void RoboS3::atualizar()
{
    if (status_.imu)
    {
        imu_.update();
    }
#if defined(ROBOS3_CALIBRATION_WEB) || defined(ROBOS3_CALIBRATION_SERIAL)
    if (calibrationManager_)
    {
        calibrationManager_->update();
    }
#endif
#ifdef ROBOS3_CALIBRATION_SERIAL
    if (serialCalibration_)
    {
        serialCalibration_->update();
    }
#endif
#ifdef ROBOS3_CALIBRATION_WEB
    if (webCalibration_)
    {
        webCalibration_->update();
    }
#endif
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
    config_.board.ultrasonicTrig = trigger;
    config_.board.ultrasonicEcho = echo;
}
void RoboS3::definePinsI2C0(uint8_t sda, uint8_t scl, uint32_t frequency)
{
    config_.board.i2c0 = {sda, scl, frequency};
}
void RoboS3::definePinsI2C1(uint8_t sda, uint8_t scl, uint32_t frequency)
{
    config_.board.i2c1 = {sda, scl, frequency};
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
}
void RoboS3::inverterMotorDireito(bool value)
{
    auto c = motors_.calibration(MOTOR_DIREITO);
    c.invertido = value;
    config_.motorDireito = c;
    motors_.setCalibration(MOTOR_DIREITO, c);
}
void RoboS3::inverterMotores(bool left, bool right)
{
    inverterMotorEsquerdo(left);
    inverterMotorDireito(right);
}
void RoboS3::configurarCompensacaoMotor(Motor motor, SentidoMotor direction, float factor)
{
    auto c = motors_.calibration(motor);
    if (direction == FRENTE)
    {
        c.compensacaoFrente = factor;
    }
    else
    {
        c.compensacaoTras = factor;
    }
    if (motor == MOTOR_ESQUERDO)
    {
        config_.motorEsquerdo = c;
    }
    else
    {
        config_.motorDireito = c;
    }
    motors_.setCalibration(motor, c);
}
void RoboS3::configurarMinimoMovimento(Motor motor, SentidoMotor direction, uint8_t minimum)
{
    auto c = motors_.calibration(motor);
    minimum = minimum > 100 ? 100 : minimum;
    if (direction == FRENTE)
    {
        c.minimoFrente = minimum;
    }
    else
    {
        c.minimoTras = minimum;
    }
    if (motor == MOTOR_ESQUERDO)
    {
        config_.motorEsquerdo = c;
    }
    else
    {
        config_.motorDireito = c;
    }
    motors_.setCalibration(motor, c);
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
const LeituraLinha& RoboS3::ultimaLeituraLinha() const
{
    return line_.last();
}
uint32_t RoboS3::tempoSemLinha() const
{
    return line_.timeWithoutLine();
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
    return ultrasonic_.distanceCm();
}
bool RoboS3::temObstaculo(float cm)
{
    return ultrasonic_.hasObstacle(cm);
}
ColorSensor& RoboS3::color(SensorCor s)
{
    return colors_[s == COR_DIREITA ? 1 : 0];
}
const ColorSensor& RoboS3::color(SensorCor s) const
{
    return colors_[s == COR_DIREITA ? 1 : 0];
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
    motors_.setCalibration(MOTOR_ESQUERDO, d.leftMotor);
    motors_.setCalibration(MOTOR_DIREITO, d.rightMotor);
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
    return storage_.save(makeCalibration());
}
bool RoboS3::carregarCalibracao()
{
    RoboCalibration d{};
    if (!storage_.load(d))
    {
        return false;
    }
    applyCalibration(d);
    return true;
}
bool RoboS3::apagarCalibracao()
{
    return storage_.erase();
}
bool RoboS3::corDisponivel(SensorCor s) const
{
    return s == COR_ESQUERDA ? status_.corEsquerda : status_.corDireita;
}

void RoboS3::printStatus(Print& out) const
{
    out.println(F("[RoboS3] Status"));
    const char* names[] = {"Motores",      "QTR",         "Ultrassom", "I2C0",
                           "I2C1",         "TCS esquerdo", "TCS direito", "MPU6050"};
    const bool values[] = {status_.motores,      status_.linha,       status_.ultrassom,
                           status_.i2c0,         status_.i2c1,        status_.corEsquerda,
                           status_.corDireita,   status_.imu};
    for (uint8_t i = 0; i < 8; ++i)
    {
        out.print(values[i] ? F("[OK] ") : F("[ERRO] "));
        out.println(names[i]);
    }
    out.print(status_.calibracaoCarregada ? F("[OK] ") : F("[PADRAO] "));
    out.println(F("Calibracao"));
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
    if (!status_.ultrassom)
    {
        out.println(F("- Ultrassom: falha ao configurar TRIGGER/ECHO."));
    }
    if (!status_.i2c0)
    {
        out.println(F("- I2C0: falha ao iniciar SDA/SCL do barramento 0."));
    }
    if (!status_.i2c1)
    {
        out.println(F("- I2C1: falha ao iniciar SDA/SCL do barramento 1."));
    }
    if (!status_.corEsquerda)
    {
        out.println(F("- Cor esquerda: TCS34725 nao respondeu no I2C0."));
    }
    if (!status_.corDireita)
    {
        out.println(F("- Cor direita: TCS34725 nao respondeu no I2C1."));
    }
    if (!status_.imu)
    {
        out.println(F("- IMU: MPU6050 nao respondeu no I2C0/endereco configurado."));
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
