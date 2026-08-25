#pragma once

#include <Arduino.h>
#include "config/RoboConfig.h"
#include "types/RoboTypes.h"
#include "modules/ColorSensor.h"
#include "modules/IMUSensor.h"
#include "modules/LinePID.h"
#include "modules/LineSensor.h"
#include "modules/MotorDriver.h"
#include "modules/UltrasonicSensor.h"
#include "modules/CalibrationStorage.h"
#include "debug/RoboDebug.h"

namespace robos3
{
class SerialCalibration;
class WebCalibration;
class CalibrationManager;
} // namespace robos3

class RoboS3
{
  public:
    RoboS3();
    bool begin();
    void atualizar();

    // Overrides devem ser chamados antes de begin().
    void definePinsQTR(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint8_t s5, uint8_t s6,
                       uint8_t s7, uint8_t s8, uint8_t emitter = 0xFF);
    void setTipoQTR(TipoQTR type)
    {
        config_.board.qtrType = type;
    }
    void setTipoLinha(TipoLinha type);
    void definePinsPonteH(uint8_t ena, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                          uint8_t enb);
    void definePinsUltrassom(uint8_t trigger, uint8_t echo);
    void definePinsI2C0(uint8_t sda, uint8_t scl, uint32_t frequency = 400000);
    void definePinsI2C1(uint8_t sda, uint8_t scl, uint32_t frequency = 400000);
    void configurarIMU(uint8_t address)
    {
        config_.board.mpuAddress = address;
    }

    void mover(int velocidadeEsquerda, int velocidadeDireita);
    void frente(int velocidade);
    void tras(int velocidade);
    void girarEsquerda(int velocidade);
    void girarDireita(int velocidade);
    void motorEsquerdo(int velocidade);
    void motorDireito(int velocidade);
    void parar();
    void frear();
    void inverterMotorEsquerdo(bool invertido);
    void inverterMotorDireito(bool invertido);
    void inverterMotores(bool esquerdo, bool direito);
    void configurarCompensacaoMotor(Motor motor, SentidoMotor sentido, float fator);
    void configurarMinimoMovimento(Motor motor, SentidoMotor sentido, uint8_t minimo);

    LeituraLinha lerLinha();
    void lerLinhaBruta(uint16_t output[8]);
    void lerLinhaCalibrada(uint16_t output[8]);
    uint8_t lerLinhaBinaria();
    const LeituraLinha& ultimaLeituraLinha() const;
    uint32_t tempoSemLinha() const;
    void calibrarLinha(uint32_t durationMs = 5000);
    void calibrarLinhaAmostras(uint16_t samples);
    void configurarLimiarLinha(uint16_t threshold);
    int16_t erroLinha(int16_t center = 3500) const;
    void configurarPID(float kp, float ki, float kd);
    void resetPID();
    float calcularPID(float error);
    void seguirLinha(int velocidadeBase);

    float distanciaCM();
    bool temObstaculo(float maximumCm);

    LeituraCor lerDadosCor(SensorCor sensor);
    RGB lerRGB(SensorCor sensor);
    HSV lerHSV(SensorCor sensor);
    HSL lerHSL(SensorCor sensor);
    CorDetectada lerCor(SensorCor sensor);
    void definirFaixaH(SensorCor sensor, CorDetectada color, float hMin, float hMax);
    void definirFaixaHSV(SensorCor sensor, CorDetectada color, float hMin, float hMax, float sMin,
                         float sMax, float vMin, float vMax);
    void calibrarCor(SensorCor sensor, CorDetectada color, uint16_t samples = 30);
    void setGanhoCor(SensorCor sensor, GanhoCor gain);
    void setTempoIntegracaoCor(SensorCor sensor, TempoIntegracaoCor time);

    LeituraIMU lerIMU();
    void atualizarIMU();
    float roll() const;
    float pitch() const;
    float anguloZ() const;
    void zerarAnguloZ();
    bool estaInclinado(float graus) const;
    void calibrarIMU(uint32_t durationMs = 3000);

    bool salvarCalibracao();
    bool carregarCalibracao();
    bool apagarCalibracao();

    bool linhaDisponivel() const
    {
        return status_.linha;
    }
    bool ultrassomDisponivel() const
    {
        return status_.ultrassom;
    }
    bool corDisponivel(SensorCor sensor) const;
    bool imuDisponivel() const
    {
        return status_.imu;
    }
    const StatusRobo& status() const
    {
        return status_;
    }

    void printStatus(Print& out = Serial) const;
    void printLinha(Print& out = Serial) const;
    void printCores(Print& out = Serial);
    void printIMU(Print& out = Serial) const;
    void printMotores(Print& out = Serial) const;
    void setDebugLevel(DebugLevel level)
    {
        debug_.setLevel(level);
    }

  private:
    friend class robos3::SerialCalibration;
    friend class robos3::WebCalibration;
    robos3::ColorSensor& color(SensorCor sensor);
    const robos3::ColorSensor& color(SensorCor sensor) const;
    robos3::RoboCalibration makeCalibration() const;
    void applyCalibration(const robos3::RoboCalibration& data);

    robos3::RoboConfig config_;
    StatusRobo status_;
    robos3::MotorDriver motors_;
    robos3::LineSensor line_;
    robos3::LinePID pid_;
    robos3::UltrasonicSensor ultrasonic_;
    robos3::ColorSensor colors_[2];
    robos3::IMUSensor imu_;
    robos3::CalibrationStorage storage_;
    robos3::RoboDebug debug_;
    bool begun_;
#ifdef ROBOS3_CALIBRATION_SERIAL
    robos3::SerialCalibration* serialCalibration_;
#endif
#ifdef ROBOS3_CALIBRATION_WEB
    robos3::WebCalibration* webCalibration_;
#endif
#if defined(ROBOS3_CALIBRATION_WEB) || defined(ROBOS3_CALIBRATION_SERIAL)
    robos3::CalibrationManager* calibrationManager_;
#endif
};
