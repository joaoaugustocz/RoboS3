#include "LineSensor.h"

#include <Arduino.h>
#include <string.h>
#include "../config/BoardPins.h"
#include "RoboMath.h"

namespace robos3
{

static_assert(board::ADC_MAX_VALUE == 4095, "O backend QTR-8A da RoboS3 exige ADC de 12 bits.");

namespace
{
bool validCalibrationRanges(const uint16_t minimum[8], const uint16_t maximum[8])
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (minimum[i] >= maximum[i])
        {
            return false;
        }
    }
    return true;
}
} // namespace

LineSensor::LineSensor()
    : type_(QTR_8A), pins_{}, analogMinimum_{}, analogMaximum_{},
      analogCalibrationInitialized_(false), lineType_(LINHA_PRETA), threshold_(500), last_{},
      lastDetectionMs_(0), lastValidPosition_(3500), begun_(false), mirrored_(false), history_{},
      historyCapacity_(8), historyCount_(0), historyNext_(0)
{
    resetAnalogCalibration();
    linePattern(0, last_.padrao);
    last_.ultimaPosicaoValida = 3500;
}

bool LineSensor::begin(const uint8_t pins[8], TipoQTR type, uint8_t emitterPin)
{
    type_ = type;
    memcpy(pins_, pins, sizeof(pins_));
    if (type == QTR_8A)
    {
        prepararADC();
        for (uint8_t i = 0; i < 8; ++i)
        {
            pinMode(pins_[i], INPUT);
        }
    }
    else
    {
        qtr_.setTypeRC();
        qtr_.setSensorPins(pins, 8);
        if (emitterPin != 0xFF)
        {
            qtr_.setEmitterPin(emitterPin);
        }
    }
    resetAnalogCalibration();
    begun_ = true;
    lastDetectionMs_ = millis();
    clearHistory();
    return true;
}

void LineSensor::readAnalogRaw(uint16_t output[8]) const
{
    prepararADC();
    for (uint8_t sensor = 0; sensor < 8; ++sensor)
    {
        uint32_t sum = 0;
        for (uint8_t sample = 0; sample < ANALOG_SAMPLES_PER_READ; ++sample)
        {
            sum += static_cast<uint16_t>(analogRead(pins_[sensor]));
        }
        const uint32_t rounded =
            (sum + ANALOG_SAMPLES_PER_READ / 2U) / ANALOG_SAMPLES_PER_READ;
        output[sensor] = static_cast<uint16_t>(
            rounded > board::ADC_MAX_VALUE ? board::ADC_MAX_VALUE : rounded);
    }
}

void LineSensor::resetAnalogCalibration()
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        analogMinimum_[i] = board::ADC_MAX_VALUE;
        analogMaximum_[i] = 0;
    }
    analogCalibrationInitialized_ = false;
}

bool LineSensor::calibrationInitialized() const
{
    return type_ == QTR_8A ? analogCalibrationInitialized_ : qtr_.calibrationOn.initialized;
}

void LineSensor::calibrateAnalogStep(uint16_t minimum[8], uint16_t maximum[8]) const
{
    uint16_t stepMinimum[8];
    uint16_t stepMaximum[8]{};
    for (uint8_t i = 0; i < 8; ++i)
    {
        stepMinimum[i] = board::ADC_MAX_VALUE;
    }

    for (uint8_t reading = 0; reading < CALIBRATION_READS_PER_STEP; ++reading)
    {
        uint16_t raw[8];
        readAnalogRaw(raw);
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (raw[i] < stepMinimum[i])
            {
                stepMinimum[i] = raw[i];
            }
            if (raw[i] > stepMaximum[i])
            {
                stepMaximum[i] = raw[i];
            }
        }
    }

    for (uint8_t i = 0; i < 8; ++i)
    {
        // Mesmo filtro da QTRSensors: os extremos internos do lote evitam que
        // uma unica amostra espuria amplie toda a faixa de calibracao.
        if (stepMaximum[i] < minimum[i])
        {
            minimum[i] = stepMaximum[i];
        }
        if (stepMinimum[i] > maximum[i])
        {
            maximum[i] = stepMinimum[i];
        }
    }
}

void LineSensor::calibrateStep(uint16_t minimum[8], uint16_t maximum[8])
{
    if (type_ == QTR_8A)
    {
        calibrateAnalogStep(minimum, maximum);
        return;
    }

    uint16_t stepMinimum[8];
    uint16_t stepMaximum[8]{};
    for (uint8_t i = 0; i < 8; ++i)
    {
        stepMinimum[i] = 0xFFFFU;
    }

    for (uint8_t reading = 0; reading < CALIBRATION_READS_PER_STEP; ++reading)
    {
        uint16_t raw[8];
        qtr_.read(raw);
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (raw[i] < stepMinimum[i])
            {
                stepMinimum[i] = raw[i];
            }
            if (raw[i] > stepMaximum[i])
            {
                stepMaximum[i] = raw[i];
            }
        }
    }

    for (uint8_t i = 0; i < 8; ++i)
    {
        if (stepMinimum[i] > maximum[i])
        {
            maximum[i] = stepMinimum[i];
        }
        if (stepMaximum[i] < minimum[i])
        {
            minimum[i] = stepMaximum[i];
        }
    }
}

void LineSensor::prepararADC() const
{
#if defined(ARDUINO_ARCH_ESP32)
    if (type_ == QTR_8A)
    {
        analogReadResolution(board::ADC_RESOLUTION_BITS);
    }
#endif
}

void LineSensor::readRawPhysical(uint16_t output[8])
{
    if (!begun_)
    {
        memset(output, 0, sizeof(uint16_t) * 8);
        return;
    }
    if (type_ == QTR_8A)
    {
        readAnalogRaw(output);
    }
    else
    {
        qtr_.read(output);
    }
}

void LineSensor::reverseSensorOrder(uint16_t values[8])
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        const uint16_t temporary = values[i];
        values[i] = values[7 - i];
        values[7 - i] = temporary;
    }
}

void LineSensor::readRaw(uint16_t output[8])
{
    readRawPhysical(output);
    if (mirrored_)
    {
        reverseSensorOrder(output);
    }
}

void LineSensor::calibrateRaw(const uint16_t raw[8], uint16_t output[8]) const
{
    if (!calibrationInitialized())
    {
        memset(output, 0, sizeof(uint16_t) * 8);
        return;
    }
    for (uint8_t i = 0; i < 8; ++i)
    {
        const uint16_t minimum =
            type_ == QTR_8A ? analogMinimum_[i] : qtr_.calibrationOn.minimum[i];
        const uint16_t maximum =
            type_ == QTR_8A ? analogMaximum_[i] : qtr_.calibrationOn.maximum[i];
        if (maximum <= minimum)
        {
            output[i] = 0;
            continue;
        }
        long value = (static_cast<long>(raw[i]) - minimum) * 1000L / (maximum - minimum);
        output[i] = static_cast<uint16_t>(value < 0 ? 0 : (value > 1000 ? 1000 : value));
    }
}

void LineSensor::readCalibrated(uint16_t output[8])
{
    uint16_t raw[8];
    readRawPhysical(raw);
    calibrateRaw(raw, output);
    if (mirrored_)
    {
        reverseSensorOrder(output);
    }
}

uint16_t LineSensor::positionFrom(const uint16_t calibrated[8], bool& valid)
{
    uint32_t weighted = 0;
    uint16_t sum = 0;
    valid = false;
    for (uint8_t i = 0; i < 8; ++i)
    {
        uint16_t value = lineType_ == LINHA_PRETA ? calibrated[i] : 1000U - calibrated[i];
        if (value > 200)
        {
            valid = true;
        }
        if (value > 50)
        {
            weighted += static_cast<uint32_t>(value) * i * 1000U;
            sum += value;
        }
    }
    if (!valid || sum == 0)
    {
        return lastValidPosition_ < 3500 ? 0 : 7000;
    }
    return static_cast<uint16_t>(weighted / sum);
}

LeituraLinha LineSensor::read()
{
    LeituraLinha reading{};
    readRawPhysical(reading.bruto); // unica aquisicao fisica
    calibrateRaw(reading.bruto, reading.calibrado);
    if (mirrored_)
    {
        reverseSensorOrder(reading.bruto);
        reverseSensorOrder(reading.calibrado);
    }
    reading.mascara = lineMask(reading.calibrado, threshold_, lineType_);
    reading.caso = static_cast<PadraoLinha>(reading.mascara);
    linePattern(reading.mascara, reading.padrao);
    reading.quantidadePretos = countBits(reading.mascara);
    reading.posicao = positionFrom(reading.calibrado, reading.posicaoValida);
    reading.detectada = reading.posicaoValida;
    reading.timestampMs = millis();
    if (reading.detectada)
    {
        lastDetectionMs_ = reading.timestampMs;
        if (reading.posicaoValida)
        {
            lastValidPosition_ = reading.posicao;
        }
    }
    reading.tempoSemLinhaMs =
        elapsedWithoutDetection(reading.timestampMs, lastDetectionMs_, reading.detectada);
    reading.ultimaPosicaoValida = lastValidPosition_;
    updateHistory(reading.caso, reading.timestampMs);
    last_ = reading;
    return reading;
}

bool LineSensor::calibrateMs(uint32_t durationMs)
{
    if (!begun_ || durationMs == 0)
    {
        return false;
    }

    uint16_t candidateMinimum[8];
    uint16_t candidateMaximum[8]{};
    for (uint8_t i = 0; i < 8; ++i)
    {
        candidateMinimum[i] = type_ == QTR_8A ? board::ADC_MAX_VALUE : 0xFFFFU;
    }

    const uint32_t start = millis();
    do
    {
        calibrateStep(candidateMinimum, candidateMaximum);
        yield();
    } while (millis() - start < durationMs);

    if (!validCalibrationRanges(candidateMinimum, candidateMaximum) ||
        !setCalibration(candidateMinimum, candidateMaximum))
    {
        return false;
    }
    resetReadState();
    return true;
}

bool LineSensor::calibrateSamples(uint16_t samples)
{
    if (!begun_ || samples == 0)
    {
        return false;
    }

    uint16_t candidateMinimum[8];
    uint16_t candidateMaximum[8]{};
    for (uint8_t i = 0; i < 8; ++i)
    {
        candidateMinimum[i] = type_ == QTR_8A ? board::ADC_MAX_VALUE : 0xFFFFU;
    }

    for (uint16_t i = 0; i < samples; ++i)
    {
        calibrateStep(candidateMinimum, candidateMaximum);
        yield();
    }

    if (!validCalibrationRanges(candidateMinimum, candidateMaximum) ||
        !setCalibration(candidateMinimum, candidateMaximum))
    {
        return false;
    }
    resetReadState();
    return true;
}

bool LineSensor::getCalibration(uint16_t minimum[8], uint16_t maximum[8]) const
{
    if (!calibrationInitialized())
    {
        return false;
    }
    const uint16_t* sourceMinimum =
        type_ == QTR_8A ? analogMinimum_ : qtr_.calibrationOn.minimum;
    const uint16_t* sourceMaximum =
        type_ == QTR_8A ? analogMaximum_ : qtr_.calibrationOn.maximum;
    memcpy(minimum, sourceMinimum, sizeof(uint16_t) * 8);
    memcpy(maximum, sourceMaximum, sizeof(uint16_t) * 8);
    return true;
}

bool LineSensor::setCalibration(const uint16_t minimum[8], const uint16_t maximum[8])
{
    if (!begun_)
    {
        return false;
    }
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (minimum[i] >= maximum[i] ||
            (type_ == QTR_8A && maximum[i] > board::ADC_MAX_VALUE))
        {
            return false;
        }
    }

    if (type_ == QTR_8A)
    {
        memcpy(analogMinimum_, minimum, sizeof(analogMinimum_));
        memcpy(analogMaximum_, maximum, sizeof(analogMaximum_));
        analogCalibrationInitialized_ = true;
        return true;
    }

    if (!qtr_.calibrationOn.initialized)
    {
        qtr_.calibrate();
    }
    memcpy(qtr_.calibrationOn.minimum, minimum, sizeof(uint16_t) * 8);
    memcpy(qtr_.calibrationOn.maximum, maximum, sizeof(uint16_t) * 8);
    qtr_.calibrationOn.initialized = true;
    return true;
}

void LineSensor::setMirrored(bool mirrored)
{
    if (mirrored_ == mirrored)
    {
        return;
    }
    mirrored_ = mirrored;
    resetReadState();
}

void LineSensor::resetCalibrationExtremes()
{
    qtr_.resetCalibration();
    resetAnalogCalibration();
}

void LineSensor::resetReadState()
{
    last_ = {};
    linePattern(0, last_.padrao);
    last_.ultimaPosicaoValida = 3500;
    lastValidPosition_ = 3500;
    lastDetectionMs_ = millis();
    clearHistory();
}

uint32_t LineSensor::timeWithoutLine() const
{
    if (last_.detectada)
    {
        return 0;
    }
    return elapsedWithoutDetection(millis(), lastDetectionMs_, false);
}

void LineSensor::resetCalibration()
{
    resetCalibrationExtremes();
    resetReadState();
}

void LineSensor::setHistoryCapacity(uint8_t capacity)
{
    historyCapacity_ = capacity > MAX_HISTORICO_LINHA ? MAX_HISTORICO_LINHA : capacity;
    clearHistory();
}

void LineSensor::clearHistory()
{
    memset(history_, 0, sizeof(history_));
    historyCount_ = 0;
    historyNext_ = 0;
}

uint8_t LineSensor::historyIndexFromNewest(uint8_t offset) const
{
    return static_cast<uint8_t>((historyNext_ + historyCapacity_ - 1U - offset) % historyCapacity_);
}

void LineSensor::updateHistory(PadraoLinha pattern, uint32_t nowMs)
{
    if (historyCapacity_ == 0)
    {
        return;
    }

    if (historyCount_ > 0)
    {
        CasoLinha& current = history_[historyIndexFromNewest(0)];
        current.duracaoMs = nowMs - current.inicioMs;
        if (current.padrao == pattern)
        {
            return;
        }
    }

    history_[historyNext_] = {pattern, nowMs, 0};
    historyNext_ = static_cast<uint8_t>((historyNext_ + 1U) % historyCapacity_);
    if (historyCount_ < historyCapacity_)
    {
        ++historyCount_;
    }
}

uint8_t LineSensor::copyRecentPatterns(PadraoLinha output[], uint8_t maximum) const
{
    if (output == nullptr || maximum == 0 || historyCount_ == 0)
    {
        return 0;
    }

    const uint8_t count = maximum < historyCount_ ? maximum : historyCount_;
    for (uint8_t i = 0; i < count; ++i)
    {
        output[i] = history_[historyIndexFromNewest(i)].padrao;
    }
    return count;
}

uint8_t LineSensor::copyHistory(CasoLinha output[], uint8_t maximum) const
{
    if (output == nullptr || maximum == 0 || historyCount_ == 0)
    {
        return 0;
    }

    const uint8_t count = maximum < historyCount_ ? maximum : historyCount_;
    for (uint8_t i = 0; i < count; ++i)
    {
        output[i] = history_[historyIndexFromNewest(i)];
    }

    // O caso mais recente ainda esta ativo. Atualizamos a copia para que sua
    // duracao represente o instante da consulta, sem alterar o buffer em metodo const.
    output[0].duracaoMs = millis() - output[0].inicioMs;
    return count;
}

} // namespace robos3
