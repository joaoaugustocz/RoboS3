#include "LineSensor.h"

#include <Arduino.h>
#include <string.h>
#include "RoboMath.h"

namespace robos3
{

LineSensor::LineSensor()
    : lineType_(LINHA_PRETA), threshold_(500), last_{}, lastDetectionMs_(0),
      lastValidPosition_(3500), begun_(false), history_{}, historyCapacity_(8), historyCount_(0),
      historyNext_(0)
{
    linePattern(0, last_.padrao);
    last_.ultimaPosicaoValida = 3500;
}

bool LineSensor::begin(const uint8_t pins[8], TipoQTR type, uint8_t emitterPin)
{
    if (type == QTR_8A)
    {
        qtr_.setTypeAnalog();
#if defined(ARDUINO_ARCH_ESP32)
        analogReadResolution(10); // QTRSensors 4.0.0 usa maximo analogico 1023.
#endif
    }
    else
    {
        qtr_.setTypeRC();
    }
    qtr_.setSensorPins(pins, 8);
    if (emitterPin != 0xFF)
    {
        qtr_.setEmitterPin(emitterPin);
    }
    begun_ = true;
    lastDetectionMs_ = millis();
    clearHistory();
    return true;
}

void LineSensor::readRaw(uint16_t output[8])
{
    if (!begun_)
    {
        memset(output, 0, sizeof(uint16_t) * 8);
        return;
    }
    qtr_.read(output);
}

void LineSensor::calibrateRaw(const uint16_t raw[8], uint16_t output[8]) const
{
    if (!qtr_.calibrationOn.initialized)
    {
        memset(output, 0, sizeof(uint16_t) * 8);
        return;
    }
    for (uint8_t i = 0; i < 8; ++i)
    {
        const uint16_t minimum = qtr_.calibrationOn.minimum[i];
        const uint16_t maximum = qtr_.calibrationOn.maximum[i];
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
    readRaw(raw);
    calibrateRaw(raw, output);
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
    readRaw(reading.bruto); // unica aquisicao fisica
    calibrateRaw(reading.bruto, reading.calibrado);
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

void LineSensor::calibrateMs(uint32_t durationMs)
{
    if (!begun_)
    {
        return;
    }
    const uint32_t start = millis();
    do
    {
        qtr_.calibrate();
        yield();
    } while (millis() - start < durationMs);
}

void LineSensor::calibrateSamples(uint16_t samples)
{
    if (!begun_)
    {
        return;
    }
    for (uint16_t i = 0; i < samples; ++i)
    {
        qtr_.calibrate();
        yield();
    }
}

bool LineSensor::getCalibration(uint16_t minimum[8], uint16_t maximum[8]) const
{
    if (!qtr_.calibrationOn.initialized)
    {
        return false;
    }
    memcpy(minimum, qtr_.calibrationOn.minimum, sizeof(uint16_t) * 8);
    memcpy(maximum, qtr_.calibrationOn.maximum, sizeof(uint16_t) * 8);
    return true;
}

bool LineSensor::setCalibration(const uint16_t minimum[8], const uint16_t maximum[8])
{
    if (!begun_)
    {
        return false;
    }
    if (!qtr_.calibrationOn.initialized)
    {
        qtr_.calibrate();
    }
    if (!qtr_.calibrationOn.initialized)
    {
        return false;
    }
    memcpy(qtr_.calibrationOn.minimum, minimum, sizeof(uint16_t) * 8);
    memcpy(qtr_.calibrationOn.maximum, maximum, sizeof(uint16_t) * 8);
    return true;
}

uint32_t LineSensor::timeWithoutLine() const
{
    if (last_.detectada)
    {
        return 0;
    }
    return elapsedWithoutDetection(millis(), lastDetectionMs_, false);
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
