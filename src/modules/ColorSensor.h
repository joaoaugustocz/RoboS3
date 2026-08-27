#pragma once

#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include "../types/RoboTypes.h"

namespace robos3
{

/** @brief Driver interno de um sensor de cor TCS34725. */
class ColorSensor
{
  public:
    /** @brief Cria o driver com faixas HSV padrao. */
    ColorSensor();

    /**
     * @brief Detecta e inicializa o TCS34725 no barramento informado.
     * @param wire Barramento I2C ja configurado.
     * @return true quando o sensor respondeu no endereco esperado.
     */
    bool begin(TwoWire& wire);

    /** @brief Define o ganho analogico. @param gain Ganho usado nas proximas leituras. */
    void setGain(GanhoCor gain);

    /** @brief Define a integracao. @param time Tempo usado nas proximas leituras. */
    void setIntegrationTime(TempoIntegracaoCor time);

    /** @brief Restaura ganho 4x, integracao 50 ms e todas as faixas HSV padrao. */
    void resetConfiguration();

    /** @return Ganho atualmente configurado. */
    GanhoCor gain() const
    {
        return gain_;
    }
    /** @return Tempo de integracao atualmente configurado. */
    TempoIntegracaoCor integrationTime() const
    {
        return integration_;
    }
    /** @return Leitura completa RGB, HSV, HSL, crua e classificada. */
    LeituraCor read();

    /** @brief Define uma faixa. @param color Classe de cor. @param range Limites HSV. */
    void setRange(CorDetectada color, const FaixaHSV& range);

    /** @param color Classe de cor. @return Referencia para sua faixa HSV. */
    const FaixaHSV& range(CorDetectada color) const;

    /** @param reading Leitura valida a classificar. @return Cor ou COR_DESCONHECIDA. */
    CorDetectada classify(const LeituraCor& reading) const;

    /** @return true quando o TCS34725 respondeu ao begin(). */
    bool available() const
    {
        return available_;
    }

  private:
    static tcs34725Gain_t nativeGain(GanhoCor gain);
    static uint8_t nativeIntegration(TempoIntegracaoCor time);
    void loadDefaultRanges();

    Adafruit_TCS34725 sensor_;
    GanhoCor gain_;
    TempoIntegracaoCor integration_;
    FaixaHSV ranges_[COR_TOTAL];
    bool available_;
};

} // namespace robos3
