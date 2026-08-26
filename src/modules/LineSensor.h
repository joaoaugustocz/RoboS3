#pragma once

#include <QTRSensors.h>
#include "../types/RoboTypes.h"

namespace robos3
{

/**
 * @brief Driver interno dos oito sensores QTR e do historico de padroes.
 *
 * Uma chamada a read() gera dados brutos, calibrados, binarios e posicionais
 * a partir de uma unica aquisicao fisica.
 */
class LineSensor
{
  public:
    /** @brief Cria o driver com linha preta, limiar 500 e historico de 8 casos. */
    LineSensor();

    /**
     * @brief Configura os oito sensores QTR.
     * @param pins GPIOs na ordem fisica S1 ate S8.
     * @param type Tipo eletrico QTR_8A ou QTR_8RC.
     * @param emitterPin GPIO de controle do emissor ou 0xFF quando ausente.
     * @return true quando a configuracao foi aceita.
     */
    bool begin(const uint8_t pins[8], TipoQTR type, uint8_t emitterPin = 0xFF);

    /** @brief Seleciona a polaridade da pista. @param type LINHA_PRETA ou LINHA_BRANCA. */
    void setTypeLine(TipoLinha type)
    {
        lineType_ = type;
    }
    /** @brief Define o limiar binario. @param threshold Valor limitado a 0...1000. */
    void setThreshold(uint16_t threshold)
    {
        threshold_ = threshold > 1000 ? 1000 : threshold;
    }
    /** @return Limiar binario atual, entre 0 e 1000. */
    uint16_t threshold() const
    {
        return threshold_;
    }
    /**
     * @brief Faz uma aquisicao completa e atualiza o historico.
     * @return Dados brutos, calibrados, binarios, posicionais e temporais.
     */
    LeituraLinha read();

    /** @brief Faz leitura bruta. @param output Vetor de oito valores na ordem S1...S8. */
    void readRaw(uint16_t output[8]);

    /** @brief Faz leitura calibrada. @param output Vetor de oito valores em 0...1000. */
    void readCalibrated(uint16_t output[8]);

    /** @brief Executa calibracao bloqueante. @param durationMs Duracao em milissegundos. */
    void calibrateMs(uint32_t durationMs);

    /** @brief Executa calibracao bloqueante. @param samples Quantidade de amostras. */
    void calibrateSamples(uint16_t samples);

    /**
     * @brief Copia os extremos da calibracao atual.
     * @param minimum Vetor que recebe os oito minimos.
     * @param maximum Vetor que recebe os oito maximos.
     * @return false quando ainda nao existe calibracao valida.
     */
    bool getCalibration(uint16_t minimum[8], uint16_t maximum[8]) const;

    /**
     * @brief Aplica extremos de calibracao fornecidos pelo usuario.
     * @param minimum Vetor com os oito minimos.
     * @param maximum Vetor com os oito maximos.
     * @return false se algum minimo nao for menor que o maximo correspondente.
     */
    bool setCalibration(const uint16_t minimum[8], const uint16_t maximum[8]);

    /** @brief Configura e limpa o historico. @param capacity Capacidade de 0 a 32. */
    void setHistoryCapacity(uint8_t capacity);

    /** @brief Remove todos os casos registrados. */
    void clearHistory();

    /** @return Capacidade configurada do historico; zero significa desativado. */
    uint8_t historyCapacity() const
    {
        return historyCapacity_;
    }
    /** @return Quantidade de casos validos presentes no buffer circular. */
    uint8_t historyCount() const
    {
        return historyCount_;
    }
    /**
     * @brief Copia padroes do mais novo ao mais antigo.
     * @param output Vetor de destino fornecido pelo usuario.
     * @param maximum Quantidade de elementos disponiveis em output.
     * @return Quantidade efetivamente copiada para output.
     */
    uint8_t copyRecentPatterns(PadraoLinha output[], uint8_t maximum) const;

    /**
     * @brief Copia padroes e tempos do mais novo ao mais antigo.
     * @param output Vetor de destino fornecido pelo usuario.
     * @param maximum Quantidade de elementos disponiveis em output.
     * @return Quantidade efetivamente copiada para output.
     */
    uint8_t copyHistory(CasoLinha output[], uint8_t maximum) const;

    /** @return Referencia para a ultima leitura, sem nova aquisicao. */
    const LeituraLinha& last() const
    {
        return last_;
    }
    /** @return Milissegundos decorridos desde a ultima deteccao valida. */
    uint32_t timeWithoutLine() const;

    /** @return true depois que begin() configurou o QTR. */
    bool available() const
    {
        return begun_;
    }

  private:
    void calibrateRaw(const uint16_t raw[8], uint16_t output[8]) const;
    uint16_t positionFrom(const uint16_t calibrated[8], bool& valid);
    void updateHistory(PadraoLinha pattern, uint32_t nowMs);
    uint8_t historyIndexFromNewest(uint8_t offset) const;

    QTRSensors qtr_;
    TipoLinha lineType_;
    uint16_t threshold_;
    LeituraLinha last_;
    uint32_t lastDetectionMs_;
    uint16_t lastValidPosition_;
    bool begun_;
    CasoLinha history_[MAX_HISTORICO_LINHA];
    uint8_t historyCapacity_;
    uint8_t historyCount_;
    uint8_t historyNext_;
};

} // namespace robos3
