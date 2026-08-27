#pragma once

#include <Arduino.h>
#include "../types/RoboTypes.h"

namespace robos3
{

/** @brief Quantidade maxima de botoes cadastrados sem alocacao dinamica. */
constexpr uint8_t MAX_BOTOES = 12;

/** @brief Gerencia botoes digitais e analogicos com debounce temporal. */
class ButtonManager
{
  public:
    /** @brief Cria um gerenciador vazio. */
    ButtonManager();

    /**
     * @brief Cadastra um botao digital entre o GPIO e GND.
     * @param id ID unico de 0 a 254.
     * @param pin GPIO digital do ESP32-S3, de 0 a 48, configurado como INPUT_PULLUP.
     * @param debounceMs Tempo estavel exigido para aceitar uma transicao; padrao 30 ms.
     * @return false para ID/pino duplicado, ID ou pino 255 ou falta de slot.
     */
    bool addDigital(uint8_t id, uint8_t pin, uint16_t debounceMs);

    /**
     * @brief Cadastra uma tecla de um conjunto resistivo em uma entrada ADC.
     * @param id ID unico de 0 a 254.
     * @param pin GPIO ADC do ESP32-S3, de 1 a 20, compartilhado pelo conjunto.
     * @param minimum Menor valor ADC que representa esta tecla, de 0 a 1023.
     * @param maximum Maior valor ADC que representa esta tecla, de 0 a 1023.
     * @param debounceMs Tempo estavel exigido para aceitar uma transicao.
     * @return false para faixa invalida/sobreposta, conflito de pino, ID duplicado ou falta de slot.
     */
    bool addAnalog(uint8_t id, uint8_t pin, uint16_t minimum, uint16_t maximum,
                   uint16_t debounceMs);

    /** @brief Atualiza todos os estados e acumula eventos ainda nao consultados. */
    void update();

    /** @brief Consulta sem consumir eventos. @param id ID cadastrado. @return Estado completo. */
    LeituraBotao read(uint8_t id);

    /** @brief Consulta o estado estavel. @param id ID cadastrado. @return true enquanto pressionado. */
    bool pressed(uint8_t id);

    /** @brief Consome um aperto. @param id ID cadastrado. @return true uma vez por aperto. */
    bool consumePressed(uint8_t id);

    /** @brief Consome uma soltura. @param id ID cadastrado. @return true uma vez por soltura. */
    bool consumeReleased(uint8_t id);

    /**
     * @brief Testa a duracao do pressionamento atual.
     * @param id ID cadastrado. @param durationMs Duracao minima em milissegundos.
     * @return true enquanto o botao permanecer pressionado pelo tempo informado.
     */
    bool heldFor(uint8_t id, uint32_t durationMs);

    /** @brief Verifica um cadastro. @param id ID cadastrado. @return true quando existe. */
    bool available(uint8_t id) const;

    /** @brief Exclui um cadastro. @param id ID cadastrado. @return true quando removido. */
    bool remove(uint8_t id);

    /** @brief Remove todos os cadastros e eventos de botoes. */
    void clear();

  private:
    struct Slot
    {
        bool used;
        uint8_t id;
        TipoBotao type;
        uint8_t pin;
        uint16_t minimum;
        uint16_t maximum;
        uint16_t debounceMs;
        uint16_t rawValue;
        bool rawPressed;
        bool stablePressed;
        bool pressedEvent;
        bool releasedEvent;
        uint32_t rawChangedMs;
        uint32_t pressedSinceMs;
        uint32_t lastPressDurationMs;
        uint32_t pressCount;
    };

    int8_t indexOf(uint8_t id) const;
    bool rawPressed(Slot& slot);
    void initializeSlot(Slot& slot);

    Slot slots_[MAX_BOTOES];
};

} // namespace robos3
