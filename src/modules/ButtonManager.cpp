#include "ButtonManager.h"

#include <string.h>

namespace robos3
{

ButtonManager::ButtonManager() : slots_{}
{
}

int8_t ButtonManager::indexOf(uint8_t id) const
{
    for (uint8_t i = 0; i < MAX_BOTOES; ++i)
    {
        if (slots_[i].used && slots_[i].id == id)
        {
            return static_cast<int8_t>(i);
        }
    }
    return -1;
}

bool ButtonManager::rawPressed(Slot& slot)
{
    if (slot.type == BOTAO_DIGITAL_PULLUP)
    {
        slot.rawValue = digitalRead(slot.pin);
        return slot.rawValue == LOW;
    }
    slot.rawValue = analogRead(slot.pin);
    return slot.rawValue >= slot.minimum && slot.rawValue <= slot.maximum;
}

void ButtonManager::initializeSlot(Slot& slot)
{
    pinMode(slot.pin, slot.type == BOTAO_DIGITAL_PULLUP ? INPUT_PULLUP : INPUT);
    slot.rawPressed = rawPressed(slot);
    slot.stablePressed = slot.rawPressed;
    slot.rawChangedMs = millis();
    slot.pressedSinceMs = slot.stablePressed ? slot.rawChangedMs : 0;
}

bool ButtonManager::addDigital(uint8_t id, uint8_t pin, uint16_t debounceMs)
{
    if (id == 255 || pin > 48 || indexOf(id) >= 0)
    {
        return false;
    }
    for (uint8_t i = 0; i < MAX_BOTOES; ++i)
    {
        if (slots_[i].used && slots_[i].pin == pin)
        {
            return false;
        }
    }
    for (uint8_t i = 0; i < MAX_BOTOES; ++i)
    {
        if (!slots_[i].used)
        {
            slots_[i] = {true, id, BOTAO_DIGITAL_PULLUP, pin, 0, 0, debounceMs};
            initializeSlot(slots_[i]);
            return true;
        }
    }
    return false;
}

bool ButtonManager::addAnalog(uint8_t id, uint8_t pin, uint16_t minimum, uint16_t maximum,
                              uint16_t debounceMs)
{
    if (id == 255 || pin < 1 || pin > 20 || indexOf(id) >= 0 || minimum > maximum ||
        maximum > 1023)
    {
        return false;
    }
#if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(10);
#endif
    for (uint8_t i = 0; i < MAX_BOTOES; ++i)
    {
        if (slots_[i].used && slots_[i].pin == pin)
        {
            if (slots_[i].type != BOTAO_ANALOGICO)
            {
                return false;
            }
            const bool sobreposto =
                !(maximum < slots_[i].minimum || minimum > slots_[i].maximum);
            if (sobreposto)
            {
                return false;
            }
        }
    }
    for (uint8_t i = 0; i < MAX_BOTOES; ++i)
    {
        if (!slots_[i].used)
        {
            slots_[i] = {true, id, BOTAO_ANALOGICO, pin, minimum, maximum, debounceMs};
            initializeSlot(slots_[i]);
            return true;
        }
    }
    return false;
}

void ButtonManager::update()
{
    const uint32_t now = millis();
    for (uint8_t i = 0; i < MAX_BOTOES; ++i)
    {
        Slot& slot = slots_[i];
        if (!slot.used)
        {
            continue;
        }
        const bool raw = rawPressed(slot);
        if (raw != slot.rawPressed)
        {
            slot.rawPressed = raw;
            slot.rawChangedMs = now;
        }
        if (raw == slot.stablePressed || now - slot.rawChangedMs < slot.debounceMs)
        {
            continue;
        }
        slot.stablePressed = raw;
        if (raw)
        {
            slot.pressedEvent = true;
            slot.pressedSinceMs = now;
            ++slot.pressCount;
        }
        else
        {
            slot.releasedEvent = true;
            slot.lastPressDurationMs = now - slot.pressedSinceMs;
            slot.pressedSinceMs = 0;
        }
    }
}

LeituraBotao ButtonManager::read(uint8_t id)
{
    update();
    LeituraBotao reading{};
    reading.id = id;
    const int8_t index = indexOf(id);
    if (index < 0)
    {
        return reading;
    }
    const Slot& slot = slots_[index];
    reading.tipo = slot.type;
    reading.pino = slot.pin;
    reading.valorBruto = slot.rawValue;
    reading.valido = true;
    reading.pressionado = slot.stablePressed;
    reading.eventoApertado = slot.pressedEvent;
    reading.eventoSolto = slot.releasedEvent;
    reading.duracaoPressionadoMs = slot.stablePressed ? millis() - slot.pressedSinceMs : 0;
    reading.duracaoUltimoApertoMs = slot.lastPressDurationMs;
    reading.quantidadeApertos = slot.pressCount;
    return reading;
}

bool ButtonManager::pressed(uint8_t id)
{
    return read(id).pressionado;
}

bool ButtonManager::consumePressed(uint8_t id)
{
    update();
    const int8_t index = indexOf(id);
    if (index < 0 || !slots_[index].pressedEvent)
    {
        return false;
    }
    slots_[index].pressedEvent = false;
    return true;
}

bool ButtonManager::consumeReleased(uint8_t id)
{
    update();
    const int8_t index = indexOf(id);
    if (index < 0 || !slots_[index].releasedEvent)
    {
        return false;
    }
    slots_[index].releasedEvent = false;
    return true;
}

bool ButtonManager::heldFor(uint8_t id, uint32_t durationMs)
{
    const LeituraBotao reading = read(id);
    return reading.valido && reading.pressionado && reading.duracaoPressionadoMs >= durationMs;
}

bool ButtonManager::available(uint8_t id) const
{
    return indexOf(id) >= 0;
}

bool ButtonManager::remove(uint8_t id)
{
    const int8_t index = indexOf(id);
    if (index < 0)
    {
        return false;
    }
    slots_[index] = {};
    return true;
}

void ButtonManager::clear()
{
    memset(slots_, 0, sizeof(slots_));
}

} // namespace robos3
