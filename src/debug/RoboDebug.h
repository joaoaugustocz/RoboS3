#pragma once

#include <Arduino.h>
#include "../types/RoboTypes.h"

#ifndef ROBOS3_DEBUG_LEVEL
#ifdef ROBOS3_DEBUG
#define ROBOS3_DEBUG_LEVEL 2
#else
#define ROBOS3_DEBUG_LEVEL 0
#endif
#endif

namespace robos3
{

/** @brief Saida de diagnostico filtrada por nivel de severidade. */
class RoboDebug
{
  public:
    /** @brief Usa Serial e ROBOS3_DEBUG_LEVEL como configuracao inicial. */
    RoboDebug() : output_(&Serial), level_(static_cast<DebugLevel>(ROBOS3_DEBUG_LEVEL))
    {
    }
    /** @brief Redireciona as mensagens. @param output Novo destino Print. */
    void setOutput(Print& output)
    {
        output_ = &output;
    }
    /** @brief Define o filtro. @param level Menor severidade que sera impressa. */
    void setLevel(DebugLevel level)
    {
        level_ = level;
    }
    /** @return Nivel de diagnostico atualmente configurado. */
    DebugLevel level() const
    {
        return level_;
    }
    /** @brief Registra uma mensagem. @param level Severidade. @param message Texto. */
    void log(DebugLevel level, const char* message) const;
    /** @brief Registra um modulo. @param name Nome. @param ok Estado de inicializacao. */
    void module(const char* name, bool ok) const;

  private:
    Print* output_;
    DebugLevel level_;
};

} // namespace robos3
