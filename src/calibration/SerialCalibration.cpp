#include "SerialCalibration.h"
#include "CalibrationManager.h"
#include "../RoboS3.h"
#include <string.h>
#include <stdlib.h>

namespace robos3
{

namespace
{
void printCommandResult(Print& out, bool ok, const __FlashStringHelper* success,
                        const __FlashStringHelper* failure)
{
    out.print(ok ? F("[RoboS3][OK] ") : F("[RoboS3][ERRO] "));
    out.println(ok ? success : failure);
}

void printCommandSuccess(Print& out, const __FlashStringHelper* message)
{
    out.print(F("[RoboS3][OK] "));
    out.println(message);
}

void printCommandInfo(Print& out, const __FlashStringHelper* message)
{
    out.print(F("[RoboS3][INFO] "));
    out.println(message);
}

void printCommandError(Print& out, const __FlashStringHelper* message)
{
    out.print(F("[RoboS3][ERRO] "));
    out.println(message);
}
} // namespace

SerialCalibration::SerialCalibration(CalibrationManager& m)
    : manager_(m), stream_(nullptr), buffer_{}, length_(0)
{
}
void SerialCalibration::begin(Stream& s)
{
    stream_ = &s;
    stream_->println();
    stream_->println(F("======================================================================"));
    stream_->println(F("RoboS3 | INTERFACE SERIAL DE CALIBRACAO"));
    stream_->println(F("======================================================================"));
    stream_->println(F("Digite 'help' para ver comandos e formatos."));
    stream_->println();
}
void SerialCalibration::update()
{
    if (!stream_)
    {
        return;
    }
    while (stream_->available())
    {
        char c = stream_->read();
        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            buffer_[length_] = '\0';
            if (length_)
            {
                execute(buffer_);
            }
            length_ = 0;
        }
        else if (length_ < sizeof(buffer_) - 1)
        {
            buffer_[length_++] = c;
        }
    }
}
void SerialCalibration::help()
{
    stream_->println(F("COMANDOS DISPONIVEIS"));
    stream_->println(F("----------------------------------------------------------------------"));
    stream_->println(F("SISTEMA"));
    stream_->println(F("  help | status | save | load | reset"));
    stream_->println(F("LINHA"));
    stream_->println(F("  line read | line raw | line calibrated"));
    stream_->println(F("  line calibrate <ms> | line threshold <0-1000> | line save"));
    stream_->println(F("MOTORES"));
    stream_->println(F("  motor status | motor stop | motor brake | motor move <E> <D>"));
    stream_->println(F("  motor min <left|right> <forward|reverse> <0-100>"));
    stream_->println(F("  motor compensation <left|right> <forward|reverse> <fator>"));
    stream_->println(F("CORES"));
    stream_->println(F("  color read|rgb|hsv <left|right>"));
    stream_->println(F("  color calibrate <left|right> <color> [samples]"));
    stream_->println(F("  color hue <left|right> <color> <hmin> <hmax>"));
    stream_->println(F("  color hsv-range <left|right> <color>"));
    stream_->println(F("      <hmin> <hmax> <smin> <smax> <vmin> <vmax>"));
    stream_->println(F("  color gain <left|right> <1|4|16|60>"));
    stream_->println(F("  color integration <left|right> <2|24|50|101|154|600>"));
    stream_->println(F("IMU"));
    stream_->println(F("  imu read | imu status | imu zero | imu calibrate <ms>"));
    stream_->println(F("----------------------------------------------------------------------"));
    stream_->println(F("Lados: left/right. Sentidos: forward/reverse."));
    stream_->println(F("Cores: black, white, red, green, blue, yellow, orange, purple."));
}

static SensorCor parseSide(const char* s)
{
    return s && !strcmp(s, "right") ? COR_DIREITA : COR_ESQUERDA;
}
static Motor parseMotor(const char* s)
{
    return s && !strcmp(s, "right") ? MOTOR_DIREITO : MOTOR_ESQUERDO;
}
static SentidoMotor parseDirection(const char* s)
{
    return s && !strcmp(s, "reverse") ? TRAS : FRENTE;
}
static CorDetectada parseColor(const char* s)
{
    if (!s)
    {
        return COR_DESCONHECIDA;
    }
    const char* names[] = {"unknown", "black",  "white",  "red",   "green",
                           "blue",    "yellow", "orange", "purple"};
    for (uint8_t i = 0; i < COR_TOTAL; ++i)
    {
        if (!strcmp(s, names[i]))
        {
            return static_cast<CorDetectada>(i);
        }
    }
    return COR_DESCONHECIDA;
}
static GanhoCor parseGain(int value)
{
    return value >= 60 ? GANHO_60X : value >= 16 ? GANHO_16X : value >= 4 ? GANHO_4X : GANHO_1X;
}
static TempoIntegracaoCor parseIntegration(int value)
{
    return value >= 600   ? INTEGRACAO_600MS
           : value >= 154 ? INTEGRACAO_154MS
           : value >= 101 ? INTEGRACAO_101MS
           : value >= 50  ? INTEGRACAO_50MS
           : value >= 24  ? INTEGRACAO_24MS
                          : INTEGRACAO_2_4MS;
}

void SerialCalibration::execute(char* line)
{
    RoboS3& robo = manager_.robo();
    char* cmd = strtok(line, " ");
    if (!cmd)
    {
        return;
    }
    if (!strcmp(cmd, "help"))
    {
        help();
        return;
    }
    if (!strcmp(cmd, "status"))
    {
        robo.printStatus(*stream_);
        return;
    }
    if (!strcmp(cmd, "save") || !strcmp(cmd, "load") || !strcmp(cmd, "reset"))
    {
        bool ok = !strcmp(cmd, "save")   ? manager_.save()
                  : !strcmp(cmd, "load") ? manager_.load()
                                         : manager_.reset();
        const __FlashStringHelper* success =
            !strcmp(cmd, "save")   ? F("Calibracao salva na NVS.")
            : !strcmp(cmd, "load") ? F("Calibracao carregada da NVS.")
                                    : F("Calibracao restaurada para os padroes.");
        const __FlashStringHelper* failure =
            !strcmp(cmd, "save")   ? F("Falha ao salvar calibracao na NVS.")
            : !strcmp(cmd, "load") ? F("Nao ha calibracao valida para carregar.")
                                    : F("NVS nao apagada; padroes aplicados apenas em memoria.");
        printCommandResult(*stream_, ok, success, failure);
        return;
    }
    if (!strcmp(cmd, "line"))
    {
        char* sub = strtok(nullptr, " ");
        if (!sub)
        {
            return;
        }
        if (!strcmp(sub, "read"))
        {
            robo.lerLinha();
            robo.printLinha(*stream_);
        }
        else if (!strcmp(sub, "raw") || !strcmp(sub, "calibrated"))
        {
            uint16_t v[8];
            if (!strcmp(sub, "raw"))
            {
                robo.lerLinhaBruta(v);
                stream_->print(F("[RoboS3][QTR] Bruto      S1..S8: "));
            }
            else
            {
                robo.lerLinhaCalibrada(v);
                stream_->print(F("[RoboS3][QTR] Calibrado  S1..S8: "));
            }
            stream_->print('[');
            for (uint8_t i = 0; i < 8; ++i)
            {
                stream_->print(v[i]);
                if (i < 7)
                {
                    stream_->print(F(", "));
                }
            }
            stream_->print(']');
            stream_->println();
        }
        else if (!strcmp(sub, "calibrate"))
        {
            char* v = strtok(nullptr, " ");
            const bool calibrated = manager_.calibrateLine(v ? strtoul(v, nullptr, 10) : 5000);
            printCommandResult(*stream_, calibrated,
                               F("Calibracao QTR concluida e aplicada."),
                               F("Faixa invalida; calibracao anterior mantida, se existente."));
        }
        else if (!strcmp(sub, "threshold"))
        {
            char* v = strtok(nullptr, " ");
            if (v)
            {
                robo.configurarLimiarLinha(atoi(v));
                stream_->print(F("[RoboS3][OK] Limiar do QTR atualizado (0..1000): "));
                stream_->println(atoi(v));
            }
        }
        else if (!strcmp(sub, "save"))
        {
            const bool saved = manager_.save();
            printCommandResult(*stream_, saved, F("Calibracao salva na NVS."),
                               F("Falha ao salvar calibracao na NVS."));
        }
        return;
    }
    if (!strcmp(cmd, "motor"))
    {
        char* sub = strtok(nullptr, " ");
        if (!sub)
        {
            return;
        }
        if (!strcmp(sub, "move"))
        {
            char *l = strtok(nullptr, " "), *r = strtok(nullptr, " ");
            if (l && r)
            {
                manager_.testMotors(atoi(l), atoi(r));
                printCommandInfo(*stream_, F("Teste ativo; motores param automaticamente em ate 2 s."));
            }
        }
        else if (!strcmp(sub, "stop"))
        {
            manager_.stopMotors();
            printCommandSuccess(*stream_, F("Motores parados."));
        }
        else if (!strcmp(sub, "brake"))
        {
            manager_.stopMotors(true);
            printCommandSuccess(*stream_, F("Freio aplicado."));
        }
        else if (!strcmp(sub, "status"))
        {
            robo.printMotores(*stream_);
        }
        else if (!strcmp(sub, "min") || !strcmp(sub, "compensation"))
        {
            char *m = strtok(nullptr, " "), *d = strtok(nullptr, " "), *v = strtok(nullptr, " ");
            if (m && d && v)
            {
                if (!strcmp(sub, "min"))
                {
                    robo.configurarMinimoMovimento(parseMotor(m), parseDirection(d), atoi(v));
                    printCommandSuccess(*stream_, F("Minimo de movimento atualizado."));
                }
                else
                {
                    robo.configurarCompensacaoMotor(parseMotor(m), parseDirection(d), atof(v));
                    printCommandSuccess(*stream_, F("Compensacao do motor atualizada."));
                }
            }
        }
        return;
    }
    if (!strcmp(cmd, "color"))
    {
        char *sub = strtok(nullptr, " "), *side = strtok(nullptr, " ");
        if (!sub || !side)
        {
            return;
        }
        SensorCor s = parseSide(side);
        if (!strcmp(sub, "read") || !strcmp(sub, "rgb") || !strcmp(sub, "hsv"))
        {
            LeituraCor r = robo.lerDadosCor(s);
            stream_->print(F("[RoboS3][COR]["));
            stream_->print(r.valida ? F("OK") : F("FALHA"));
            stream_->print(F("] Sensor "));
            stream_->print(s == COR_DIREITA ? F("direito") : F("esquerdo"));
            stream_->print(F(" | "));
            if (!strcmp(sub, "hsv"))
            {
                stream_->print(F("H="));
                stream_->print(r.hsv.h, 1);
                stream_->print(F(" deg | S="));
                stream_->print(r.hsv.s, 1);
                stream_->print(F("% | V="));
                stream_->print(r.hsv.v, 1);
                stream_->println('%');
            }
            else
            {
                stream_->print(F("R="));
                stream_->print(r.rgb.r);
                stream_->print(F(" G="));
                stream_->print(r.rgb.g);
                stream_->print(F(" B="));
                stream_->println(r.rgb.b);
            }
            return;
        }
        if (!strcmp(sub, "gain"))
        {
            char* v = strtok(nullptr, " ");
            if (v)
            {
                robo.setGanhoCor(s, parseGain(atoi(v)));
                printCommandSuccess(*stream_, F("Ganho do sensor de cor atualizado."));
            }
            return;
        }
        if (!strcmp(sub, "integration"))
        {
            char* v = strtok(nullptr, " ");
            if (v)
            {
                robo.setTempoIntegracaoCor(s, parseIntegration(atoi(v)));
                printCommandSuccess(*stream_, F("Tempo de integracao do sensor de cor atualizado."));
            }
            return;
        }
        char* name = strtok(nullptr, " ");
        CorDetectada c = parseColor(name);
        if (c == COR_DESCONHECIDA)
        {
            return;
        }
        if (!strcmp(sub, "calibrate"))
        {
            char* n = strtok(nullptr, " ");
            if (!robo.corDisponivel(s))
            {
                printCommandError(*stream_, F("Calibracao cancelada: sensor TCS34725 indisponivel."));
                return;
            }
            robo.calibrarCor(s, c, n ? atoi(n) : 30);
            printCommandInfo(*stream_, F("Faixa HSV calibrada; use 'save' para persistir."));
        }
        else if (!strcmp(sub, "hue"))
        {
            char *a = strtok(nullptr, " "), *b = strtok(nullptr, " ");
            if (a && b)
            {
                robo.definirFaixaH(s, c, atof(a), atof(b));
                printCommandSuccess(*stream_, F("Faixa de matiz atualizada."));
            }
        }
        else if (!strcmp(sub, "hsv-range"))
        {
            float v[6];
            for (uint8_t i = 0; i < 6; ++i)
            {
                char* t = strtok(nullptr, " ");
                if (!t)
                {
                    return;
                }
                v[i] = atof(t);
            }
            robo.definirFaixaHSV(s, c, v[0], v[1], v[2], v[3], v[4], v[5]);
            printCommandSuccess(*stream_, F("Faixa HSV atualizada."));
        }
        return;
    }
    if (!strcmp(cmd, "imu"))
    {
        char* sub = strtok(nullptr, " ");
        if (!sub)
        {
            return;
        }
        if (!strcmp(sub, "read"))
        {
            robo.lerIMU();
            robo.printIMU(*stream_);
        }
        else if (!strcmp(sub, "status"))
        {
            const bool available = robo.imuDisponivel();
            printCommandResult(*stream_, available, F("MPU6050 detectada na inicializacao."),
                               F("MPU6050 nao respondeu na inicializacao."));
        }
        else if (!strcmp(sub, "zero"))
        {
            robo.zerarAnguloZ();
            printCommandSuccess(*stream_, F("Angulo Z relativo zerado."));
        }
        else if (!strcmp(sub, "calibrate"))
        {
            char* v = strtok(nullptr, " ");
            if (!robo.imuDisponivel())
            {
                printCommandError(*stream_, F("Calibracao cancelada: MPU6050 indisponivel."));
                return;
            }
            manager_.calibrateImu(v ? strtoul(v, nullptr, 10) : 3000);
            printCommandInfo(*stream_, F("Offsets da IMU recalculados; use 'save' para persistir."));
        }
        return;
    }
    printCommandError(*stream_, F("Comando desconhecido. Digite 'help'."));
}

} // namespace robos3
