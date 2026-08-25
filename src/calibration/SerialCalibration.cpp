#ifdef ROBOS3_CALIBRATION_SERIAL
#include "SerialCalibration.h"
#include "CalibrationManager.h"
#include "../RoboS3.h"
#include <string.h>
#include <stdlib.h>

namespace robos3
{

SerialCalibration::SerialCalibration(CalibrationManager& m)
    : manager_(m), stream_(nullptr), buffer_{}, length_(0)
{
}
void SerialCalibration::begin(Stream& s)
{
    stream_ = &s;
    stream_->println(F("=====================================\n          RoboS3 "
                       "Calibration\n=====================================\nDigite: help"));
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
    stream_->println(
        F("SISTEMA: help | status | save | load | reset\n"
          "LINHA: line read|raw|calibrated | line calibrate <ms> | line threshold <0-1000> | line "
          "save\n"
          "MOTORES: motor status|stop|brake | motor move <E> <D>\n"
          "         motor min <left|right> <forward|reverse> <0-100>\n"
          "         motor compensation <left|right> <forward|reverse> <fator>\n"
          "CORES: color read|rgb|hsv <left|right> | color calibrate <lado> <cor> [amostras]\n"
          "       color hue <lado> <cor> <hmin> <hmax>\n"
          "       color hsv-range <lado> <cor> <hmin> <hmax> <smin> <smax> <vmin> <vmax>\n"
          "       color gain <lado> <1|4|16|60> | color integration <lado> <2|24|50|101|154|600>\n"
          "IMU: imu read|status|zero | imu calibrate <ms>"));
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
        stream_->println(ok ? F("OK") : F("ERRO"));
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
            }
            else
            {
                robo.lerLinhaCalibrada(v);
            }
            for (auto x : v)
            {
                stream_->print(x);
                stream_->print(' ');
            }
            stream_->println();
        }
        else if (!strcmp(sub, "calibrate"))
        {
            char* v = strtok(nullptr, " ");
            manager_.calibrateLine(v ? strtoul(v, nullptr, 10) : 5000);
        }
        else if (!strcmp(sub, "threshold"))
        {
            char* v = strtok(nullptr, " ");
            if (v)
            {
                robo.configurarLimiarLinha(atoi(v));
            }
        }
        else if (!strcmp(sub, "save"))
        {
            stream_->println(manager_.save() ? F("OK") : F("ERRO"));
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
            }
        }
        else if (!strcmp(sub, "stop"))
        {
            manager_.stopMotors();
        }
        else if (!strcmp(sub, "brake"))
        {
            manager_.stopMotors(true);
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
                }
                else
                {
                    robo.configurarCompensacaoMotor(parseMotor(m), parseDirection(d), atof(v));
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
            if (!strcmp(sub, "hsv"))
            {
                stream_->print(r.hsv.h);
                stream_->print(' ');
                stream_->print(r.hsv.s);
                stream_->print(' ');
                stream_->println(r.hsv.v);
            }
            else
            {
                stream_->print(r.rgb.r);
                stream_->print(' ');
                stream_->print(r.rgb.g);
                stream_->print(' ');
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
            }
            return;
        }
        if (!strcmp(sub, "integration"))
        {
            char* v = strtok(nullptr, " ");
            if (v)
            {
                robo.setTempoIntegracaoCor(s, parseIntegration(atoi(v)));
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
            robo.calibrarCor(s, c, n ? atoi(n) : 30);
        }
        else if (!strcmp(sub, "hue"))
        {
            char *a = strtok(nullptr, " "), *b = strtok(nullptr, " ");
            if (a && b)
            {
                robo.definirFaixaH(s, c, atof(a), atof(b));
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
            stream_->println(robo.imuDisponivel() ? F("OK") : F("ERRO"));
        }
        else if (!strcmp(sub, "zero"))
        {
            robo.zerarAnguloZ();
        }
        else if (!strcmp(sub, "calibrate"))
        {
            char* v = strtok(nullptr, " ");
            manager_.calibrateImu(v ? strtoul(v, nullptr, 10) : 3000);
        }
        return;
    }
    stream_->println(F("Comando desconhecido. Digite help."));
}

} // namespace robos3
#endif
