#include "WebCalibration.h"
#include "CalibrationManager.h"
#include "../RoboS3.h"
#include <WiFi.h>
#include <esp_system.h>

namespace robos3
{

static const char PAGE[] PROGMEM =
    R"HTML(<!doctype html><html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>RoboS3</title><style>body{font:16px system-ui;margin:auto;max-width:900px;padding:16px;background:#101827;color:#eef}section{background:#1d2939;padding:14px;margin:12px 0;border-radius:12px}button,input,select{font:inherit;padding:9px;margin:4px}pre{white-space:pre-wrap}.bar{display:inline-block;width:8%;margin:2px;background:#42b883;vertical-align:bottom}</style></head><body><h1>RoboS3 Calibracao</h1><section><h2>Status</h2><pre id="status"></pre></section><section><h2>Linha</h2><div id="bars"></div><pre id="line"></pre><input id="th" type="number" value="500" min="0" max="1000"><button onclick="post('/api/line/threshold?value='+th.value)">Aplicar threshold</button><button onclick="post('/api/line/calibrate?ms=5000')">Calibrar linha</button></section><section><h2>Motores</h2><input id="l" type="number" value="30" min="-100" max="100"><input id="r" type="number" value="30" min="-100" max="100"><button onpointerdown="motor()" onpointerup="stop()" onpointerleave="stop()">Segure para testar</button><button onclick="stop()">Frear</button><br><select id="mot"><option value="0">Esquerdo</option><option value="1">Direito</option></select><select id="dir"><option value="0">Frente</option><option value="1">Tras</option></select><input id="min" type="number" value="18" min="0" max="100"><input id="comp" type="number" value="1" step=".01"><button onclick="motorCfg()">Aplicar calibracao</button></section><section><h2>Cores</h2><pre id="colors"></pre><select id="side"><option value="0">Esquerdo</option><option value="1">Direito</option></select><select id="cor"><option value="4">Verde</option><option value="3">Vermelho</option><option value="5">Azul</option><option value="6">Amarelo</option></select><button onclick="post('/api/colors/calibrate?side='+side.value+'&color='+cor.value+'&samples=30')">Calibrar cor</button></section><section><h2>IMU</h2><pre id="imu"></pre><button onclick="post('/api/imu/zero')">Zerar Z</button><button onclick="post('/api/imu/calibrate?ms=3000')">Calibrar IMU</button></section><section><h2>Sistema</h2><button onclick="post('/api/save')">Salvar</button><button onclick="post('/api/load')">Recarregar</button><button onclick="if(confirm('Apagar calibracao?'))post('/api/reset')">Restaurar padrao</button><button onclick="if(confirm('Reiniciar robo?'))post('/api/restart')">Reiniciar</button></section><script>async function get(u){return(await fetch(u)).json()}async function post(u){let r=await fetch(u,{method:'POST'});if(!r.ok)alert('Falha: '+await r.text())}async function tick(){try{let s=await get('/api/status');status.textContent=JSON.stringify(s,null,2);let x=await get('/api/line');line.textContent=x.pattern+'  pos='+x.position+'  sem linha='+x.lost_ms+'ms';bars.innerHTML=x.values.map(v=>'<i class="bar" style="height:'+(10+v/4)+'px"></i>').join('');colors.textContent=JSON.stringify(await get('/api/colors'),null,2);imu.textContent=JSON.stringify(await get('/api/imu'),null,2)}catch(e){}setTimeout(tick,350)}function motor(){post('/api/motors?left='+l.value+'&right='+r.value)}function stop(){post('/api/motors/stop')}function motorCfg(){post('/api/motors/config?motor='+mot.value+'&direction='+dir.value+'&minimum='+min.value+'&compensation='+comp.value)}tick()</script></body></html>)HTML";

WebCalibration::WebCalibration(CalibrationManager& m) : manager_(m), server_(80), ssid_{}
{
}
bool WebCalibration::begin()
{
    uint64_t id = ESP.getEfuseMac();
    snprintf(ssid_, sizeof(ssid_), "RoboS3-Calibracao-%04X", static_cast<unsigned>(id & 0xFFFF));
    if (!WiFi.softAP(ssid_))
    {
        return false;
    }
    routes();
    server_.begin();
    return true;
}
void WebCalibration::routes()
{
    server_.on("/", HTTP_GET, [this] { root(); });
    server_.on("/api/status", HTTP_GET, [this] { apiStatus(); });
    server_.on("/api/line", HTTP_GET, [this] { apiLine(); });
    server_.on("/api/colors", HTTP_GET, [this] { apiColors(); });
    server_.on("/api/imu", HTTP_GET, [this] { apiImu(); });
    server_.on("/api/motors", HTTP_POST, [this] { apiMotors(); });
    server_.on("/api/motors/stop", HTTP_POST, [this] { apiStop(); });
    server_.on("/api/motors/config", HTTP_POST, [this] { apiMotorConfig(); });
    server_.on("/api/colors/config", HTTP_POST, [this] { apiColorConfig(); });
    server_.on("/api/colors/range", HTTP_POST, [this] { apiColorRange(); });
    server_.on("/api/colors/calibrate", HTTP_POST,
               [this]
               {
                   SensorCor s = static_cast<SensorCor>(server_.arg("side").toInt());
                   CorDetectada c = static_cast<CorDetectada>(server_.arg("color").toInt());
                   manager_.robo().calibrarCor(s, c, server_.arg("samples").toInt());
                   server_.send(204);
               });
    server_.on("/api/save", HTTP_POST, [this] { apiSave(); });
    server_.on("/api/load", HTTP_POST, [this] { apiLoad(); });
    server_.on("/api/reset", HTTP_POST, [this] { apiReset(); });
    server_.on("/api/imu/zero", HTTP_POST,
               [this]
               {
                   manager_.robo().zerarAnguloZ();
                   server_.send(204);
               });
    server_.on("/api/imu/calibrate", HTTP_POST,
               [this]
               {
                   manager_.calibrateImu(server_.arg("ms").toInt());
                   server_.send(204);
               });
    server_.on("/api/line/calibrate", HTTP_POST,
               [this]
               {
                   const bool calibrated = manager_.calibrateLine(server_.arg("ms").toInt());
                   if (calibrated)
                   {
                       server_.send(204);
                   }
                   else
                   {
                       server_.send(422, "application/json",
                                    "{\"ok\":false,\"error\":\"qtr_calibration_invalid\"}");
                   }
               });
    server_.on("/api/line/threshold", HTTP_POST,
               [this]
               {
                   manager_.robo().configurarLimiarLinha(server_.arg("value").toInt());
                   server_.send(204);
               });
    server_.on("/api/restart", HTTP_POST,
               [this]
               {
                   manager_.stopMotors(true);
                   server_.send(200, "application/json", "{\"ok\":true}");
                   delay(100);
                   ESP.restart();
               });
}
void WebCalibration::update()
{
    server_.handleClient();
}
void WebCalibration::root()
{
    server_.send_P(200, "text/html; charset=utf-8", PAGE);
}
void WebCalibration::apiStatus()
{
    auto& s = manager_.robo().status();
    IPAddress ip = WiFi.softAPIP();
    char b[420];
    snprintf(b, sizeof(b),
             "{\"version\":\"0.4.0\",\"ssid\":\"%s\",\"ip\":\"%u.%u.%u.%u\",\"qtr\":%s,"
             "\"ultrasonic\":%s,\"ultrasonic_2\":%s,\"vl53_1\":%s,\"vl53_2\":%s,"
             "\"color_left\":%s,\"color_right\":%s,\"imu\":%s,\"calibration\":%s,"
             "\"heap\":%u,\"uptime_ms\":%u}",
             ssid_, ip[0], ip[1], ip[2], ip[3], s.linha ? "true" : "false",
             s.ultrassom ? "true" : "false", s.ultrassom2 ? "true" : "false",
             s.vl53_1 ? "true" : "false", s.vl53_2 ? "true" : "false",
             s.corEsquerda ? "true" : "false",
             s.corDireita ? "true" : "false", s.imu ? "true" : "false",
             s.calibracaoCarregada ? "true" : "false", ESP.getFreeHeap(), millis());
    server_.send(200, "application/json", b);
}
void WebCalibration::apiLine()
{
    auto r = manager_.robo().lerLinha();
    char b[384];
    int n = snprintf(b, sizeof(b),
                     "{\"pattern\":\"%s\",\"mask\":%u,\"position\":%u,\"lost_ms\":%u,\"values\":[",
                     r.padrao, r.mascara, r.posicao, r.tempoSemLinhaMs);
    for (uint8_t i = 0; i < 8; ++i)
    {
        n += snprintf(b + n, sizeof(b) - n, "%s%u", i ? "," : "", r.calibrado[i]);
    }
    snprintf(b + n, sizeof(b) - n, "]}");
    server_.send(200, "application/json", b);
}
void WebCalibration::apiColors()
{
    auto l = manager_.robo().lerDadosCor(COR_ESQUERDA),
         r = manager_.robo().lerDadosCor(COR_DIREITA);
    char b[320];
    snprintf(b, sizeof(b),
             "{\"left\":{\"r\":%.1f,\"g\":%.1f,\"b\":%.1f,\"h\":%.1f,\"s\":%.1f,\"v\":%.1f,"
             "\"color\":%u},\"right\":{\"r\":%.1f,\"g\":%.1f,\"b\":%.1f,\"h\":%.1f,\"s\":%.1f,"
             "\"v\":%.1f,\"color\":%u}}",
             l.rgb.r, l.rgb.g, l.rgb.b, l.hsv.h, l.hsv.s, l.hsv.v, l.cor, r.rgb.r, r.rgb.g, r.rgb.b,
             r.hsv.h, r.hsv.s, r.hsv.v, r.cor);
    server_.send(200, "application/json", b);
}
void WebCalibration::apiImu()
{
    auto r = manager_.robo().lerIMU();
    char b[256];
    snprintf(b, sizeof(b),
             "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"roll\":%."
             "2f,\"pitch\":%.2f,\"yaw\":%.2f}",
             r.accX, r.accY, r.accZ, r.gyroX, r.gyroY, r.gyroZ, r.roll, r.pitch, r.yawRelativo);
    server_.send(200, "application/json", b);
}
void WebCalibration::apiMotors()
{
    manager_.testMotors(server_.arg("left").toInt(), server_.arg("right").toInt());
    server_.send(204);
}
void WebCalibration::apiStop()
{
    manager_.stopMotors(true);
    server_.send(204);
}
void WebCalibration::apiMotorConfig()
{
    Motor m = server_.arg("motor").toInt() ? MOTOR_DIREITO : MOTOR_ESQUERDO;
    SentidoMotor d = server_.arg("direction").toInt() ? TRAS : FRENTE;
    if (server_.hasArg("minimum"))
    {
        manager_.robo().configurarMinimoMovimento(m, d, server_.arg("minimum").toInt());
    }
    if (server_.hasArg("compensation"))
    {
        manager_.robo().configurarCompensacaoMotor(m, d, server_.arg("compensation").toFloat());
    }
    if (server_.hasArg("inverted"))
    {
        bool inv = server_.arg("inverted").toInt();
        if (m == MOTOR_ESQUERDO)
        {
            manager_.robo().inverterMotorEsquerdo(inv);
        }
        else
        {
            manager_.robo().inverterMotorDireito(inv);
        }
    }
    server_.send(204);
}
void WebCalibration::apiColorConfig()
{
    SensorCor s = server_.arg("side").toInt() ? COR_DIREITA : COR_ESQUERDA;
    if (server_.hasArg("gain"))
    {
        manager_.robo().setGanhoCor(
            s, static_cast<GanhoCor>(constrain(server_.arg("gain").toInt(), 0, 3)));
    }
    if (server_.hasArg("integration"))
    {
        manager_.robo().setTempoIntegracaoCor(s, static_cast<TempoIntegracaoCor>(constrain(
                                                     server_.arg("integration").toInt(), 0, 5)));
    }
    server_.send(204);
}
void WebCalibration::apiColorRange()
{
    SensorCor s = server_.arg("side").toInt() ? COR_DIREITA : COR_ESQUERDA;
    CorDetectada c =
        static_cast<CorDetectada>(constrain(server_.arg("color").toInt(), 1, (int)COR_TOTAL - 1));
    manager_.robo().definirFaixaHSV(s, c, server_.arg("hmin").toFloat(),
                                    server_.arg("hmax").toFloat(), server_.arg("smin").toFloat(),
                                    server_.arg("smax").toFloat(), server_.arg("vmin").toFloat(),
                                    server_.arg("vmax").toFloat());
    server_.send(204);
}
void WebCalibration::apiSave()
{
    bool ok = manager_.save();
    server_.send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}
void WebCalibration::apiLoad()
{
    bool ok = manager_.load();
    server_.send(ok ? 200 : 404, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}
void WebCalibration::apiReset()
{
    bool ok = manager_.reset();
    server_.send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

} // namespace robos3
