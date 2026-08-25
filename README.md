# RoboS3

Biblioteca Arduino/PlatformIO em português para a PCB educacional **ESP_FB V1**, baseada no **ESP32-S3-N8R2**. A API começa simples (`frente`, `parar`, `seguirLinha`) e preserva acesso a valores brutos, calibração, máscara, PID, RGB/HSV/HSL e IMU para o aluno evoluir sem trocar de biblioteca.

> Estado do hardware: o repositório recebido estava vazio e não continha esquemático/PCB. A pinagem abaixo foi reconstruída das capturas de layout fornecidas. Ela precisa de teste elétrico na placa antes de movimentar um robô apoiado no chão.

## Hardware e pinagem

| Função | GPIO/rede | Evidência |
|---|---:|---|
| QTR S1…S8 | 4, 5, 6, 7, 15, 16, 17, 18 | trilhas S1…S8 até o header do ESP32 |
| Tipo QTR | QTR-8A (analógico) | oito GPIO ADC; não há temporização RC/emitter visível |
| Emitter QTR | não conectado | nenhuma rede CTRL/LEDON nas imagens |
| ULTRA1 TRIG/ECHO | 9 / 46 | redes TRIG1/ECHO1 |
| I2C0 SDA/SCL | 1 / 42 | primeiro par de redes SDA/SCL |
| I2C1 SDA/SCL | 41 / 40 | segundo par de redes SDA1/SCL1 |
| TCS esquerdo | I2C0, endereço 0x29 | sensores iguais em buses distintos |
| TCS direito | I2C1, endereço 0x29 | sensores iguais em buses distintos |
| MPU6050 | I2C0, endereço 0x68 | compartilha barramento sem conflito de endereço |
| INT MPU6050 | não conectado/comprovado | não aparece nas capturas |
| L298N ENA/IN1/IN2 | 0 / 36 / 39 | redes do conector PONTE-H |
| L298N ENB/IN3/IN4 | 35 / 38 / 37 | redes do conector PONTE-H |

Não foram comprovados LED, botão ou buzzer controláveis. `BoardPins.h` usa `-1` para esses periféricos. GPIO0 é pino de strapping; mantenha a ponte H sem forçar nível indevido durante boot. O ECHO do HC-SR04 precisa chegar ao ESP32 em **3,3 V**; confirme divisor/level shifter na placa.

## Instalação

Em um projeto PlatformIO:

```ini
[env:robos3]
platform = https://github.com/pioarduino/platform-espressif32.git#55.03.37
board = esp32-s3-devkitc1-n8r2
framework = arduino
lib_deps =
    pololu/QTRSensors@4.0.0
    martinsos/HCSR04@2.0.0
    adafruit/Adafruit TCS34725@1.4.4
    adafruit/Adafruit BusIO@1.17.4
    electroniccats/MPU6050@1.4.4
    https://github.com/bytevoltateam/PIDEasy.git#PIDEasy-v1.0.2
```

O ambiente validado usa Arduino-ESP32 3.3.7 e a API LEDC por pino. O `platformio.ini` deste repositório contém workarounds de include para dependências internas que o LDF dessa combinação não descobre corretamente.

## Ambientes PlatformIO

Os ambientes separam firmware de uso, interfaces opcionais e verificações de integração. O firmware comum não incorpora Wi-Fi nem calibração; cada exemplo é verificado com as mesmas dependências do robô.

| Ambiente | Quando usar | O que compila | Por que existe |
|---|---|---|---|
| `robos3` | Uso normal e upload do robô | Núcleo e periféricos | É o padrão, menor e sem interface de calibração. |
| `robos3_serial` | Calibrar pela USB/Serial | Núcleo + console Serial | Mantém a ferramenta de bancada fora do firmware final. |
| `robos3_web` | Calibrar por Wi-Fi | Núcleo + SoftAP/WebServer | Inclui o dashboard Web e consome mais memória. |
| `robos3_calibration` | Usar ou validar Web e Serial juntas | Núcleo + ambas as interfaces | Permite bancada completa e cobre a integração. |
| `examples` | Base de compilação de um exemplo isolado | Biblioteca sem checagem agregada | Evita misturar sketch de exemplo ao firmware interno. |
| `example_serial` | Compilar `24_CalibracaoSerial` isoladamente | Exemplo + Serial | Propaga a macro aos `.cpp` da biblioteca. |
| `example_web` | Compilar `23_CalibracaoWeb` isoladamente | Exemplo + Web | Propaga a macro aos `.cpp` da biblioteca. |
| `examples_all` | Verificação de desenvolvimento/CI | Os 25 exemplos, isolados internamente | Detecta quebra de API sem gerar firmware para upload. |
| `unit` | Testar PWM, máscara e HSV | Unity + `RoboMath` | Separa cálculos determinísticos do hardware. |
| `native` | Executar matemática em PC com toolchain nativo | Apenas matemática pura | É opcional; não é necessário no ESP32-S3. |

Comandos mais usados:

```powershell
# Firmware normal para o robô.
pio run -e robos3

# Firmware de calibração pela Web ou pela Serial.
pio run -e robos3_web
pio run -e robos3_serial

# Validação completa antes de alterar a API pública.
pio run -e robos3 -e robos3_calibration -e examples_all
```

## Primeiro programa

```cpp
#include <RoboS3.h>
RoboS3 robo;

void setup()
{
    robo.begin();
}

void loop()
{
    robo.frente(50);
}
```

`begin()` deixa os motores parados, inicializa cada módulo independentemente e continua quando um periférico opcional não responde. Consulte `linhaDisponivel()`, `corDisponivel()`, `imuDisponivel()` e `printStatus()`.

## Motores e L298N

`mover(esquerda, direita)` recebe valores de -100 a +100. Positivo é frente; negativo é ré. `parar()` desliga enable e deixa INs baixos (coast). `frear()` coloca os dois INs de cada canal altos e enable em 100% (frenagem ativa).

Pipeline: comando → sentido → módulo → PWM mínimo → compensação → clamp → LEDC → L298N.

```cpp
robo.configurarMinimoMovimento(MOTOR_ESQUERDO, FRENTE, 18);
robo.configurarCompensacaoMotor(MOTOR_DIREITO, FRENTE, 0.96f);
robo.inverterMotores(false, true);
```

As quatro combinações esquerda/direita e frente/ré são independentes. Zero sempre produz duty zero.

## Sensor de linha

O padrão usa QTR-8A e chama `analogReadResolution(10)` porque QTRSensors 4.0.0 normaliza o backend analógico em 0…1023. Após calibração, os valores são 0…1000.

```cpp
robo.calibrarLinha(5000);       // tempo em ms
robo.configurarLimiarLinha(500);
LeituraLinha linha = robo.lerLinha();
```

`lerLinha()` faz uma aquisição física bruta e deriva dela valores calibrados, `BBBPPBBB`, máscara, contagem e posição. S1 é o bit 7; S8 é o bit 0. A posição vai de 0 a 7000, com centro 3500. Há suporte a `LINHA_PRETA` e `LINHA_BRANCA`, QTR-8A e QTR-8RC, pinagem customizada e getter const da última leitura.

`tempoSemLinha()` volta a zero quando a linha reaparece. Calibração por tempo é bloqueante e intencional; leituras normais não introduzem `delay`.

## PID

O backend é PIDEasy 1.0.2, fixado pela tag oficial. Ele permanece acessível por primitivas didáticas:

```cpp
LeituraLinha linha = robo.lerLinha();
int erro = (int)linha.posicao - 3500;
float correcao = robo.calcularPID(erro);
robo.mover(60 + correcao, 60 - correcao);
```

Use `configurarPID(kp, ki, kd)`, `resetPID()` ou a conveniência `seguirLinha(base)`. O sinal final deve ser confirmado com a montagem física dos motores/sensores.

## HC-SR04

`distanciaCM()` retorna `NAN` em leitura inválida. `temObstaculo(cm)` só retorna true para uma medida válida dentro do limite. A biblioteca Martin Sosic é bloqueante pelo tempo de eco; evite chamar em alta frequência.

## Cores: RGB, HSV e HSL

Dois TCS34725 de endereço 0x29 funcionam simultaneamente por `Wire` e `Wire1`. Uma aquisição gera raw, RGB 0…255, HSV/HSL, canal clear, classificação e timestamp.

```cpp
LeituraCor e = robo.lerDadosCor(COR_ESQUERDA);
robo.definirFaixaHSV(COR_ESQUERDA, COR_VERDE, 80,150, 30,100, 20,100);
robo.setGanhoCor(COR_DIREITA, GANHO_16X);
```

Faixas de Hue podem atravessar 360° (`345…15`). Preto/branco usam saturação e valor. Perfis padrão são apenas ponto de partida; calibre sob a iluminação real. Tempos disponíveis: 2,4; 24; 50; 101; 154 e 600 ms.

## MPU6050

`LeituraIMU` expõe aceleração em m/s², giroscópio em graus/s, temperatura e ângulos em graus. Pitch/roll usam filtro complementar; Z é integração relativa do giroscópio desde `zerarAnguloZ()`.

Não há magnetômetro: `anguloZ()` **não é bússola** e deriva com o tempo. Como nenhuma linha INT foi comprovada, o DMP/FIFO não é usado. `calibrarIMU(ms)` exige o robô imóvel, é bloqueante e calcula offsets persistíveis.

## Persistência

`salvarCalibracao()`, `carregarCalibracao()` e `apagarCalibracao()` usam Preferences/NVS no namespace `robos3`. O blob contém magic, versão de schema, tamanho e checksum. A flash só é escrita por comando explícito.

## Calibração Serial e Web

Ative por `build_flags`, pois macros definidas apenas dentro do sketch não são propagadas aos `.cpp` de uma biblioteca Arduino:

```ini
build_flags = -DROBOS3_CALIBRATION_SERIAL
; ou -DROBOS3_CALIBRATION_WEB
```

Ambos podem ser usados juntos e compartilham `CalibrationManager`. No `loop`, chame `robo.atualizar()`.

Serial opera a 115200 e oferece `help`, `status`, linha, motores, cores, IMU, `save`, `load` e `reset`, com parser de buffer fixo. Web cria `RoboS3-Calibracao-XXXX`, serve dashboard responsivo e APIs `/api/status`, `/api/line`, `/api/motors`, `/api/colors` e `/api/imu`. Testes de motor exigem ação explícita e param/freiam automaticamente em até 2 s se não forem renovados.

## Debug

Defina `ROBOS3_DEBUG_LEVEL` como 0…3 via build flag. O modo normal não imprime continuamente. `printLinha`, `printCores`, `printIMU`, `printMotores` e `printStatus` podem ser chamados explicitamente.

## Override de pinos

Use antes de `begin()`:

```cpp
robo.definePinsQTR(4,5,6,7,15,16,17,18);
robo.definePinsPonteH(0,36,39,38,37,35);
robo.definePinsUltrassom(9,46);
robo.definePinsI2C0(1,42);
robo.definePinsI2C1(41,40);
```

## Estratégias de competição

A biblioteca não afirma “encruzilhada”, “gap”, “T” ou “curva de 90°”. Uma máscara vazia pode significar vários eventos; muitos sensores pretos também. Ela entrega máscara, padrão, posição, quantidade, tempo sem linha e ângulos para o aluno confirmar eventos com estado e histórico.

Consulte `13_LogicaPista`, `21_MPU6050_Curva90` e `22_MPU6050_Rampa`.

## Troubleshooting

- Motores invertidos: use `inverterMotores` antes de alterar a lógica do aluno.
- Motor não parte: aumente o PWM mínimo daquela direção.
- QTR sempre 0/1000: refaça calibração movendo todos os sensores sobre branco e preto.
- TCS ausente: confira se cada módulo está no barramento físico correto.
- IMU deriva em Z: calibre imóvel, zere antes de cada curva e mantenha giros curtos.
- Upload falha: GPIO0/strapping pode estar sendo afetado pela ponte H; desligue potência dos motores durante diagnóstico.

## API pública resumida

- Sistema: `begin`, `atualizar`, `printStatus`, disponibilidade e persistência.
- Motores: `mover`, `frente`, `tras`, giros, motores individuais, `parar`, `frear`, inversão, mínimo e compensação.
- Linha: raw, calibrado, leitura rica, binário, threshold, calibração, erro, PID e tempo sem linha.
- Ultrassom: `distanciaCM`, `temObstaculo`.
- Cores: leitura completa/RGB/HSV/HSL/classificação, faixas, calibração, gain e integration time.
- IMU: leitura, atualização, roll, pitch, Z relativo, zero, inclinação e calibração.
- Hardware: overrides de QTR, ponte H, ultrassom, I2C e endereço do MPU.

Os 25 exemplos em `examples/` cobrem cada nível da API.

## Limites de validação

Compilação não valida polaridade física dos motores, ordem esquerda/direita, divisor de tensão do ECHO, orientação mecânica do MPU, posição física S1/S8, presença dos sensores nem qualidade das faixas de cor. Esses pontos exigem placa conectada; nenhuma porta serial foi detectada durante o desenvolvimento.
