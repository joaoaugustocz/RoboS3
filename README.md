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
| `examples_all` | Verificação de desenvolvimento/CI | Os 28 exemplos, isolados internamente | Detecta quebra de API sem gerar firmware para upload. |
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
    Serial.begin(115200);
    robo.begin();
}

void loop()
{
    robo.frente(50);
}
```

`begin()` deixa os motores parados e tenta inicializar cada módulo independentemente. Ele retorna `true` somente quando todos os módulos previstos na PCB respondem. Quando algo falha, o próprio `begin()` imprime na `Serial` qual módulo não respondeu e uma indicação do que verificar. Inicie a `Serial` antes dele para visualizar essas mensagens.

Uma falha não impede a configuração dos módulos restantes. Quando o programa precisa decidir se pode continuar parcialmente, consulte `status()`, `linhaDisponivel()`, `corDisponivel()`, `ultrassomDisponivel()` e `imuDisponivel()`. `printErrosInicializacao()` continua disponível para reimprimir o diagnóstico sob demanda ou em outro objeto `Print`.

Ausência de calibração salva não é erro de hardware. Nesse caso, `status().calibracaoCarregada` será `false`, `printStatus()` mostrará `[PADRAO] Calibracao` e a biblioteca usará seus valores iniciais.

O exemplo `26_TratamentoErros` mostra o fluxo completo: imprime o diagnóstico, desativa recursos dependentes do módulo ausente e interrompe somente quando motores ou QTR não estão disponíveis.

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

### Qual leitura de linha usar

| Chamada | Faz nova aquisição? | Retorno | Quando usar |
|---|---:|---|---|
| `lerLinha()` | Sim | `LeituraLinha` completa | Quando precisa de dois ou mais dados: bruto, calibrado, posição, máscara, nome P/B ou tempo sem linha. É a leitura principal. |
| `lerLinhaBruta(saida)` | Sim | Preenche `uint16_t saida[8]` | Diagnóstico elétrico, conferência da ordem S1–S8 e contraste sem calibração. |
| `lerLinhaCalibrada(saida)` | Sim | Preenche `uint16_t saida[8]` em `0…1000` | Observar o resultado da calibração sensor por sensor. |
| `lerLinhaBinaria()` | Sim | `uint8_t` | Quando deseja somente bits, por exemplo `0b00011000`. |
| `lerPadraoLinha()` | Sim | `PadraoLinha` | Quando deseja escrever diretamente `BBBPPBBB` em um `if` ou `switch`. |
| `ultimaLeituraLinha()` | Não | Referência para a última `LeituraLinha` | Reutilizar a aquisição anterior sem ler fisicamente o QTR outra vez. |

`LeituraLinha` contém:

- `bruto[8]`: aquisição física S1…S8;
- `calibrado[8]`: valores normalizados em `0…1000`;
- `mascara`: bits em que preto físico vale `1`;
- `caso`: o mesmo byte como `PadraoLinha`, por exemplo `BBBPPBBB`;
- `padrao`: texto de nove posições contendo `P/B` e terminador zero;
- `quantidadePretos`, `posicao`, `detectada`, `posicaoValida`, `timestampMs`, `tempoSemLinhaMs` e `ultimaPosicaoValida`.

### Padrões P/B e binários

`PadraoLinha` possui os 256 casos possíveis. `P` significa preto físico/bit `1`, `B` significa branco físico/bit `0`, e os caracteres seguem S1 até S8:

```cpp
LeituraLinha linha = robo.lerLinha();

if (linha.caso == BBBPPBBB)       // igual a 0b00011000
{
    robo.frente(60);
}

switch (linha.caso)
{
    case BBBBBBBP:                // igual a 0b00000001
        robo.girarDireita(40);
        break;

    case PBBBBBBB:                // igual a 0b10000000
        robo.girarEsquerda(40);
        break;

    default:
        break;
}
```

Os nomes são constantes de compilação e não criam uma tabela com 256 elementos na RAM. Para converter qualquer caso em texto, use `RoboS3::padraoLinhaTexto(caso, texto)` com um `char texto[9]`.

### Histórico dos últimos casos

O histórico registra **mudanças de padrão**, não cada amostra repetida. Ele usa um buffer circular: quando enche, o caso mais antigo é substituído. O tamanho inicial é 8 e pode ser configurado entre 0 e 32; zero desativa o registro.

```cpp
robo.configurarHistoricoLinha(10); // limpa e guarda até 10 mudanças

// Cada lerLinha() atualiza o histórico.
LeituraLinha linha = robo.lerLinha();

PadraoLinha casos[10];
uint8_t quantidade = robo.copiarUltimosCasosLinha(casos, 10);

CasoLinha detalhado[10];
quantidade = robo.copiarHistoricoLinha(detalhado, 10);
```

As duas cópias vêm do caso mais novo para o mais antigo. `CasoLinha` contém `padrao`, `inicioMs` e `duracaoMs`. A duração de um caso encerrado vai da primeira leitura desse padrão até a primeira leitura do padrão seguinte. A duração do caso mais recente continua sendo atualizada até o momento da consulta.

Não há `new`, alocação dinâmica nem cópia automática de listas durante a leitura. A atualização é O(1) e reserva 384 bytes para o máximo de 32 casos no ESP32-S3, independentemente do tamanho configurado. A medição do firmware principal passou de 25.044 para 25.428 bytes de RAM. As listas só são copiadas quando o usuário chama uma das funções `copiar...`.

Consulte `27_HistoricoLinha` para um exemplo completo das duas visualizações.

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

Há duas formas de calibrar a classificação:

- automática: `calibrarCor(sensor, cor, amostras)` mede o objeto posicionado sob o sensor;
- manual: observe HSV no Monitor Serial e informe os limites com `definirFaixaHSV()`.

```cpp
robo.definirFaixaHSV(COR_ESQUERDA, COR_VERDE,
                     75, 165,   // H mínimo e máximo
                     25, 100,   // S mínimo e máximo
                     15, 100);  // V mínimo e máximo
robo.salvarCalibracao();
```

Uma faixa vermelha pode atravessar o zero do hue, por exemplo `345…15`. O exemplo `28_CalibracaoCorManual` imprime HSV, configura verde/vermelho/azul, ajusta ganho e integração e salva os valores.

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

Defina `ROBOS3_DEBUG_LEVEL` como 0…3 via build flag. O modo normal não imprime continuamente. A única saída automática é o diagnóstico de falha do `begin()`. `printLinha`, `printCores`, `printIMU`, `printMotores`, `printStatus` e `printErrosInicializacao` podem ser chamados explicitamente.

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

## Referência completa da API

Todos os métodos abaixo pertencem a `RoboS3`. Crie uma única instância, configure o que for necessário e chame `begin()` uma vez. Métodos de pinagem devem vir **antes** de `begin()`; os demais, depois dela.

### 1. Sistema, status e depuração

| Método | Aplicação |
|---|---|
| `RoboS3()` | Cria o objeto com a pinagem padrão da PCB. |
| `begin(diagnostico)` | Inicializa todos os módulos. Retorna `true` somente se todos responderam e imprime automaticamente as falhas no `Print` recebido, `Serial` por padrão. |
| `atualizar()` | Atualiza IMU e interfaces Serial/Web habilitadas. Chame continuamente no `loop()`. |
| `status()` | Retorna `const StatusRobo&` com o estado de todos os módulos. |
| `inicializacaoCompleta()` | Retorna `true` quando todos os módulos previstos responderam ao `begin()`. |
| `operacaoBasicaDisponivel()` | Retorna `true` quando motores e QTR estão disponíveis para um seguidor simples. |
| `linhaDisponivel()` / `ultrassomDisponivel()` / `imuDisponivel()` | Informam se o periférico iniciou corretamente. |
| `corDisponivel(sensor)` | Informa se o TCS34725 de `COR_ESQUERDA` ou `COR_DIREITA` está disponível. |
| `printStatus(out)` | Imprime o estado dos módulos; o destino padrão é `Serial`. |
| `printErrosInicializacao(out)` | Imprime somente os módulos que falharam e uma indicação do ponto a verificar. |
| `setDebugLevel(nivel)` | Seleciona `DEBUG_OFF`, `DEBUG_ERROR`, `DEBUG_INFO` ou `DEBUG_VERBOSE`. |

### 2. Motores e ponte H L298N

Velocidades aceitam `-100` a `100`: positivo é frente, negativo é ré e zero para o motor.

| Método | Aplicação |
|---|---|
| `mover(esquerda, direita)` | Comanda os dois motores independentemente; base para curvas suaves. |
| `frente(velocidade)` / `tras(velocidade)` | Deslocamento retilíneo. |
| `girarEsquerda(velocidade)` / `girarDireita(velocidade)` | Giro no próprio eixo. |
| `motorEsquerdo(velocidade)` / `motorDireito(velocidade)` | Atualiza somente um motor e preserva o comando do outro. |
| `parar()` | Desliga enables: parada livre (*coast*). |
| `frear()` | Aplica frenagem ativa na ponte H. |
| `inverterMotorEsquerdo(invertido)` / `inverterMotorDireito(invertido)` | Inverte eletricamente um motor. |
| `inverterMotores(esquerdo, direito)` | Configura as duas inversões de uma vez. |
| `configurarMinimoMovimento(motor, sentido, minimo)` | Define PWM mínimo de partida `0…100` para cada motor e sentido. |
| `configurarCompensacaoMotor(motor, sentido, fator)` | Aplica multiplicador para corrigir velocidades diferentes; `1.0` não altera. |
| `printMotores(out)` | Mostra comandos e calibração atual. |

### 3. Sensor de linha QTR e pista

| Método | Aplicação |
|---|---|
| `lerLinha()` | Retorna `LeituraLinha`: dados brutos, calibrados, máscara, padrão, posição e tempo sem linha. |
| `lerLinhaBruta(saida)` | Preenche `uint16_t saida[8]` com a leitura física. |
| `lerLinhaCalibrada(saida)` | Preenche `uint16_t saida[8]` com valores `0…1000`. |
| `lerLinhaBinaria()` | Retorna a máscara de sensores pretos: S1 é bit 7 e S8 é bit 0. |
| `lerPadraoLinha()` | Retorna diretamente um `PadraoLinha`, como `BBBPPBBB`. |
| `ultimaLeituraLinha()` | Retorna a última `LeituraLinha`, sem nova aquisição. |
| `tempoSemLinha()` | Retorna milissegundos desde a última detecção válida. |
| `configurarHistoricoLinha(tamanho)` | Limpa e configura entre 0 e 32 mudanças armazenadas; zero desativa. |
| `limparHistoricoLinha()` | Remove todos os casos registrados. |
| `capacidadeHistoricoLinha()` / `quantidadeHistoricoLinha()` | Informam espaço configurado e quantidade atualmente válida. |
| `copiarUltimosCasosLinha(saida, maximo)` | Copia somente `PadraoLinha`, do mais novo para o mais antigo. |
| `copiarHistoricoLinha(saida, maximo)` | Copia `CasoLinha` com início e duração, do mais novo para o mais antigo. |
| `padraoLinhaTexto(caso, saida)` | Converte qualquer enum P/B para `char saida[9]`. |
| `calibrarLinha(ms)` | Para os motores e coleta extremos por tempo; padrão 5000 ms. É bloqueante. |
| `calibrarLinhaAmostras(amostras)` | Variante bloqueante por quantidade de amostras. |
| `configurarLimiarLinha(limiar)` | Define o limiar binário `0…1000`; inicial 500. |
| `erroLinha(centro)` | Retorna `posição - centro`; centro padrão 3500. |
| `printLinha(out)` | Imprime valores, padrão, máscara, posição e tempo sem linha. |

`LeituraLinha` também fornece `semPreto()`, `todosPretos()`, `temPretoEsquerda()`, `temPretoCentro()` e `temPretoDireita()`. Os campos principais são `padrao`, `mascara`, `quantidadePretos`, `posicao`, `detectada` e `posicaoValida`.

### 4. PID para seguidor de linha

| Método | Aplicação |
|---|---|
| `configurarPID(kp, ki, kd)` | Define os ganhos proporcional, integral e derivativo. |
| `resetPID()` | Zera memória integral e derivativa, por exemplo ao reiniciar a pista. |
| `calcularPID(erro)` | Retorna a correção limitada para um erro informado. |
| `seguirLinha(velocidadeBase)` | Lê a linha, calcula PID e comanda os motores; para se não houver linha. |

### 5. Distância e obstáculo (HC-SR04)

| Método | Aplicação |
|---|---|
| `distanciaCM()` | Retorna centímetros ou `NAN` quando a leitura falha. |
| `temObstaculo(maximoCm)` | Retorna `true` somente para medida válida até o limite. |

### 6. Cores: dois TCS34725, RGB, HSV e HSL

Use `COR_ESQUERDA` ou `COR_DIREITA` para selecionar o módulo.

| Método | Aplicação |
|---|---|
| `lerDadosCor(sensor)` | Retorna `LeituraCor` completa: RGB, HSV, HSL, raw, clear, classificação e timestamp. |
| `lerRGB(sensor)` / `lerHSV(sensor)` / `lerHSL(sensor)` | Retornam somente a conversão solicitada. RGB é `0…255`; HSV/HSL usam hue `0…360`. |
| `lerCor(sensor)` | Retorna `CorDetectada`, como `COR_VERDE` ou `COR_DESCONHECIDA`. |
| `definirFaixaH(sensor, cor, hMin, hMax)` | Define hue; pode cruzar 360°, por exemplo `345, 15`. |
| `definirFaixaHSV(sensor, cor, hMin, hMax, sMin, sMax, vMin, vMax)` | Define a faixa completa de classificação. |
| `calibrarCor(sensor, cor, amostras)` | Coleta faixa HSV para a cor sob o sensor; padrão 30 amostras. |
| `setGanhoCor(sensor, ganho)` | Seleciona `GANHO_1X`, `GANHO_4X`, `GANHO_16X` ou `GANHO_60X`. |
| `setTempoIntegracaoCor(sensor, tempo)` | Seleciona integração de 2,4; 24; 50; 101; 154 ou 600 ms. |
| `printCores(out)` | Imprime os dois sensores. |

### 7. Orientação e inclinação (MPU6050)

Use `atualizar()` no `loop()` ou `atualizarIMU()` antes de consultar ângulos. O yaw/Z é relativo, não uma bússola.

| Método | Aplicação |
|---|---|
| `lerIMU()` | Atualiza e retorna `LeituraIMU`: aceleração, giroscópio, temperatura e ângulos. |
| `atualizarIMU()` | Atualiza somente a estimativa da IMU. |
| `roll()` / `pitch()` | Retornam inclinação lateral e longitudinal em graus. |
| `anguloZ()` | Retorna yaw/Z relativo desde a última zeragem. |
| `zerarAnguloZ()` | Define o yaw atual como zero; use antes de curvas controladas. |
| `estaInclinado(graus)` | Retorna `true` se `abs(roll)` ou `abs(pitch)` atingir o limite. |
| `calibrarIMU(ms)` | Para motores e calibra offsets com o robô imóvel; padrão 3000 ms. É bloqueante. |
| `printIMU(out)` | Imprime dados atuais da IMU. |

### 8. Persistência de calibração

| Método | Aplicação |
|---|---|
| `salvarCalibracao()` | Salva limites QTR, calibração dos motores, faixas de cor e offsets da IMU no NVS. |
| `carregarCalibracao()` | Carrega e aplica os valores gravados; retorna `false` se não houver dado válido. |
| `apagarCalibracao()` | Remove a calibração persistida. |

### 9. Configuração de hardware e pinagem

Chame estes métodos somente antes de `begin()`.

| Método | Aplicação |
|---|---|
| `definePinsQTR(s1, …, s8, emitter)` | Define os oito GPIOs QTR e emitter opcional; use `0xFF` quando não existir emitter. |
| `setTipoQTR(tipo)` | Seleciona `QTR_8A` ou `QTR_8RC`. |
| `setTipoLinha(tipo)` | Seleciona `LINHA_PRETA` ou `LINHA_BRANCA`. |
| `definePinsPonteH(ena, in1, in2, in3, in4, enb)` | Define toda a pinagem L298N. |
| `definePinsUltrassom(trigger, echo)` | Define os pinos do HC-SR04. |
| `definePinsI2C0(sda, scl, frequencia)` / `definePinsI2C1(sda, scl, frequencia)` | Define cada barramento I2C; padrão 400000 Hz. |
| `configurarIMU(endereco)` | Define endereço I2C MPU6050, normalmente `0x68`. |

### 10. Tipos retornados

| Tipo | Campos ou valores principais |
|---|---|
| `Motor` / `SentidoMotor` | `MOTOR_ESQUERDO`, `MOTOR_DIREITO`; `FRENTE`, `TRAS`. |
| `TipoQTR` / `TipoLinha` | `QTR_8A`, `QTR_8RC`; `LINHA_PRETA`, `LINHA_BRANCA`. |
| `SensorCor` / `CorDetectada` | Lado esquerdo/direito; preta, branca, vermelha, verde, azul, amarela, laranja, roxa ou desconhecida. |
| `RGB`, `HSV`, `HSL` | Conversões retornadas pelas leituras de cor. |
| `PadraoLinha` | Todos os 256 nomes P/B; `BBBPPBBB` equivale a `0b00011000`. |
| `LeituraLinha` | `bruto`, `calibrado`, `mascara`, `caso`, texto `padrao`, contagem, posição, validade e tempos. |
| `CasoLinha` | `padrao`, `inicioMs` e `duracaoMs` de uma mudança registrada. |
| `LeituraCor` | `rgb`, `hsv`, `hsl`, raw, `clear`, `cor`, `valida`, `timestampMs`. |
| `LeituraIMU` | `accX/Y/Z`, `gyroX/Y/Z`, `roll`, `pitch`, `yawRelativo`, `temperatura`, `valida`, `timestampMs`. |
| `StatusRobo` | Flags de disponibilidade dos módulos e da calibração salva; também oferece `inicializacaoCompleta()` e `operacaoBasicaDisponivel()`. |

Os 28 exemplos em `examples/` seguem essa divisão: básicos e motores (01–04), ultrassom (05), linha/PID (06–13), cores (14–18), IMU (19–22), calibração Web/Serial (23–24), pinagem alternativa (25), diagnóstico de inicialização (26), histórico de linha (27) e calibração manual de cor (28).

## Limites de validação

Compilação não valida polaridade física dos motores, ordem esquerda/direita, divisor de tensão do ECHO, orientação mecânica do MPU, posição física S1/S8, presença dos sensores nem qualidade das faixas de cor. Esses pontos exigem placa conectada; nenhuma porta serial foi detectada durante o desenvolvimento.
