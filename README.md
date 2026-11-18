# RoboS3

Biblioteca Arduino/PlatformIO em português para a PCB educacional OPENSOURCE **ESP_FB V1**, baseada no **ESP32-S3-N8R2**. A API começa simples (`frente`, `parar`, `seguirLinha`) e preserva acesso a valores brutos, calibração, máscara, PID, RGB/HSV/HSL e IMU para o aluno evoluir sem trocar de biblioteca.

**Versão estável:** 0.4.3 · **Framework:** Arduino · **Arquitetura:** ESP32 · **Licença:** MIT

## Conteúdo

- [Hardware e pinagem](#hardware-e-pinagem)
- [Instalação](#instalação)
- [Primeiro programa](#primeiro-programa)
- [Motores, linha e PID](#motores-e-l298n)
- [Sensores de distância, cores e IMU](#sensores-de-distância)
- [Botões digitais e analógicos](#botões-digitais-e-analógicos)
- [Calibração e debug](#persistência)
- [Referência completa da API](#referência-completa-da-api)
- [Exemplos](#exemplos-incluídos)
- [Limites de validação](#limites-de-validação)

> Estado do hardware: QTR, ponte H, dois barramentos I2C e os dois conectores ultrassônicos foram revisados conforme a configuração informada para a placa.

## Hardware e pinagem

| Função | GPIO/rede padrão | Estado |
|---|---:|---|
| QTR S1…S8 | 1, 2, 3, 5, 6, 7, 8, 9 | revisão informada; S1 é o bit 7 e S8 é o bit 0 |
| Tipo QTR | QTR-8A (analógico) | oito GPIO ADC; não há temporização RC/emitter visível |
| Emitter QTR | não conectado | nenhuma rede CTRL/LEDON nas imagens |
| ULTRA1 TRIG/ECHO | 17 / 18 | pinagem informada |
| ULTRA2 TRIG/ECHO | 16 / 42 | pinagem informada |
| I2C0 SDA/SCL | 47 / 48 | pinagem informada |
| I2C1 SDA/SCL | 39 / 40 | pinagem informada |
| TCS esquerdo | I2C0, endereço 0x29 | padrão configurável antes de `begin()` |
| TCS direito | I2C1, endereço 0x29 | padrão configurável antes de `begin()` |
| VL53L0X #1/#2 | I2C configurável, endereço inicial 0x29 | opcionais; desativados por padrão |
| MPU6050 | I2C0, endereço 0x68 | padrão configurável; pode compartilhar I2C0 ou I2C1 com um TCS |
| INT MPU6050 | não conectado/comprovado | não aparece nas capturas |
| L298N ENA/IN1/IN2 | 14 / 15 / 11 | revisão informada da ponte H |
| L298N ENB/IN3/IN4 | 21 / 12 / 13 | revisão informada da ponte H |

Um GPIO só pode exercer uma função elétrica por vez. Ao adicionar periféricos, não reutilize os pinos da tabela. O ECHO de cada HC-SR04 precisa chegar ao ESP32 em **3,3 V**; confirme divisor ou *level shifter* na placa.

## Instalação

### Arduino IDE

Abra **Ferramentas → Gerenciar Bibliotecas**, pesquise por `RoboS3` e instale a versão mais recente. As dependências declaradas em `library.properties` serão resolvidas pelo gerenciador. Depois selecione uma placa ESP32-S3 compatível e abra um sketch em **Arquivo → Exemplos → RoboS3**.

Para instalar uma versão baixada do GitHub, use **Sketch → Incluir Biblioteca → Adicionar Biblioteca .ZIP** e selecione o arquivo da [release desejada](https://github.com/joaoaugustocz/RoboS3/releases). Não mantenha duas cópias de RoboS3 na pasta de bibliotecas, pois a IDE pode escolher a versão errada.

### PlatformIO

Em um projeto PlatformIO:

```ini
[env:robos3]
platform = https://github.com/pioarduino/platform-espressif32.git#55.03.37
board = esp32-s3-devkitc1-n8r2
framework = arduino
lib_deps =
    pololu/QTRSensors@4.0.0
    martinsos/HCSR04@2.0.0
    pololu/VL53L0X@1.3.1
    adafruit/Adafruit TCS34725@1.4.4
    adafruit/Adafruit BusIO@1.17.4
    electroniccats/MPU6050@1.4.4
    https://github.com/bytevoltateam/PIDEasy.git#PIDEasy-v1.0.2
```

O ambiente validado usa Arduino-ESP32 3.3.7 e a API LEDC por pino. O `platformio.ini` deste repositório contém workarounds de include para dependências internas que o LDF dessa combinação não descobre corretamente.

## Ambientes PlatformIO

Os ambientes separam upload, atalhos de calibração e verificações de integração. Eles existem principalmente para desenvolvimento da própria biblioteca; em um projeto do aluno basta normalmente um único ambiente. Desde a versão 0.4.0, Serial e Web também podem ser iniciadas por métodos no sketch, sem flags.

| Ambiente | Quando usar | O que compila | Por que existe |
|---|---|---|---|
| `robos3` | Uso normal e upload do robô | Núcleo e periféricos | É o padrão e não inicia nenhuma interface de calibração automaticamente. |
| `robos3_serial` | Testar inicialização automática Serial | Núcleo + chamada automática do console | Mantém compatibilidade com projetos que já usam a macro. |
| `robos3_web` | Testar inicialização automática Web | Núcleo + chamada automática do painel | Mantém compatibilidade com projetos que já usam a macro. |
| `robos3_calibration` | Validar Web e Serial juntas | Núcleo + ambas iniciadas automaticamente | Cobre a integração completa em CI/bancada. |
| `examples` | Base de compilação de um exemplo isolado | Biblioteca sem checagem agregada | Evita misturar sketch de exemplo ao firmware interno. |
| `example_serial` | Compilar `24_CalibracaoSerial` isoladamente | Exemplo do console | Verifica o fluxo didático separado. |
| `example_web` | Compilar `23_CalibracaoWeb` isoladamente | Exemplo do painel | Verifica o fluxo didático separado. |
| `examples_all` | Verificação de desenvolvimento/CI | Os 31 exemplos, isolados internamente | Detecta quebra de API sem gerar firmware para upload. |
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

Em toda inicialização, `begin()` também imprime a origem base da calibração, se há alterações em memória ainda não salvas e lista QTR, limiar, compensações/inversões dos motores, mapeamentos E/D, faixas ativas dos sensores de cor e offsets da IMU. Use `robo.printCalibracao()` para repetir esse relatório durante o diagnóstico.

### Serial e velocidade de comunicação

`robo.begin()` **não chama `Serial.begin()`**, não escolhe baud rate e não trava a velocidade da porta. O sketch continua responsável pela configuração:

```cpp
Serial.begin(9600);   // valido, se o Monitor Serial tambem usar 9600
// ou Serial.begin(115200);

robo.begin();         // apenas escreve o diagnostico na Serial ja configurada
```

Se `Serial.begin()` não for chamado antes, uma mensagem enviada pelo diagnóstico não inicializa a porta automaticamente; ela simplesmente pode não aparecer. No PlatformIO, `monitor_speed` deve ter o mesmo valor usado no sketch. O parâmetro de `begin(Print& diagnostico)` também permite redirecionar o texto para outro destino compatível com `Print`, mas a inicialização desse destino continua sendo responsabilidade do programa.

Uma falha não impede a configuração dos módulos restantes. Quando o programa precisa decidir se pode continuar parcialmente, consulte `status()`, `linhaDisponivel()`, `corDisponivel()`, `distanciaDisponivel(id)`, `ultrassomDisponivel()` e `imuDisponivel()`. `printErrosInicializacao()` continua disponível para reimprimir o diagnóstico sob demanda ou em outro objeto `Print`.

Ausência de calibração salva não é erro de hardware. Nesse caso, `status().calibracaoCarregada` será `false`, `printStatus()` mostrará `[PADRAO] Calibracao` e a biblioteca usará seus valores iniciais.

O exemplo `26_TratamentoErros` mostra o fluxo completo: imprime o diagnóstico, desativa recursos dependentes do módulo ausente e interrompe somente quando motores ou QTR não estão disponíveis.

## Motores e L298N

`mover(esquerda, direita)` recebe valores de -100 a +100. Positivo é frente; negativo é ré. `parar()` desliga enable e deixa INs baixos (coast). `frear()` coloca os dois INs de cada canal altos e enable em 100% (frenagem ativa).

Pipeline: comando → sentido → módulo → PWM mínimo → compensação → clamp → LEDC → L298N.

```cpp
robo.configurarMinimoMovimento(MOTOR_ESQUERDO, FRENTE, 18);
robo.configurarCompensacaoMotor(MOTOR_DIREITO, FRENTE, 0.96f);
robo.inverterMotores(false, true);
robo.trocarMotores(true); // somente se os conectores E/D foram trocados
```

`inverterMotores()` corrige polaridade: um motor que deveria avançar está girando para trás. `trocarMotores()` corrige lado: o comando esquerdo está acionando fisicamente o motor direito; por segurança, a chamada para os motores. As quatro combinações esquerda/direita e frente/ré são independentes. Zero sempre produz duty zero.

Configurações explícitas de inversão, compensação e mínimo feitas pelo sketch têm prioridade por campo sobre a NVS, inclusive quando são chamadas antes de `begin()`. Os demais campos continuam vindo da calibração salva. Por exemplo:

```cpp
robo.inverterMotorDireito(true);
robo.begin();
```

Mesmo que a NVS tenha `invertido=false`, o relatório mostrará `SALVA NA NVS + AJUSTES DO CODIGO` e `Motor D: ... invertido=sim ajustesCodigo=inversao`. Depois de `salvarCalibracao()`, o valor efetivo passa a estar na própria NVS e deixa de ser marcado como ajuste temporário do código.

## Sensor de linha

O padrão usa QTR-8A com aquisição nativa de **12 bits** no ESP32-S3: a leitura bruta vai de `0…4095`. A RoboS3 não acessa nem altera o membro privado `_maxValue` da QTRSensors. No modo analógico, ela lê os oito GPIOs diretamente, faz a média de quatro amostras e mantém calibração própria; a QTRSensors continua sendo usada no modo QTR-8RC. Assim, uma atualização da dependência não volta a limitar o QTR-8A a 10 bits. Após calibração, a escala pública permanece `0…1000`, portanto limiar, máscara, posição e PID continuam compatíveis.

Uma escala nominal de 12 bits fornece mais níveis, mas não garante 12 bits efetivos de precisão: ruído, alimentação e características do ADC ainda influenciam a medida. Calibre o conjunto na pista real. Calibrações QTR-8A salvas pela versão 0.4.1 ou anterior são convertidas automaticamente de `0…1023` para `0…4095` ao carregar; chame `salvarCalibracao()` para persistir definitivamente o novo formato.

```cpp
if (!robo.calibrarLinha(5000)) // tempo em ms
{
    Serial.println("Calibracao invalida; calibracao anterior mantida.");
    return;
}
robo.configurarLimiarLinha(500);
LeituraLinha linha = robo.lerLinha();
```

`calibrarLinha()` e `calibrarLinhaAmostras()` retornam `true` somente quando os oito sensores produzem faixas com minimo menor que maximo. Se algum canal nao variar o suficiente, retornam `false` e preservam a calibracao valida anterior, quando houver. Confira o retorno antes de salvar.

`lerLinha()` faz uma aquisição física bruta e deriva dela valores calibrados, `BBBPPBBB`, máscara, contagem e posição. Na ordem lógica, S1 é o bit 7 e S8 é o bit 0; com espelhamento habilitado, essa ordem acompanha esquerda→direita. A posição vai de 0 a 7000, com centro 3500. Há suporte a `LINHA_PRETA` e `LINHA_BRANCA`, QTR-8A e QTR-8RC, pinagem customizada e getter const da última leitura.

Se a barra QTR estiver montada com S1 à direita e S8 à esquerda, espelhe a ordem lógica em cada inicialização, antes de `begin()`. As leituras brutas, calibradas, máscara, padrão P/B, posição e histórico passam a seguir a ordem lógica esquerda→direita; as calibrações por sensor continuam associadas ao canal físico correspondente.

```cpp
robo.espelharSensoresLinha(); // S1...S8 montados da direita para a esquerda
robo.begin();
```

### Qual leitura de linha usar

| Chamada | Faz nova aquisição? | Retorno | Quando usar |
|---|---:|---|---|
| `lerLinha()` | Sim | `LeituraLinha` completa | Quando precisa de dois ou mais dados: bruto, calibrado, posição, máscara, nome P/B ou tempo sem linha. É a leitura principal. |
| `lerLinhaBruta(saida)` | Sim | Preenche `uint16_t saida[8]` | Diagnóstico elétrico, conferência da ordem S1–S8 e contraste sem calibração. |
| `lerLinhaCalibrada(saida)` | Sim | Preenche `uint16_t saida[8]` em `0…1000` | Observar o resultado da calibração sensor por sensor. |
| `lerLinhaBinaria()` | Sim | `uint8_t` | Quando deseja somente bits, por exemplo `0b00011000`. |
| `lerPadraoLinha()` | Sim | `PadraoLinha` | Quando deseja escrever diretamente `BBBPPBBB` em um `if` ou `switch`. |
| `ultimaLeituraLinha()` | Não | Referência para a última `LeituraLinha` | Reutilizar a aquisição anterior sem ler fisicamente o QTR outra vez. |

### Campos e funções de `LeituraLinha`

`linha.posicao` não é um submétodo: é um campo da estrutura retornada. Uma única chamada a `lerLinha()` preenche todos os campos abaixo, que também possuem documentação Doxygen exibida ao passar o mouse na IDE:

```cpp
LeituraLinha linha = robo.lerLinha();

if (linha.posicaoValida)
{
    Serial.println(linha.posicao);
}
```

| Campo ou função | Tipo/escala | Como usar |
|---|---|---|
| `bruto[8]` | `uint16_t[8]` | Aquisição na ordem lógica S1…S8; com espelhamento habilitado, segue esquerda→direita. QTR-8A usa `0…4095`, enquanto QTR-8RC retorna tempo de descarga. |
| `calibrado[8]` | `uint16_t[8]`, `0…1000` | Valores normalizados na ordem lógica S1…S8. |
| `mascara` | `uint8_t` | S1 lógico é bit 7, S8 lógico é bit 0 e preto físico vale 1. |
| `caso` | `PadraoLinha` | A mesma máscara com nome legível, como `BBBPPBBB`. |
| `padrao[9]` | texto terminado em zero | Representação P/B pronta para impressão. |
| `quantidadePretos` | `0…8` | Quantidade de sensores sobre preto físico. |
| `posicao` | `0…7000` | Posição ponderada; 3500 é o centro. Confira `posicaoValida`. |
| `detectada` | `bool` | Indica que a leitura atual encontrou contraste suficiente de linha. |
| `posicaoValida` | `bool` | Indica que `posicao` pode ser usada no controle. |
| `timestampMs` | milissegundos | Instante da aquisição segundo `millis()`. |
| `tempoSemLinhaMs` | milissegundos | Tempo desde a última detecção válida. |
| `ultimaPosicaoValida` | `0…7000` | Última posição confiável, útil para decidir o lado de busca após uma perda. |
| `semPreto()` | `bool` | `true` quando nenhum dos oito sensores observa preto físico. |
| `todosPretos()` | `bool` | `true` quando todos observam preto físico. |
| `temPretoEsquerda()` | `bool` | Testa S1, S2 e S3. |
| `temPretoCentro()` | `bool` | Testa S4 e S5. |
| `temPretoDireita()` | `bool` | Testa S6, S7 e S8. |

### Padrões P/B e binários

`PadraoLinha` possui os 256 casos possíveis. `P` significa preto físico/bit `1`, `B` significa branco físico/bit `0`, e os caracteres seguem a ordem lógica S1 até S8:

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

## Sensores de distância

A mesma API atende aos dois HC-SR04 e a até dois VL53L0X. Cada sensor recebe um ID numérico escolhido pela equipe; depois disso, o algoritmo não precisa saber qual tecnologia realizou a medida.

```cpp
const uint8_t FRENTE = 10;
const uint8_t TRAS = 11;
const uint8_t LATERAL = 20;

void setup()
{
    robo.adicionarSensorDistancia(FRENTE, SENSOR_ULTRASSONICO, 17, 18);
    robo.adicionarSensorDistancia(TRAS, SENSOR_ULTRASSONICO, 16, 42);

    // O TCS direito tambem usa 0x29; libere o I2C1 para o VL53L0X.
    robo.habilitarSensorCor(COR_DIREITA, false);
    robo.adicionarSensorDistancia(LATERAL, SENSOR_TOF_VL53L0X,
                                  BARRAMENTO_I2C1, 100);

    robo.begin();
}

void loop()
{
    LeituraDistancia leitura = robo.lerDistancia(FRENTE);
    if (leitura.valida)
    {
        Serial.println(leitura.centimetros);
    }

    if (robo.temObstaculo(LATERAL, 15.0f))
    {
        robo.parar();
    }
}
```

`LeituraDistancia` contém `id`, `sensor`, `tipo`, `centimetros`, `milimetros`, `valida`, `timeout` e `timestampMs`. Verifique `valida` antes de usar a distância. Em leitura inválida, `centimetros` vale `NAN` e `milimetros` vale zero. `timeout` distingue a expiração de uma leitura VL53L0X de um sensor apenas indisponível. O HC-SR04 dispara no máximo uma medição a cada 60 ms; chamadas mais rápidas reutilizam o último resultado, e `timestampMs` informa quando ocorreu a última medição real.

Os IDs padrões são `DISTANCIA_ULTRASSOM_1`/`DISTANCIA_ULTRASSOM` para ULTRA1 e `DISTANCIA_ULTRASSOM_2` para ULTRA2. Os métodos históricos sem ID, como `distanciaCM()` e `temObstaculo(20)`, continuam usando ULTRA1. Os dois HC-SR04 são habilitados por padrão; os VL53L0X são opcionais e só participam do `begin()` depois de cadastrados ou configurados. Há dois slots para cada tecnologia; IDs de 0 a 254 precisam ser únicos, pois 255 é reservado para `DISTANCIA_DESCONHECIDA`. `adicionarSensorDistancia()` retorna `false` quando o cadastro é inválido ou não existe slot livre.

### Regra do endereço I2C 0x29

TCS34725 e VL53L0X iniciam no endereço `0x29`. Sem multiplexador ou isolamento elétrico, só pode haver **um desses dispositivos em cada barramento**. Para usar um VL53L0X no I2C0, desative o TCS esquerdo; para usar no I2C1, desative o TCS direito. Se houver conflito, `begin()` não tenta conversar com os dispositivos daquele endereço, retorna `false` e explica o problema na Serial. A MPU6050 usa `0x68` e pode continuar no mesmo barramento.

A biblioteca Pololu permite selecionar `Wire` ou `Wire1`, e o driver RoboS3 usa aquisição contínua com timeout configurável. A chamada de leitura do VL53L0X é bloqueante até chegar uma medida ou expirar o timeout. O HC-SR04 também é bloqueante durante o tempo de eco; evite consultar sensores de distância mais vezes do que o controle realmente necessita.

## Botões digitais e analógicos

É possível cadastrar até 12 botões com IDs de `0` a `254`. A biblioteca mantém estado estável, eventos de apertar/soltar, duração do pressionamento e contador, sem alocação dinâmica. `robo.atualizar()` já processa o debounce; chame-a frequentemente no `loop()`.

### Botão digital com pull-up

Ligue um terminal do botão ao GPIO e o outro ao GND. Não é necessário resistor externo: `adicionarBotao()` configura `INPUT_PULLUP`, portanto solto é `HIGH` e pressionado é `LOW`.

```cpp
robo.adicionarBotao(1, 35, 30); // ID 1, GPIO35, debounce de 30 ms

if (robo.botaoFoiApertado(1))
{
    // Executado uma vez por aperto.
}
```

### Vários botões em uma entrada analógica

Um teclado resistivo produz um valor ADC diferente para cada tecla. Cadastre todas no mesmo pino, com faixas inclusivas que não se sobreponham. A RoboS3 usa ADC de 12 bits (`0…4095`). Os valores dependem dos resistores e da alimentação: primeiro observe `robo.lerBotao(id).valorBruto`, depois deixe margem entre as faixas.

```cpp
robo.adicionarBotaoAnalogico(2, 4, 120, 220, 30);
robo.adicionarBotaoAnalogico(3, 4, 430, 540, 30);
```

Com a pinagem padrão desta placa, os GPIOs já ocupados são `1, 2, 3, 5–9, 11–18, 21, 39, 40, 42, 47 e 48`. Recomendações conservadoras:

| Uso | GPIOs sugeridos | Observações |
|---|---|---|
| Entrada analógica | **4 ou 10** | São ADC e não estão usados pela configuração padrão. Confirme se o GPIO escolhido está exposto no conector da sua revisão. |
| Entrada digital | **4, 10, 35, 36, 37, 38 ou 41** | Não conflitam com a configuração padrão informada. |
| Digital somente se UART0 estiver livre | 43 ou 44 | Normalmente associados a TX/RX; evite se usar essa UART. |

Evite GPIO0, GPIO3, GPIO45 e GPIO46 por serem pinos de *strapping*; evite GPIO19/20 quando usar USB nativo; e não use GPIO26–32, normalmente ligados à flash/PSRAM do módulo N8R2. Um cadastro retorna `false` em caso de ID/pino inválido ou duplicado, faixa analógica inválida/sobreposta, mistura digital/analógica no mesmo pino ou limite de 12 atingido.

`botaoPressionado(id)` permanece `true` enquanto pressionado. `botaoFoiApertado(id)` e `botaoFoiSolto(id)` consomem o evento e retornam `true` somente uma vez. `botaoPressionadoPor(id, ms)` testa um pressionamento longo. `lerBotao(id)` não consome eventos e retorna todos os campos. Veja `31_Botoes`.

## Cores: RGB, HSV e HSL

Dois TCS34725 de endereço `0x29` funcionam simultaneamente desde que estejam em barramentos separados. Por padrão, o esquerdo usa I2C0 (`Wire`) e o direito I2C1 (`Wire1`), mas essa atribuição é configurável antes de `begin()`. Uma aquisição gera raw, RGB 0…255, HSV/HSL, canal clear, classificação e timestamp.

Se os conectores esquerdo e direito foram montados ao contrário, chame `robo.trocarSensoresCor(true)` **antes** de configurar os barramentos e antes de `begin()`. O método retorna `false` depois da inicialização para evitar alterar silenciosamente o significado das calibrações em uso.

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

Para uma curva bloqueante controlada pelo giroscópio, use ângulo positivo para direita e negativo para esquerda:

```cpp
robo.inverterAnguloZ(false); // use true se a IMU estiver montada invertida

bool completou = robo.girarAngulo(40, 90);   // 90 graus para direita
completou = robo.girarAngulo(40, -90);       // 90 graus para esquerda
```

`girarAngulo()` retorna `false` se motores ou IMU estiverem indisponíveis ou se o timeout for atingido, evitando que o robô fique preso em um laço infinito. A função freia brevemente, deixa os motores parados ao terminar e é bloqueante; os valores reais ainda dependem da calibração da IMU, aderência, bateria e inércia.

## Persistência

`salvarCalibracao()`, `carregarCalibracao()` e `apagarCalibracao()` usam Preferences/NVS no namespace `robos3`. O blob contém magic, versão de schema, tamanho e checksum. A flash só é escrita por comando explícito.

`apagarCalibracao()` (ou o alias `resetarCalibracao()`) apaga o blob da NVS e restaura imediatamente, na RAM, os limites do QTR, limiar da linha, motores, faixas/ganho/integração de cor e offsets da IMU. Retorna `true` quando a remoção da NVS foi concluída; os padrões em RAM são restaurados mesmo se não existia calibração salva.

## Calibração Serial e Web

Prefira ativar diretamente no sketch, depois de `robo.begin()`:

```cpp
robo.iniciarCalibracaoSerial(Serial); // usa a porta ja iniciada pelo sketch
robo.iniciarCalibracaoWeb();          // cria o ponto de acesso e o painel
```

Não é necessário alterar `platformio.ini` nem usar `#define`. As macros `ROBOS3_CALIBRATION_SERIAL` e `ROBOS3_CALIBRATION_WEB` continuam disponíveis apenas como atalhos de inicialização automática para projetos antigos. Um `#define` escrito somente no `.ino` não configura os outros `.cpp` da biblioteca, por isso o método público é a solução portátil. Ambas as interfaces podem ser usadas juntas. No `loop`, chame `robo.atualizar()`.

O console Serial opera na velocidade escolhida pelo próprio `Serial.begin(...)` e oferece `help`, `status`, linha, motores, cores, IMU, `save`, `load` e `reset`, com parser de buffer fixo. Web cria `RoboS3-Calibracao-XXXX`, serve dashboard responsivo e APIs `/api/status`, `/api/line`, `/api/motors`, `/api/colors` e `/api/imu`. Testes de motor exigem ação explícita e param/freiam automaticamente em até 2 s se não forem renovados.

## Debug

Defina `ROBOS3_DEBUG_LEVEL` como 0…3 via build flag. O modo normal não imprime continuamente. A única saída automática é o diagnóstico de falha do `begin()`. `printLinha`, `printCores`, `printIMU`, `printMotores`, `printStatus` e `printErrosInicializacao` podem ser chamados explicitamente.

## Override de pinos

Use antes de `begin()`. Os valores de QTR e ponte H abaixo são os padrões atualmente definidos em `BoardPins.h`:

```cpp
robo.definePinsQTR(1, 2, 3, 5, 6, 7, 8, 9);
robo.definePinsPonteH(14, 15, 11, 12, 13, 21);
```

Defina I2C e ultrassom somente com GPIOs físicos confirmados e que não estejam usados acima. Por exemplo, para colocar a MPU6050 no I2C1:

```cpp
robo.definirBarramentoIMU(BARRAMENTO_I2C1);
robo.definirBarramentoCor(COR_ESQUERDA, BARRAMENTO_I2C0);
robo.definirBarramentoCor(COR_DIREITA, BARRAMENTO_I2C1);
```

`definirBarramentoCor()` altera o barramento de cada TCS34725; `definirBarramentoIMU()` altera somente a MPU6050. A IMU (`0x68`) pode compartilhar qualquer barramento com um TCS ou VL53L0X (`0x29`). Sem multiplexador, cada barramento admite somente um dispositivo `0x29`; nesse caso `begin()` deixa os dispositivos conflitantes indisponíveis e informa o motivo pela Serial.

## Estratégias de competição

A biblioteca não afirma “encruzilhada”, “gap”, “T” ou “curva de 90°”. Uma máscara vazia pode significar vários eventos; muitos sensores pretos também. Ela entrega máscara, padrão, posição, quantidade, tempo sem linha e ângulos para o aluno confirmar eventos com estado e histórico.

Consulte `13_LogicaPista`, `21_MPU6050_Curva90`, `22_MPU6050_Rampa` e
`29_EsqueletoOBR`. O último é deliberadamente um esqueleto: mostra como separar
casos especiais do seguidor normal/PID, mas tempos, confirmações e regras devem
ser validados e completados pela equipe na pista real.

## Troubleshooting

- Motores invertidos: use `inverterMotores` antes de alterar a lógica do aluno.
- Motor não parte: aumente o PWM mínimo daquela direção.
- QTR sempre 0/1000: refaça calibração movendo todos os sensores sobre branco e preto.
- TCS ausente: confira se cada módulo está no barramento físico correto.
- IMU deriva em Z: calibre imóvel, zere antes de cada curva e mantenha giros curtos.
- Upload falha: GPIO0/strapping pode estar sendo afetado pela ponte H; desligue potência dos motores durante diagnóstico.

## Referência completa da API

Todos os métodos abaixo pertencem a `RoboS3`. Crie uma única instância, configure o que for necessário e chame `begin()` uma vez. Métodos de pinagem devem vir **antes** de `begin()`; os demais, depois dela.

As assinaturas mostram parâmetros obrigatórios e opcionais; valores entre parênteses indicam a unidade ou a faixa esperada. Métodos `void` produzem o efeito descrito e métodos com retorno informam explicitamente o tipo e o significado. Operações bloqueantes estão marcadas. O contrato definitivo da API fica em [`src/RoboS3.h`](src/RoboS3.h).

### Documentação exibida pela IDE

Todos os elementos públicos da biblioteca usam comentários Doxygen. VS Code/PlatformIO e IDEs compatíveis exibem `@brief`, parâmetros, retorno, unidades e observações ao passar o mouse sobre classes, tipos ou métodos. Ao adicionar uma API pública, mantenha este padrão:

```cpp
/**
 * @brief Explica objetivamente o que a função faz.
 * @param valor Informa significado, unidade e faixa válida.
 * @return Explica o tipo retornado e também valores de falha.
 * @note Registra efeitos colaterais, bloqueios ou ordem obrigatória de chamadas.
 */
bool exemplo(uint16_t valor);
```

Use `@return` em funções não `void`, documente cada `@param`, declare unidades (`ms`, `cm`, graus, porcentagem) e informe quando uma função faz aquisição física, altera histórico, grava flash ou bloqueia o programa.

### Tipos, faixas e convenções dos parâmetros

| Parâmetro | Tipo C++ | Faixa/unidade e tratamento |
|---|---|---|
| velocidades de motor | `int` | `-100…100`; valores externos são limitados. Positivo avança, negativo recua. Em métodos de sentido fixo é usado o módulo. |
| `id` de distância/botão | `uint8_t` | `0…254`; `255` é reservado como inválido. IDs devem ser únicos dentro de cada gerenciador. |
| GPIO (`pino`, `sda`, `scl`, `trigger`, `echo`) | `uint8_t` | Número físico `0…48`; a validade elétrica depende do ESP32-S3 e da placa. `255` só é aceito como ausência do emitter QTR. |
| tempo/debounce/timeout | `uint16_t` ou `uint32_t` | Milissegundos. O tipo exato aparece na assinatura de cada método; contagens usam aritmética segura com `millis()`. |
| QTR calibrado/limiar | `uint16_t` | `0…1000`; posição da linha `0…7000`, centro `3500`. |
| ADC de botão | `uint16_t` | `0…4095`, limites inclusivos e não sobrepostos no mesmo pino. |
| ângulo, distância, ganhos e cores | `float` | Ângulo em graus; distância em cm; HSV/HSL: H `0…360`, S/V/L `0…100`; compensação de motor `1.0f` é neutra. |
| seletores | `enum` | Use as constantes documentadas, como `MOTOR_ESQUERDO`, `COR_DIREITA` e `BARRAMENTO_I2C1`, em vez de números soltos. |
| saída textual | `Print&`/`Stream&` | `Serial` por padrão; o sketch deve inicializar a porta e escolher sua velocidade. |

### 1. Sistema, status e depuração

| Método | Aplicação |
|---|---|
| `RoboS3()` | Cria o objeto com a pinagem padrão da PCB. |
| `begin(diagnostico)` | Inicializa todos os módulos. Retorna `true` somente se todos responderam e imprime automaticamente as falhas no `Print` recebido, `Serial` por padrão. |
| `atualizar()` | Atualiza IMU, debounce dos botões e interfaces Serial/Web iniciadas. Chame continuamente no `loop()`. |
| `bool iniciarCalibracaoSerial(Stream& stream = Serial)` | Ativa o console na porta já inicializada; retorna `false` se chamado antes de `begin()`. |
| `bool iniciarCalibracaoWeb()` | Ativa SoftAP/painel sem build flag; retorna `false` antes de `begin()` ou se o servidor falhar. |
| `status()` | Retorna `const StatusRobo&` com o estado de todos os módulos. |
| `inicializacaoCompleta()` | Retorna `true` quando todos os módulos previstos responderam ao `begin()`. |
| `operacaoBasicaDisponivel()` | Retorna `true` quando motores e QTR estão disponíveis para um seguidor simples. |
| `linhaDisponivel()` / `ultrassomDisponivel()` / `imuDisponivel()` | Informam se o periférico iniciou corretamente; `ultrassomDisponivel()` consulta ULTRA1. |
| `distanciaDisponivel(id)` | Consulta qualquer HC-SR04 ou VL53L0X cadastrado. |
| `corDisponivel(sensor)` | Informa se o TCS34725 de `COR_ESQUERDA` ou `COR_DIREITA` está disponível. |
| `printStatus(out)` | Imprime o estado dos módulos; o destino padrão é `Serial`. |
| `printErrosInicializacao(out)` | Imprime somente os módulos que falharam e uma indicação do ponto a verificar. |
| `void printCalibracao(Print& out = Serial) const` | Imprime origem base, valores QTR/motores/cores/IMU e se há alterações em memória ainda não salvas; `begin()` já chama automaticamente. |
| `setDebugLevel(nivel)` | Seleciona `DEBUG_OFF`, `DEBUG_ERROR`, `DEBUG_INFO` ou `DEBUG_VERBOSE`. |

### 2. Motores e ponte H L298N

Velocidades aceitam `-100` a `100`: positivo é frente, negativo é ré e zero para o motor.

| Método | Aplicação |
|---|---|
| `mover(esquerda, direita)` | Comanda os dois motores independentemente; base para curvas suaves. |
| `frente(velocidade)` / `tras(velocidade)` | Deslocamento retilíneo. |
| `girarEsquerda(velocidade)` / `girarDireita(velocidade)` | Giro no próprio eixo. |
| `girarAngulo(velocidade, anguloZ, timeoutMs)` | Giro bloqueante pela MPU6050; positivo é direita, negativo é esquerda. Retorna `false` sem motores/IMU ou por timeout. |
| `motorEsquerdo(velocidade)` / `motorDireito(velocidade)` | Atualiza somente um motor e preserva o comando do outro. |
| `parar()` | Desliga enables: parada livre (*coast*). |
| `frear()` | Aplica frenagem ativa na ponte H. |
| `void inverterMotorEsquerdo(bool invertido)` / `void inverterMotorDireito(bool invertido)` | Inverte eletricamente um motor; o valor explícito prevalece sobre a NVS. |
| `void inverterMotores(bool esquerdo, bool direito)` | Configura as duas inversões de uma vez, com prioridade sobre a NVS. |
| `void trocarMotores(bool trocados = true)` | Para os motores e troca o mapeamento lógico E/D; não altera a polaridade. |
| `void configurarMinimoMovimento(Motor motor, SentidoMotor sentido, uint8_t minimo)` | Define PWM mínimo `0…100`; o campo explícito prevalece sobre a NVS. |
| `void configurarCompensacaoMotor(Motor motor, SentidoMotor sentido, float fator)` | Aplica multiplicador; `1.0` é neutro e o campo explícito prevalece sobre a NVS. |
| `printMotores(out)` | Mostra comandos e calibração atual. |

### 3. Sensor de linha QTR e pista

| Método | Aplicação |
|---|---|
| `lerLinha()` | Retorna `LeituraLinha`: dados brutos, calibrados, máscara, padrão, posição e tempo sem linha. |
| `lerLinhaBruta(saida)` | Preenche `uint16_t saida[8]` com a leitura física. |
| `lerLinhaCalibrada(saida)` | Preenche `uint16_t saida[8]` com valores `0…1000`. |
| `lerLinhaBinaria()` | Retorna a máscara de sensores pretos: S1 lógico é bit 7 e S8 lógico é bit 0. |
| `lerPadraoLinha()` | Retorna diretamente um `PadraoLinha`, como `BBBPPBBB`. |
| `ultimaLeituraLinha()` | Retorna a última `LeituraLinha`, sem nova aquisição. |
| `tempoSemLinha()` | Retorna milissegundos desde a última detecção válida. |
| `configurarHistoricoLinha(tamanho)` | Limpa e configura entre 0 e 32 mudanças armazenadas; zero desativa. |
| `limparHistoricoLinha()` | Remove todos os casos registrados. |
| `capacidadeHistoricoLinha()` / `quantidadeHistoricoLinha()` | Informam espaço configurado e quantidade atualmente válida. |
| `copiarUltimosCasosLinha(saida, maximo)` | Copia somente `PadraoLinha`, do mais novo para o mais antigo. |
| `copiarHistoricoLinha(saida, maximo)` | Copia `CasoLinha` com início e duração, do mais novo para o mais antigo. |
| `padraoLinhaTexto(caso, saida)` | Converte qualquer enum P/B para `char saida[9]`. |
| `calibrarLinha(ms)` | Para os motores e coleta extremos por tempo; retorna `bool`, é bloqueante e usa 5000 ms por padrão. |
| `calibrarLinhaAmostras(amostras)` | Variante bloqueante por quantidade de amostras; retorna `bool`. |
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

### 5. Distância e obstáculo (HC-SR04 e VL53L0X)

| Método | Aplicação |
|---|---|
| `distanciaCM()` | Retorna centímetros ou `NAN` quando a leitura falha. |
| `temObstaculo(maximoCm)` | Retorna `true` somente para medida válida até o limite. |
| `adicionarSensorDistancia(id, SENSOR_ULTRASSONICO, trig, echo, maximoCm)` | Cadastra um dos dois slots HC-SR04 com ID próprio. |
| `adicionarSensorDistancia(id, SENSOR_TOF_VL53L0X, barramento, timeoutMs)` | Cadastra um dos dois slots VL53L0X com ID próprio. |
| `removerSensorDistancia(id)` | Desabilita o sensor correspondente antes de `begin()`. |
| `lerDistancia(id)` | Retorna `LeituraDistancia`, independentemente da tecnologia cadastrada. |
| `distanciaCM(id)` | Atalho que retorna centímetros ou `NAN`. |
| `temObstaculo(id, maximoCm)` | Testa o limite usando o sensor identificado. |
| `distanciaDisponivel(id)` | Informa se o sensor cadastrado respondeu ao `begin()`. |

### 6. Botões digitais e teclado analógico

| Assinatura C++ | Parâmetros, retorno e aplicação |
|---|---|
| `bool adicionarBotao(uint8_t id, uint8_t pino, uint16_t debounceMs = 30)` | Digital entre GPIO e GND com `INPUT_PULLUP`; ID `0…254`, debounce em ms. Retorna `false` para conflito ou limite. |
| `bool adicionarBotaoAnalogico(uint8_t id, uint8_t pino, uint16_t valorMinimo, uint16_t valorMaximo, uint16_t debounceMs = 30)` | Cadastra uma faixa ADC inclusiva `0…4095`; várias teclas podem compartilhar o pino com faixas separadas. |
| `void atualizarBotoes()` | Atualiza o debounce; é desnecessário se `atualizar()` já é chamado no `loop()`. |
| `LeituraBotao lerBotao(uint8_t id)` | Retorna estado, valor bruto, eventos, tempos e contador sem consumir eventos; `valido=false` para ID ausente. |
| `bool botaoPressionado(uint8_t id)` | `true` enquanto o estado filtrado permanecer pressionado. |
| `bool botaoFoiApertado(uint8_t id)` | Consome e retorna uma vez cada transição estável de solto para pressionado. |
| `bool botaoFoiSolto(uint8_t id)` | Consome e retorna uma vez cada transição estável de pressionado para solto. |
| `bool botaoPressionadoPor(uint8_t id, uint32_t tempoMs)` | Testa se o pressionamento atual já atingiu a duração informada. |
| `bool botaoDisponivel(uint8_t id) const` | Informa se o ID está cadastrado. |
| `bool removerBotao(uint8_t id)` / `void limparBotoes()` | Remove um cadastro ou todos os 12 slots. |

### 7. Cores: dois TCS34725, RGB, HSV e HSL

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
| `bool trocarSensoresCor(bool trocados = true)` | Troca os lados lógicos antes de `begin()`; retorna `false` se o robô já iniciou. |
| `printCores(out)` | Imprime os dois sensores. |

### 8. Orientação e inclinação (MPU6050)

Use `atualizar()` no `loop()` ou `atualizarIMU()` antes de consultar ângulos. O yaw/Z é relativo, não uma bússola.

| Método | Aplicação |
|---|---|
| `lerIMU()` | Atualiza e retorna `LeituraIMU`: aceleração, giroscópio, temperatura e ângulos. |
| `atualizarIMU()` | Atualiza somente a estimativa da IMU. |
| `roll()` / `pitch()` | Retornam inclinação lateral e longitudinal em graus. |
| `anguloZ()` | Retorna yaw/Z relativo desde a última zeragem. |
| `zerarAnguloZ()` | Define o yaw atual como zero; use antes de curvas controladas. |
| `inverterAnguloZ(invertido)` | Corrige o sinal do eixo Z quando a orientação física da IMU está invertida. Não muda a regra: positivo continua sendo direita. |
| `estaInclinado(graus)` | Retorna `true` se `abs(roll)` ou `abs(pitch)` atingir o limite. |
| `calibrarIMU(ms)` | Para motores e calibra offsets com o robô imóvel; padrão 3000 ms. É bloqueante. |
| `printIMU(out)` | Imprime dados atuais da IMU. |

### 9. Persistência de calibração

| Método | Aplicação |
|---|---|
| `salvarCalibracao()` | Salva limites QTR, calibração dos motores, faixas de cor e offsets da IMU no NVS. |
| `carregarCalibracao()` | Carrega e aplica os valores gravados; retorna `false` se não houver dado válido. |
| `bool apagarCalibracao()` / `bool resetarCalibracao()` | Apaga a NVS e restaura os padrões em RAM; retorna o resultado da remoção persistente. |

### 10. Configuração de hardware e pinagem

Chame estes métodos somente antes de `begin()`.

| Método | Aplicação |
|---|---|
| `definePinsQTR(s1, …, s8, emitter)` | Define os oito GPIOs QTR e emitter opcional; use `0xFF` quando não existir emitter. |
| `setTipoQTR(tipo)` | Seleciona `QTR_8A` ou `QTR_8RC`. |
| `setTipoLinha(tipo)` | Seleciona `LINHA_PRETA` ou `LINHA_BRANCA`. |
| `espelharSensoresLinha(espelhados)` | Espelha a ordem lógica S1…S8 sem alterar os GPIOs configurados. |
| `definePinsPonteH(ena, in1, in2, in3, in4, enb)` | Define toda a pinagem L298N. |
| `definePinsUltrassom(trigger, echo)` | Compatibilidade: redefine o ULTRA1. |
| `configurarUltrassom(sensor, trigger, echo, maximoCm)` | Configura `DISTANCIA_ULTRASSOM_1` ou `_2`. |
| `habilitarUltrassom(sensor, habilitado)` | Ativa ou desativa um dos conectores HC-SR04. |
| `definePinsI2C0(sda, scl, frequencia)` / `definePinsI2C1(sda, scl, frequencia)` | Define cada barramento I2C; padrão 400000 Hz. |
| `definirBarramentoCor(sensor, barramento)` | Atribui cada TCS a `BARRAMENTO_I2C0` ou `BARRAMENTO_I2C1`. Os dois TCS devem ficar separados por causa do endereço `0x29`. |
| `habilitarSensorCor(sensor, habilitado)` | Libera o endereço `0x29` de um barramento para um VL53L0X. |
| `configurarVL53L0X(sensor, barramento, timeoutMs)` | Configura `DISTANCIA_VL53_1` ou `_2`. Para IDs próprios, prefira `adicionarSensorDistancia()`. |
| `desabilitarVL53L0X(sensor)` | Desativa um slot VL53L0X antes de `begin()`. |
| `definirBarramentoIMU(barramento)` | Atribui a MPU6050 a I2C0 ou I2C1. Ela pode compartilhar o barramento com um TCS. |
| `configurarIMU(endereco)` | Define endereço I2C MPU6050, normalmente `0x68`. |

### 11. Tipos retornados

| Tipo | Campos ou valores principais |
|---|---|
| `Motor` / `SentidoMotor` | `MOTOR_ESQUERDO`, `MOTOR_DIREITO`; `FRENTE`, `TRAS`. |
| `TipoQTR` / `TipoLinha` | `QTR_8A`, `QTR_8RC`; `LINHA_PRETA`, `LINHA_BRANCA`. |
| `SensorCor` / `BarramentoI2C` / `CorDetectada` | Lado esquerdo/direito; `BARRAMENTO_I2C0` ou `BARRAMENTO_I2C1`; preta, branca, vermelha, verde, azul, amarela, laranja, roxa ou desconhecida. |
| `RGB`, `HSV`, `HSL` | Conversões retornadas pelas leituras de cor. |
| `PadraoLinha` | Todos os 256 nomes P/B; `BBBPPBBB` equivale a `0b00011000`. |
| `LeituraLinha` | `bruto`, `calibrado`, `mascara`, `caso`, texto `padrao`, contagem, posição, validade e tempos. |
| `CasoLinha` | `padrao`, `inicioMs` e `duracaoMs` de uma mudança registrada. |
| `LeituraCor` | `rgb`, `hsv`, `hsl`, raw, `clear`, `cor`, `valida`, `timestampMs`. |
| `LeituraIMU` | `accX/Y/Z`, `gyroX/Y/Z`, `roll`, `pitch`, `yawRelativo`, `temperatura`, `valida`, `timestampMs`. |
| `SensorDistancia` / `TipoSensorDistancia` | Slots físicos e tecnologias `SENSOR_ULTRASSONICO`/`SENSOR_TOF_VL53L0X`. |
| `LeituraDistancia` | ID, slot, tecnologia, centímetros, milímetros, validade, timeout e timestamp. |
| `TipoBotao` / `LeituraBotao` | Digital pull-up ou analógico; ID, pino, valor bruto, estado, eventos, durações e contador. |
| `StatusRobo` | Flags de disponibilidade dos módulos e da calibração salva; também oferece `inicializacaoCompleta()` e `operacaoBasicaDisponivel()`. |

#### Unidades e validade dos campos

| Estrutura/campo | Unidade ou domínio | Observação |
|---|---|---|
| `LeituraLinha.bruto[8]` | leitura bruta do QTR | Ordem lógica S1…S8; com espelhamento, esquerda→direita. QTR-8A usa ADC `0…4095`; QTR-8RC usa tempo de descarga. |
| `LeituraLinha.calibrado[8]` | `0…1000` | Valores normalizados na ordem lógica S1…S8. |
| `LeituraLinha.mascara` / `.caso` | byte / `PadraoLinha` | S1 lógico é bit 7, S8 lógico é bit 0 e preto físico vale 1. |
| `LeituraLinha.padrao[9]` | texto P/B | O nono caractere é o terminador nulo. |
| `LeituraLinha.quantidadePretos` | `0…8` | Quantidade de bits pretos na máscara. |
| `LeituraLinha.posicao` | `0…7000` | Centro geométrico 3500; confira `posicaoValida` antes de usar. |
| `LeituraLinha.detectada` / `.posicaoValida` | booleano | Informam se houve linha e se a posição pode orientar os motores. |
| `LeituraLinha.ultimaPosicaoValida` | `0…7000` | Posição anterior preservada durante uma perda. |
| `LeituraLinha.timestampMs` / `.tempoSemLinhaMs` | milissegundos | Baseados em `millis()` e sujeitos ao rollover normal do Arduino. |
| `CasoLinha.padrao` | `PadraoLinha` | Padrão observado naquela permanência. |
| `CasoLinha.inicioMs` / `.duracaoMs` | milissegundos | O caso atual continua acumulando duração até a consulta. |
| `LeituraCor.rgb` | `0…255` | Canais normalizados pelo canal clear. |
| `LeituraCor.hsv` | H `0…360`; S/V `0…100` | Confira `valida` antes de classificar ou controlar o robô. |
| `LeituraCor.hsl` | H `0…360`; S/L `0…100` | Conversão derivada da mesma aquisição RGB. |
| `LeituraCor.rRaw/gRaw/bRaw/clear` | `uint16_t` cru | Canais entregues diretamente pelo TCS34725. |
| `LeituraCor.cor` | `CorDetectada` | Resultado das faixas HSV ativas. |
| `LeituraCor.valida` | booleano | `false` quando o sensor selecionado não está disponível. |
| `LeituraIMU.accX/Y/Z` | m/s² | Aceleração calculada com a escala configurada da MPU6050. |
| `LeituraIMU.gyroX/Y/Z` | graus/s | Velocidade angular após aplicação dos offsets. |
| `LeituraIMU.roll/pitch/yawRelativo` | graus | Yaw é relativo e deriva; não é direção magnética. |
| `LeituraIMU.temperatura` | °C | Temperatura interna do chip, não do ambiente. |
| `LeituraIMU.valida` | booleano | `false` quando a MPU6050 não está disponível. |
| `LeituraDistancia.centimetros/milimetros` | cm / mm | `NAN`/zero quando `valida` é false. |
| `LeituraDistancia.id/sensor/tipo` | identificadores | Separam o nome escolhido, o slot físico e a tecnologia usada. |
| `LeituraDistancia.valida/timeout` | booleanos | `timeout` indica especificamente expiração do VL53L0X. |
| `LeituraBotao.id/pino/tipo` | identificadores | Repetem o cadastro; confira `valido` antes dos demais campos. |
| `LeituraBotao.valorBruto` | digital `0/1` ou ADC `0…4095` | Permite diagnosticar ligação e escolher faixas do teclado resistivo. |
| `LeituraBotao.pressionado` | booleano | Estado estável depois do debounce. |
| `LeituraBotao.eventoApertado/eventoSolto` | booleanos | Apenas indicam evento pendente; a leitura da estrutura não o consome. |
| `LeituraBotao.duracaoPressionadoMs/duracaoUltimoApertoMs` | milissegundos | Duração atual ou duração completa do último aperto. |
| `LeituraBotao.quantidadeApertos` | `uint32_t` | Total de transições filtradas para pressionado desde o cadastro. |
| `StatusRobo.motores/linha/ultrassom/ultrassom2` | booleano | Disponibilidade dos módulos sem I2C; `ultrassom` é ULTRA1. |
| `StatusRobo.i2c0/i2c1` | booleano | Resultado da inicialização de cada controlador I2C. |
| `StatusRobo.corEsquerda/corDireita/imu/vl53_1/vl53_2` | booleano | Indica quais dispositivos responderam ao `begin()`. |
| `StatusRobo.*Habilitado` | booleano | Diferencia módulo opcional desativado de falha de inicialização. |
| `StatusRobo.calibracaoCarregada` | booleano | `false` usa os valores padrão; não representa falha de hardware. |
| `*.timestampMs` | milissegundos | Instante de aquisição segundo `millis()`. |

## Exemplos incluídos

Os 31 sketches em [`examples/`](examples) seguem uma progressão didática:

| Faixa | Aplicação |
|---|---|
| 01–04 | início, motores e calibração dos motores |
| 05 | sensor ultrassônico |
| 06–13 | QTR bruto/calibrado, padrões, PID e lógica de pista |
| 14–18 | RGB, HSV/HSL, detecção e calibração de cor |
| 19–22 | MPU6050, ângulos, curva de 90° e rampa |
| 23–24 | calibração Web e Serial |
| 25–28 | pinagem alternativa, erros do `begin()`, histórico e cor manual |
| 29 | esqueleto OBR: calibração, obstáculo, padrões P/B, PID, verde, T, encruzilhada e giros de 90° |
| 30 | cadastro por ID e leitura unificada de dois ultrassons e VL53L0X |
| 31 | botões digitais e teclado resistivo analógico com debounce e eventos |

## Limites de validação

Compilação não valida polaridade física dos motores, ordem esquerda/direita, divisor de tensão do ECHO, orientação mecânica do MPU, posição física S1/S8, presença dos sensores nem qualidade das faixas de cor. Esses pontos exigem placa conectada; nenhuma porta serial foi detectada durante o desenvolvimento.

## Licença e manutenção

Distribuída sob a [licença MIT](LICENSE). Ao alterar a API pública, atualize em conjunto o comentário Doxygen, esta referência, ao menos um exemplo relevante e o `CHANGELOG.md`; depois compile `robos3`, `robos3_calibration` e `examples_all` antes de publicar uma nova versão.
