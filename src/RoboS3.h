#pragma once

#include <Arduino.h>
#include "config/RoboConfig.h"
#include "types/RoboTypes.h"
#include "modules/ColorSensor.h"
#include "modules/IMUSensor.h"
#include "modules/LinePID.h"
#include "modules/LineSensor.h"
#include "modules/MotorDriver.h"
#include "modules/UltrasonicSensor.h"
#include "modules/VL53L0XSensor.h"
#include "modules/CalibrationStorage.h"
#include "modules/ButtonManager.h"
#include "debug/RoboDebug.h"

namespace robos3
{
class SerialCalibration;
class WebCalibration;
} // namespace robos3

/**
 * @brief Interface principal da placa RoboS3.
 *
 * Centraliza motores, QTR, ultrassom, sensores de cor, IMU, PID, diagnostico
 * e persistencia. Crie normalmente uma unica instancia global.
 */
class RoboS3
{
  public:
    /** @brief Cria o robo usando a pinagem e os valores padrao da placa. */
    RoboS3();

    /**
     * @brief Inicializa todos os modulos e imprime automaticamente as falhas.
     * @param diagnostico Destino das mensagens de erro; usa Serial por padrao.
     * @return true somente quando todos os modulos previstos responderam.
     * @note A biblioteca nao chama Serial.begin(). O sketch escolhe o baud rate
     * e deve iniciar a porta antes desta funcao para visualizar o diagnostico.
     */
    bool begin(Print& diagnostico = Serial);

    /** @brief Atualiza IMU, botoes e interfaces de calibracao iniciadas. */
    void atualizar();

    /**
     * @brief Inicia o console interativo de calibracao em uma porta serial.
     * @param stream Porta ja inicializada pelo sketch, por exemplo Serial.
     * @return true quando o robo ja passou por begin() e o console foi ativado.
     * @note Nao exige ROBOS3_CALIBRATION_SERIAL nem define em platformio.ini.
     */
    bool iniciarCalibracaoSerial(Stream& stream = Serial);

    /**
     * @brief Inicia o ponto de acesso e o painel Web de calibracao.
     * @return true quando o robo ja passou por begin() e o servidor foi iniciado.
     * @note Nao exige ROBOS3_CALIBRATION_WEB nem define em platformio.ini.
     */
    bool iniciarCalibracaoWeb();

    /**
     * @brief Substitui os oito GPIOs e o emissor do QTR.
     * @param s1 GPIO do primeiro sensor, o mais a esquerda.
     * @param s2 GPIO do segundo sensor.
     * @param s3 GPIO do terceiro sensor.
     * @param s4 GPIO do quarto sensor.
     * @param s5 GPIO do quinto sensor.
     * @param s6 GPIO do sexto sensor.
     * @param s7 GPIO do setimo sensor.
     * @param s8 GPIO do oitavo sensor, o mais a direita.
     * @param emitter GPIO do emissor, ou 0xFF quando nao houver controle.
     * @note Chame antes de begin().
     */
    void definePinsQTR(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint8_t s5, uint8_t s6,
                       uint8_t s7, uint8_t s8, uint8_t emitter = 0xFF);

    /**
     * @brief Seleciona a interface eletrica do conjunto QTR antes de begin().
     * @param type QTR_8A para sensores analogicos ou QTR_8RC para sensores RC.
     */
    void setTipoQTR(TipoQTR type)
    {
        config_.board.qtrType = type;
    }

    /**
     * @brief Seleciona a polaridade visual da pista.
     * @param type LINHA_PRETA ou LINHA_BRANCA.
     */
    void setTipoLinha(TipoLinha type);

    /**
     * @brief Espelha a ordem logica dos oito sensores de linha.
     * @param espelhados true quando S1...S8 estao montados da direita para a esquerda.
     * @note Chame em cada inicializacao antes de begin(); a calibracao e mantida.
     */
    void espelharSensoresLinha(bool espelhados = true);

    /**
     * @brief Substitui a pinagem completa da ponte H antes de begin().
     * @param ena Enable/PWM esquerdo. @param in1 Direcao esquerda 1.
     * @param in2 Direcao esquerda 2. @param in3 Direcao direita 1.
     * @param in4 Direcao direita 2. @param enb Enable/PWM direito.
     */
    void definePinsPonteH(uint8_t ena, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                          uint8_t enb);

    /**
     * @brief Substitui os GPIOs TRIGGER e ECHO do ultrassom antes de begin().
     * @param trigger GPIO do sinal TRIG. @param echo GPIO do sinal ECHO.
     */
    void definePinsUltrassom(uint8_t trigger, uint8_t echo);

    /**
     * @brief Configura um dos dois conectores ultrassonicos antes de begin().
     * @param sensor DISTANCIA_ULTRASSOM_1 ou DISTANCIA_ULTRASSOM_2.
     * @param trigger GPIO TRIG. @param echo GPIO ECHO.
     * @param maximoCm Maior distancia aguardada pelo driver.
     * @return false quando o identificador nao representa um ultrassom.
     */
    bool configurarUltrassom(SensorDistancia sensor, uint8_t trigger, uint8_t echo,
                             uint16_t maximoCm = 400);

    /**
     * @brief Habilita ou desabilita um conector ultrassonico antes de begin().
     * @param sensor DISTANCIA_ULTRASSOM_1 ou DISTANCIA_ULTRASSOM_2.
     * @param habilitado true para inicializar; false para ignorar o conector.
     * @return false quando o identificador nao representa um ultrassom.
     */
    bool habilitarUltrassom(SensorDistancia sensor, bool habilitado = true);

    /**
     * @brief Configura o barramento I2C0 antes de begin().
     * @param sda GPIO de dados. @param scl GPIO de clock.
     * @param frequency Frequencia do barramento em hertz.
     */
    void definePinsI2C0(uint8_t sda, uint8_t scl, uint32_t frequency = 400000);

    /**
     * @brief Configura o barramento I2C1 antes de begin().
     * @param sda GPIO de dados. @param scl GPIO de clock.
     * @param frequency Frequencia do barramento em hertz.
     */
    void definePinsI2C1(uint8_t sda, uint8_t scl, uint32_t frequency = 400000);

    /**
     * @brief Atribui um TCS34725 a um dos dois barramentos I2C antes de begin().
     * @param sensor COR_ESQUERDA ou COR_DIREITA.
     * @param barramento BARRAMENTO_I2C0 (Wire) ou BARRAMENTO_I2C1 (Wire1).
     * @note Os dois TCS34725 usam endereco 0x29 e nao podem compartilhar barramento.
     */
    void definirBarramentoCor(SensorCor sensor, BarramentoI2C barramento);

    /**
     * @brief Habilita ou desabilita um TCS34725 antes de begin().
     * @param sensor COR_ESQUERDA ou COR_DIREITA.
     * @param habilitado true para inicializar; false para liberar o barramento.
     * @note Desabilite o TCS do barramento que receber um VL53L0X: ambos usam 0x29.
     */
    void habilitarSensorCor(SensorCor sensor, bool habilitado = true);

    /**
     * @brief Habilita e atribui um slot VL53L0X a um barramento I2C.
     * @param sensor DISTANCIA_VL53_1 ou DISTANCIA_VL53_2.
     * @param barramento BARRAMENTO_I2C0 ou BARRAMENTO_I2C1.
     * @param timeoutMs Limite bloqueante de cada leitura; use valor maior que zero.
     * @return false quando o identificador nao representa um VL53L0X.
     * @note Cada barramento admite somente um dispositivo de endereco 0x29 sem multiplexador.
     */
    bool configurarVL53L0X(SensorDistancia sensor, BarramentoI2C barramento,
                           uint16_t timeoutMs = 100);

    /**
     * @brief Desabilita um slot VL53L0X.
     * @param sensor DISTANCIA_VL53_1 ou DISTANCIA_VL53_2.
     * @return false para um identificador invalido.
     */
    bool desabilitarVL53L0X(SensorDistancia sensor);

    /**
     * @brief Cadastra um ultrassom com um ID escolhido pela equipe.
     * @param id Numero unico de 0 a 254 usado depois em lerDistancia(id).
     * @param tipo Deve ser SENSOR_ULTRASSONICO.
     * @param trigger GPIO TRIG. @param echo GPIO ECHO.
     * @param maximoCm Maior distancia aguardada pelo driver.
     * @return true quando cadastrado; false por tipo, ID duplicado ou falta de slot.
     */
    bool adicionarSensorDistancia(uint8_t id, TipoSensorDistancia tipo, uint8_t trigger,
                                  uint8_t echo, uint16_t maximoCm = 400);

    /**
     * @brief Cadastra um VL53L0X com um ID escolhido pela equipe.
     * @param id Numero unico de 0 a 254 usado depois em lerDistancia(id).
     * @param tipo Deve ser SENSOR_TOF_VL53L0X.
     * @param barramento I2C0 ou I2C1 onde o VL53 esta ligado.
     * @param timeoutMs Limite de espera de uma leitura em milissegundos.
     * @return true quando cadastrado; false por tipo, ID duplicado ou falta de slot.
     */
    bool adicionarSensorDistancia(uint8_t id, TipoSensorDistancia tipo,
                                  BarramentoI2C barramento, uint16_t timeoutMs = 100);

    /**
     * @brief Desabilita o sensor cadastrado com o ID informado.
     * @param id Numero de 0 a 254 usado no cadastro.
     * @return true quando o ID foi encontrado.
     */
    bool removerSensorDistancia(uint8_t id);

    /**
     * @brief Cadastra um botao digital ligado entre GPIO e GND.
     * @param id ID unico de 0 a 254 usado nas consultas.
     * @param pino GPIO digital do ESP32-S3, de 0 a 48, configurado como INPUT_PULLUP.
     * @param debounceMs Tempo estavel para aceitar pressionar/soltar; padrao 30 ms.
     * @return true quando cadastrado; false para ID/pino invalido ou duplicado,
     * conflito de uso do GPIO ou limite de 12 botoes.
     */
    bool adicionarBotao(uint8_t id, uint8_t pino, uint16_t debounceMs = 30);

    /**
     * @brief Cadastra uma tecla de um conjunto resistivo em um unico GPIO ADC.
     * @param id ID unico de 0 a 254 usado nas consultas.
     * @param pino GPIO ADC do ESP32-S3, de 1 a 20; varias teclas podem compartilhar este pino.
     * @param valorMinimo Inicio inclusivo da faixa ADC de 12 bits, de 0 a 4095.
     * @param valorMaximo Fim inclusivo da faixa ADC de 12 bits, de 0 a 4095.
     * @param debounceMs Tempo estavel para aceitar uma transicao; padrao 30 ms.
     * @return true quando cadastrado; false para ID/faixa invalida ou sobreposta,
     * conflito de uso do GPIO ou limite de 12 botoes.
     */
    bool adicionarBotaoAnalogico(uint8_t id, uint8_t pino, uint16_t valorMinimo,
                                 uint16_t valorMaximo, uint16_t debounceMs = 30);

    /** @brief Atualiza o debounce de todos os botoes; atualizar() ja chama este metodo. */
    void atualizarBotoes();

    /**
     * @brief Consulta todos os dados de um botao sem consumir eventos.
     * @param id ID informado no cadastro.
     * @return Estado, ADC, tempos e eventos; valido=false para ID inexistente.
     */
    LeituraBotao lerBotao(uint8_t id);

    /** @brief Consulta o estado estavel. @param id ID cadastrado. @return true enquanto pressionado. */
    bool botaoPressionado(uint8_t id);

    /**
     * @brief Consome o evento de pressionamento.
     * @param id ID cadastrado.
     * @return true uma unica vez para cada transicao filtrada de solto para pressionado.
     */
    bool botaoFoiApertado(uint8_t id);

    /**
     * @brief Consome o evento de soltura.
     * @param id ID cadastrado.
     * @return true uma unica vez para cada transicao filtrada de pressionado para solto.
     */
    bool botaoFoiSolto(uint8_t id);

    /**
     * @brief Testa a duracao do pressionamento atual.
     * @param id ID cadastrado. @param tempoMs Duracao minima em milissegundos.
     * @return true enquanto o botao permanecer pressionado por pelo menos tempoMs.
     */
    bool botaoPressionadoPor(uint8_t id, uint32_t tempoMs);

    /** @brief Verifica um cadastro. @param id ID consultado. @return true quando ele existe. */
    bool botaoDisponivel(uint8_t id) const;

    /** @brief Exclui um cadastro. @param id ID cadastrado. @return true quando removido. */
    bool removerBotao(uint8_t id);

    /** @brief Remove todos os cadastros e eventos de botoes. */
    void limparBotoes();

    /**
     * @brief Atribui a MPU6050 a um dos dois barramentos I2C antes de begin().
     * @param barramento BARRAMENTO_I2C0 (Wire) ou BARRAMENTO_I2C1 (Wire1).
     * @note A MPU6050 (0x68) pode compartilhar o barramento com um TCS34725 (0x29).
     */
    void definirBarramentoIMU(BarramentoI2C barramento);

    /**
     * @brief Define o endereco I2C da MPU6050 antes de begin().
     * @param address Endereco de 7 bits, normalmente 0x68.
     */
    void configurarIMU(uint8_t address)
    {
        config_.board.mpuAddress = address;
    }

    /**
     * @brief Comanda os dois motores independentemente.
     * @param velocidadeEsquerda Comando esquerdo de -100 a 100.
     * @param velocidadeDireita Comando direito de -100 a 100.
     */
    void mover(int velocidadeEsquerda, int velocidadeDireita);

    /** @brief Move para frente. @param velocidade Intensidade de 0 a 100. */
    void frente(int velocidade);

    /** @brief Move para tras. @param velocidade Intensidade de 0 a 100. */
    void tras(int velocidade);

    /** @brief Gira no proprio eixo para a esquerda. @param velocidade Intensidade de 0 a 100. */
    void girarEsquerda(int velocidade);

    /** @brief Gira no proprio eixo para a direita. @param velocidade Intensidade de 0 a 100. */
    void girarDireita(int velocidade);

    /**
     * @brief Gira no proprio eixo ate atingir um angulo medido pela MPU6050.
     * @param velocidade Intensidade positiva de 1 a 100.
     * @param anguloZ Angulo relativo em graus; positivo gira para a direita e
     * negativo gira para a esquerda.
     * @param timeoutMs Tempo maximo da manobra em milissegundos.
     * @return true quando o angulo foi atingido; false sem motores, sem IMU ou
     * por timeout.
     * @note A chamada e bloqueante e deixa os motores parados ao terminar.
     */
    bool girarAngulo(int velocidade, float anguloZ, uint32_t timeoutMs = 5000);

    /**
     * @brief Inverte o sinal medido do eixo Z usado por girarAngulo().
     * @param invertido true quando a orientacao fisica da IMU troca o sinal Z.
     * @note Configure antes de chamar girarAngulo(); o comando continua usando
     * angulo positivo para direita e negativo para esquerda.
     */
    void inverterAnguloZ(bool invertido)
    {
        anguloZInvertido_ = invertido;
    }

    /** @brief Atualiza apenas o motor esquerdo. @param velocidade Comando de -100 a 100. */
    void motorEsquerdo(int velocidade);

    /** @brief Atualiza apenas o motor direito. @param velocidade Comando de -100 a 100. */
    void motorDireito(int velocidade);

    /** @brief Para em modo livre, desligando os enables da ponte H. */
    void parar();

    /** @brief Aplica frenagem ativa aos dois motores. */
    void frear();

    /**
     * @brief Configura a inversao esquerda.
     * @param invertido true para inverter.
     * @note Um valor definido antes de begin() tem prioridade sobre a NVS.
     */
    void inverterMotorEsquerdo(bool invertido);

    /**
     * @brief Configura a inversao direita.
     * @param invertido true para inverter.
     * @note Um valor definido antes de begin() tem prioridade sobre a NVS.
     */
    void inverterMotorDireito(bool invertido);

    /**
     * @brief Configura a inversao dos dois motores em uma chamada.
     * @param esquerdo true para inverter o esquerdo.
     * @param direito true para inverter o direito.
     */
    void inverterMotores(bool esquerdo, bool direito);

    /**
     * @brief Troca os canais fisicos dos motores esquerdo e direito.
     * @param trocados true quando o motor esquerdo esta no conector direito e vice-versa.
     * @note Para os motores e pode ser chamado antes ou depois de begin(). Nao inverte a polaridade;
     * para isso, use inverterMotores().
     */
    void trocarMotores(bool trocados = true);

    /**
     * @brief Aplica um fator multiplicador a um motor e sentido.
     * @param motor MOTOR_ESQUERDO ou MOTOR_DIREITO.
     * @param sentido FRENTE ou TRAS. @param fator Multiplicador; 1.0 e neutro.
     * @note Um valor definido antes de begin() tem prioridade sobre a NVS.
     */
    void configurarCompensacaoMotor(Motor motor, SentidoMotor sentido, float fator);

    /**
     * @brief Define o menor comando capaz de movimentar um motor.
     * @param motor MOTOR_ESQUERDO ou MOTOR_DIREITO.
     * @param sentido FRENTE ou TRAS. @param minimo Valor de 0 a 100.
     * @note Um valor definido antes de begin() tem prioridade sobre a NVS.
     */
    void configurarMinimoMovimento(Motor motor, SentidoMotor sentido, uint8_t minimo);

    /**
     * @brief Faz uma aquisicao QTR completa e deriva todos os dados de linha.
     * @return LeituraLinha com bruto[8], calibrado[8], mascara, caso P/B, texto
     * padrao, quantidade de pretos, posicao 0...7000 e tempos.
     * @note Esta chamada tambem atualiza o historico de mudancas de padrao.
     */
    LeituraLinha lerLinha();

    /**
     * @brief Faz uma aquisicao fisica sem aplicar a calibracao.
     * @param output Vetor de oito uint16_t preenchido na ordem logica S1...S8.
     */
    void lerLinhaBruta(uint16_t output[8]);

    /**
     * @brief Faz uma aquisicao e normaliza cada QTR para 0...1000.
     * @param output Vetor de oito uint16_t preenchido na ordem logica S1...S8.
     */
    void lerLinhaCalibrada(uint16_t output[8]);

    /**
     * @brief Faz uma leitura e retorna a mascara binaria dos sensores pretos.
     * @return Mascara em que S1 logico e bit 7, S8 logico e bit 0 e preto fisico vale 1.
     */
    uint8_t lerLinhaBinaria();

    /**
     * @brief Faz uma leitura e retorna o nome P/B correspondente.
     * @return Um dos 256 valores de PadraoLinha, como BBBPPBBB.
     */
    PadraoLinha lerPadraoLinha();

    /** @return Referencia para a ultima LeituraLinha, sem nova aquisicao. */
    const LeituraLinha& ultimaLeituraLinha() const;

    /** @return Milissegundos desde a ultima linha detectada. */
    uint32_t tempoSemLinha() const;

    /**
     * @brief Define quantas mudancas de padrao o historico conserva.
     * @param tamanho Valor de 0 a MAX_HISTORICO_LINHA (32); zero desativa.
     * @note A alteracao limpa o historico. O tamanho inicial e 8.
     */
    void configurarHistoricoLinha(uint8_t tamanho);

    /** @brief Apaga todos os casos registrados no historico de linha. */
    void limparHistoricoLinha();

    /** @return Capacidade configurada, de 0 a 32 casos. */
    uint8_t capacidadeHistoricoLinha() const;

    /** @return Quantidade de casos validos atualmente no historico. */
    uint8_t quantidadeHistoricoLinha() const;

    /**
     * @brief Copia somente os ultimos padroes, do mais novo ao mais antigo.
     * @param saida Vetor PadraoLinha fornecido pelo usuario.
     * @param maximo Quantidade disponivel no vetor saida.
     * @return Quantidade efetivamente copiada.
     */
    uint8_t copiarUltimosCasosLinha(PadraoLinha saida[], uint8_t maximo) const;

    /**
     * @brief Copia os ultimos padroes com inicio e duracao em milissegundos.
     * @param saida Vetor CasoLinha fornecido pelo usuario.
     * @param maximo Quantidade disponivel no vetor saida.
     * @return Quantidade efetivamente copiada, do mais novo ao mais antigo.
     */
    uint8_t copiarHistoricoLinha(CasoLinha saida[], uint8_t maximo) const;

    /**
     * @brief Converte um PadraoLinha para texto P/B terminado em zero.
     * @param padrao Valor binario nomeado a converter.
     * @param saida Buffer com pelo menos nove caracteres.
     */
    static void padraoLinhaTexto(PadraoLinha padrao, char saida[9]);

    /**
     * @brief Calibra o QTR durante um intervalo bloqueante.
     * @param durationMs Duracao da coleta em milissegundos.
     * @return true quando os oito canais produzem faixas validas; em caso de
     * falha, a calibracao anterior permanece aplicada.
     */
    bool calibrarLinha(uint32_t durationMs = 5000);

    /**
     * @brief Calibra o QTR de forma bloqueante.
     * @param samples Quantidade de amostras.
     * @return true quando os oito canais produzem faixas validas; em caso de
     * falha, a calibracao anterior permanece aplicada.
     */
    bool calibrarLinhaAmostras(uint16_t samples);

    /** @brief Define o limiar binario. @param threshold Valor calibrado de 0 a 1000. */
    void configurarLimiarLinha(uint16_t threshold);

    /**
     * @brief Calcula o erro da ultima leitura de linha.
     * @param center Posicao desejada; 3500 representa o centro geometrico.
     * @return ultima posicao valida menos center.
     */
    int16_t erroLinha(int16_t center = 3500) const;

    /**
     * @brief Configura os ganhos do seguidor de linha.
     * @param kp Ganho proporcional. @param ki Ganho integral. @param kd Ganho derivativo.
     */
    void configurarPID(float kp, float ki, float kd);

    /** @brief Limpa as memorias integral, derivativa e temporal do PID. */
    void resetPID();

    /** @brief Calcula a correcao PID. @param error Erro de entrada. @return Correcao limitada. */
    float calcularPID(float error);

    /**
     * @brief Le a linha, calcula PID e comanda os motores automaticamente.
     * @param velocidadeBase Velocidade nominal de 0 a 100.
     * @note Para os motores quando a leitura nao possui posicao valida.
     */
    void seguirLinha(int velocidadeBase);

    /**
     * @brief Le qualquer sensor de distancia pela mesma interface.
     * @param sensor Fonte desejada; usa ULTRA1 por padrao.
     * @return Medida em cm e mm, validade, timeout, tipo, fonte e timestamp.
     */
    LeituraDistancia lerDistancia(SensorDistancia sensor = DISTANCIA_ULTRASSOM_1);

    /**
     * @brief Le o sensor cadastrado com o ID escolhido pela equipe.
     * @param id Identificador informado no cadastro.
     * @return Medida comum; valida e false quando o ID nao existe.
     */
    LeituraDistancia lerDistancia(uint8_t id);

    /** @return Distancia do ULTRA1 em centimetros ou NAN se a medida falhar. */
    float distanciaCM();

    /**
     * @brief Le uma fonte especifica.
     * @param sensor Slot fisico desejado.
     * @return Centimetros ou NAN quando invalida.
     */
    float distanciaCM(SensorDistancia sensor);

    /**
     * @brief Le pelo ID cadastrado.
     * @param id Identificador escolhido pela equipe.
     * @return Centimetros ou NAN quando invalida.
     */
    float distanciaCM(uint8_t id);

    /**
     * @brief Verifica a presenca de um obstaculo.
     * @param maximumCm Distancia limite em centimetros.
     * @return true para uma medida valida menor ou igual ao limite.
     */
    bool temObstaculo(float maximumCm);

    /**
     * @brief Verifica obstaculo usando uma fonte especifica.
     * @param sensor Slot fisico desejado.
     * @param maximumCm Distancia limite em centimetros.
     * @return true somente para leitura valida menor ou igual ao limite.
     */
    bool temObstaculo(SensorDistancia sensor, float maximumCm);

    /**
     * @brief Verifica obstaculo usando o ID cadastrado pela equipe.
     * @param id Identificador escolhido pela equipe.
     * @param maximumCm Distancia limite em centimetros.
     * @return true somente para leitura valida dentro do limite.
     */
    bool temObstaculo(uint8_t id, float maximumCm);

    /**
     * @brief Faz uma leitura completa de um TCS34725.
     * @param sensor COR_ESQUERDA ou COR_DIREITA.
     * @return RGB, HSV, HSL, canais crus, classificacao, validade e instante.
     */
    LeituraCor lerDadosCor(SensorCor sensor);

    /** @brief Le um sensor. @param sensor Lado desejado. @return RGB normalizado em 0...255. */
    RGB lerRGB(SensorCor sensor);

    /** @brief Le um sensor. @param sensor Lado desejado. @return HSV em graus e porcentagem. */
    HSV lerHSV(SensorCor sensor);

    /** @brief Le um sensor. @param sensor Lado desejado. @return HSL em graus e porcentagem. */
    HSL lerHSL(SensorCor sensor);

    /**
     * @brief Le e classifica um sensor.
     * @param sensor COR_ESQUERDA ou COR_DIREITA.
     * @return CorDetectada ou COR_DESCONHECIDA.
     */
    CorDetectada lerCor(SensorCor sensor);

    /**
     * @brief Altera apenas os limites de hue de uma cor.
     * @param sensor Sensor a configurar. @param color Classe a configurar.
     * @param hMin Menor hue em graus. @param hMax Maior hue em graus.
     * @note hMin maior que hMax representa uma faixa que cruza 360 graus.
     */
    void definirFaixaH(SensorCor sensor, CorDetectada color, float hMin, float hMax);

    /**
     * @brief Define manualmente os limites HSV usados na classificacao.
     * @param sensor Sensor a configurar. @param color Classe a configurar.
     * @param hMin Hue minimo. @param hMax Hue maximo.
     * @param sMin Saturacao minima. @param sMax Saturacao maxima.
     * @param vMin Valor minimo. @param vMax Valor maximo.
     */
    void definirFaixaHSV(SensorCor sensor, CorDetectada color, float hMin, float hMax, float sMin,
                         float sMax, float vMin, float vMax);

    /**
     * @brief Mede amostras e cria automaticamente uma faixa HSV.
     * @param sensor Sensor usado na coleta. @param color Classe associada.
     * @param samples Quantidade de amostras; padrao 30.
     */
    void calibrarCor(SensorCor sensor, CorDetectada color, uint16_t samples = 30);

    /** @brief Configura o ganho de um TCS34725. @param sensor Lado. @param gain Ganho desejado. */
    void setGanhoCor(SensorCor sensor, GanhoCor gain);

    /** @brief Configura a integracao de um TCS34725. @param sensor Lado. @param time Tempo desejado. */
    void setTempoIntegracaoCor(SensorCor sensor, TempoIntegracaoCor time);

    /**
     * @brief Troca a identificacao logica dos sensores de cor esquerdo e direito.
     * @param trocados true quando os dois TCS34725 foram conectados em lados trocados.
     * @return true quando a configuracao foi aceita; false se begin() ja foi chamado.
     * @note Chame antes de definir barramentos, habilitar sensores ou executar begin().
     */
    bool trocarSensoresCor(bool trocados = true);

    /** @return Nova leitura com aceleracao, giro, temperatura e angulos. */
    LeituraIMU lerIMU();

    /** @brief Atualiza somente a estimativa interna da IMU. */
    void atualizarIMU();

    /** @return Inclinacao lateral da ultima leitura em graus. */
    float roll() const;

    /** @return Inclinacao longitudinal da ultima leitura em graus. */
    float pitch() const;

    /** @return Yaw/Z relativo em graus; nao representa rumo de bussola. */
    float anguloZ() const;

    /** @brief Define o angulo Z atual como zero relativo. */
    void zerarAnguloZ();

    /**
     * @brief Testa a inclinacao atual.
     * @param graus Limite absoluto em graus.
     * @return true se abs(roll) ou abs(pitch) atingiu o limite.
     */
    bool estaInclinado(float graus) const;

    /**
     * @brief Calcula offsets da IMU com o robo imovel durante um tempo bloqueante.
     * @param durationMs Duracao da coleta em milissegundos.
     */
    void calibrarIMU(uint32_t durationMs = 3000);

    /** @return true quando as calibracoes foram gravadas e confirmadas no NVS. */
    bool salvarCalibracao();

    /** @return true quando uma calibracao valida foi carregada e aplicada. */
    bool carregarCalibracao();

    /**
     * @brief Apaga toda a calibracao salva e restaura os valores padrao em RAM.
     * @return true quando o namespace NVS foi apagado; os valores em RAM sao
     * restaurados mesmo se a operacao de flash falhar.
     * @note Restaura QTR, limiar 500, motores, cores e offsets da IMU. Pinagem,
     * troca de lados e cadastro de sensores de distancia nao sao calibracao.
     */
    bool apagarCalibracao();

    /** @return O mesmo resultado de apagarCalibracao(); alias com nome explicito. */
    bool resetarCalibracao()
    {
        return apagarCalibracao();
    }

    /** @return true quando o modulo QTR iniciou corretamente. */
    bool linhaDisponivel() const
    {
        return status_.linha;
    }

    /** @return true quando o modulo ultrassonico foi configurado. */
    bool ultrassomDisponivel() const
    {
        return status_.ultrassom;
    }

    /**
     * @brief Consulta qualquer fonte de distancia.
     * @param sensor Slot fisico desejado.
     * @return true quando a fonte foi habilitada e respondeu ao begin().
     */
    bool distanciaDisponivel(SensorDistancia sensor) const;

    /**
     * @brief Consulta a disponibilidade pelo ID cadastrado.
     * @param id Identificador escolhido pela equipe.
     * @return true quando o sensor respondeu ao begin().
     */
    bool distanciaDisponivel(uint8_t id) const;

    /**
     * @brief Consulta a disponibilidade de um TCS34725.
     * @param sensor COR_ESQUERDA ou COR_DIREITA.
     * @return true quando o sensor selecionado respondeu ao begin().
     */
    bool corDisponivel(SensorCor sensor) const;

    /** @return true quando a MPU6050 respondeu ao begin(). */
    bool imuDisponivel() const
    {
        return status_.imu;
    }

    /** @return Referencia para as flags individuais da ultima inicializacao. */
    const StatusRobo& status() const
    {
        return status_;
    }

    /** @return true quando todos os modulos previstos responderam. */
    bool inicializacaoCompleta() const
    {
        return status_.inicializacaoCompleta();
    }

    /** @return true quando motores e QTR estao disponiveis. */
    bool operacaoBasicaDisponivel() const
    {
        return status_.operacaoBasicaDisponivel();
    }

    /** @brief Imprime o estado dos modulos. @param out Destino Print; Serial por padrao. */
    void printStatus(Print& out = Serial) const;

    /** @brief Imprime as falhas do begin(). @param out Destino Print; Serial por padrao. */
    void printErrosInicializacao(Print& out = Serial) const;

    /** @brief Imprime a ultima linha. @param out Destino Print; Serial por padrao. */
    void printLinha(Print& out = Serial) const;

    /** @brief Le e imprime os dois sensores de cor. @param out Destino Print. */
    void printCores(Print& out = Serial);

    /** @brief Imprime a ultima leitura da IMU. @param out Destino Print. */
    void printIMU(Print& out = Serial) const;

    /** @brief Imprime comandos e calibracoes dos motores. @param out Destino Print. */
    void printMotores(Print& out = Serial) const;

    /**
     * @brief Imprime a origem, os valores aplicados e o estado das alteracoes nao salvas.
     * @param out Destino Print; Serial por padrao.
     * @note begin() chama este metodo automaticamente depois de carregar a NVS.
     */
    void printCalibracao(Print& out = Serial) const;

    /** @brief Define o nivel de diagnostico. @param level Um valor DebugLevel. */
    void setDebugLevel(DebugLevel level)
    {
        debug_.setLevel(level);
    }

  private:
    friend class robos3::SerialCalibration;
    friend class robos3::WebCalibration;
    robos3::ColorSensor& color(SensorCor sensor);
    const robos3::ColorSensor& color(SensorCor sensor) const;
    TwoWire& wire(BarramentoI2C barramento);
    bool i2cDisponivel(BarramentoI2C barramento) const;
    uint8_t indiceCorFisica(SensorCor sensor) const;
    BarramentoI2C barramentoCor(SensorCor sensor) const;
    bool corHabilitada(SensorCor sensor) const;
    robos3::RoboCalibration makeCalibration() const;
    void applyCalibration(const robos3::RoboCalibration& data);

    robos3::RoboConfig config_;
    StatusRobo status_;
    robos3::MotorDriver motors_;
    robos3::LineSensor line_;
    robos3::LinePID pid_;
    robos3::UltrasonicSensor ultrasonic_[2];
    robos3::VL53L0XSensor vl53_[2];
    robos3::ColorSensor colors_[2];
    robos3::IMUSensor imu_;
    robos3::CalibrationStorage storage_;
    robos3::ButtonManager buttons_;
    robos3::RoboDebug debug_;
    using AtualizadorCalibracao = void (*)(void* contexto);
    void* interfacesCalibracao_[2];
    AtualizadorCalibracao atualizadoresCalibracao_[2];
    uint8_t ajustesMotorExplicitos_[2];
    bool begun_;
    bool anguloZInvertido_;
    bool sensoresCorTrocados_;
    bool calibracaoAlterada_;
};
