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
#include "modules/CalibrationStorage.h"
#include "debug/RoboDebug.h"

namespace robos3
{
class SerialCalibration;
class WebCalibration;
class CalibrationManager;
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

    /** @brief Atualiza IMU e interfaces de calibracao Serial/Web habilitadas. */
    void atualizar();

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

    /** @brief Configura a inversao esquerda. @param invertido true para inverter. */
    void inverterMotorEsquerdo(bool invertido);

    /** @brief Configura a inversao direita. @param invertido true para inverter. */
    void inverterMotorDireito(bool invertido);

    /**
     * @brief Configura a inversao dos dois motores em uma chamada.
     * @param esquerdo true para inverter o esquerdo.
     * @param direito true para inverter o direito.
     */
    void inverterMotores(bool esquerdo, bool direito);

    /**
     * @brief Aplica um fator multiplicador a um motor e sentido.
     * @param motor MOTOR_ESQUERDO ou MOTOR_DIREITO.
     * @param sentido FRENTE ou TRAS. @param fator Multiplicador; 1.0 e neutro.
     */
    void configurarCompensacaoMotor(Motor motor, SentidoMotor sentido, float fator);

    /**
     * @brief Define o menor comando capaz de movimentar um motor.
     * @param motor MOTOR_ESQUERDO ou MOTOR_DIREITO.
     * @param sentido FRENTE ou TRAS. @param minimo Valor de 0 a 100.
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
     * @param output Vetor de oito uint16_t preenchido na ordem S1...S8.
     */
    void lerLinhaBruta(uint16_t output[8]);

    /**
     * @brief Faz uma aquisicao e normaliza cada QTR para 0...1000.
     * @param output Vetor de oito uint16_t preenchido na ordem S1...S8.
     */
    void lerLinhaCalibrada(uint16_t output[8]);

    /**
     * @brief Faz uma leitura e retorna a mascara binaria dos sensores pretos.
     * @return Mascara em que S1 e bit 7, S8 e bit 0 e preto fisico vale 1.
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
     */
    void calibrarLinha(uint32_t durationMs = 5000);

    /** @brief Calibra o QTR de forma bloqueante. @param samples Quantidade de amostras. */
    void calibrarLinhaAmostras(uint16_t samples);

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

    /** @return Distancia do HC-SR04 em centimetros ou NAN se a medida falhar. */
    float distanciaCM();

    /**
     * @brief Verifica a presenca de um obstaculo.
     * @param maximumCm Distancia limite em centimetros.
     * @return true para uma medida valida menor ou igual ao limite.
     */
    bool temObstaculo(float maximumCm);

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

    /** @return true quando a calibracao persistida foi apagada do NVS. */
    bool apagarCalibracao();

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
    robos3::RoboCalibration makeCalibration() const;
    void applyCalibration(const robos3::RoboCalibration& data);

    robos3::RoboConfig config_;
    StatusRobo status_;
    robos3::MotorDriver motors_;
    robos3::LineSensor line_;
    robos3::LinePID pid_;
    robos3::UltrasonicSensor ultrasonic_;
    robos3::ColorSensor colors_[2];
    robos3::IMUSensor imu_;
    robos3::CalibrationStorage storage_;
    robos3::RoboDebug debug_;
    bool begun_;
    bool anguloZInvertido_;
#ifdef ROBOS3_CALIBRATION_SERIAL
    robos3::SerialCalibration* serialCalibration_;
#endif
#ifdef ROBOS3_CALIBRATION_WEB
    robos3::WebCalibration* webCalibration_;
#endif
#if defined(ROBOS3_CALIBRATION_WEB) || defined(ROBOS3_CALIBRATION_SERIAL)
    robos3::CalibrationManager* calibrationManager_;
#endif
};
