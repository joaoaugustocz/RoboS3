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
     * @note Chame Serial.begin() antes para visualizar o diagnostico padrao.
     */
    bool begin(Print& diagnostico = Serial);

    /** @brief Atualiza IMU e interfaces de calibracao Serial/Web habilitadas. */
    void atualizar();

    /**
     * @brief Substitui os oito GPIOs e o emissor do QTR.
     * @param s1 GPIO do sensor mais a esquerda; s2...s8 seguem a ordem fisica.
     * @param emitter GPIO do emissor, ou 0xFF quando nao houver controle.
     * @note Chame antes de begin().
     */
    void definePinsQTR(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint8_t s5, uint8_t s6,
                       uint8_t s7, uint8_t s8, uint8_t emitter = 0xFF);

    /** @brief Seleciona QTR_8A ou QTR_8RC antes de begin(). */
    void setTipoQTR(TipoQTR type)
    {
        config_.board.qtrType = type;
    }

    /** @brief Seleciona se a pista possui linha preta ou linha branca. */
    void setTipoLinha(TipoLinha type);

    /** @brief Substitui a pinagem completa da ponte H antes de begin(). */
    void definePinsPonteH(uint8_t ena, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                          uint8_t enb);

    /** @brief Substitui os GPIOs TRIGGER e ECHO do ultrassom antes de begin(). */
    void definePinsUltrassom(uint8_t trigger, uint8_t echo);

    /** @brief Configura SDA, SCL e frequencia do barramento I2C0 antes de begin(). */
    void definePinsI2C0(uint8_t sda, uint8_t scl, uint32_t frequency = 400000);

    /** @brief Configura SDA, SCL e frequencia do barramento I2C1 antes de begin(). */
    void definePinsI2C1(uint8_t sda, uint8_t scl, uint32_t frequency = 400000);

    /** @brief Define o endereco I2C da MPU6050, normalmente 0x68. */
    void configurarIMU(uint8_t address)
    {
        config_.board.mpuAddress = address;
    }

    /** @brief Comanda os dois motores com valores independentes de -100 a 100. */
    void mover(int velocidadeEsquerda, int velocidadeDireita);

    /** @brief Move os dois motores para frente com velocidade de 0 a 100. */
    void frente(int velocidade);

    /** @brief Move os dois motores para tras com velocidade de 0 a 100. */
    void tras(int velocidade);

    /** @brief Gira no proprio eixo para a esquerda. */
    void girarEsquerda(int velocidade);

    /** @brief Gira no proprio eixo para a direita. */
    void girarDireita(int velocidade);

    /** @brief Atualiza apenas o motor esquerdo e preserva o comando direito. */
    void motorEsquerdo(int velocidade);

    /** @brief Atualiza apenas o motor direito e preserva o comando esquerdo. */
    void motorDireito(int velocidade);

    /** @brief Para em modo livre, desligando os enables da ponte H. */
    void parar();

    /** @brief Aplica frenagem ativa aos dois motores. */
    void frear();

    /** @brief Inverte logicamente a polaridade do motor esquerdo. */
    void inverterMotorEsquerdo(bool invertido);

    /** @brief Inverte logicamente a polaridade do motor direito. */
    void inverterMotorDireito(bool invertido);

    /** @brief Configura a inversao dos dois motores em uma chamada. */
    void inverterMotores(bool esquerdo, bool direito);

    /** @brief Aplica um fator multiplicador a um motor e sentido. */
    void configurarCompensacaoMotor(Motor motor, SentidoMotor sentido, float fator);

    /** @brief Define o menor PWM capaz de movimentar um motor naquele sentido. */
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

    /** @brief Retorna a ultima LeituraLinha sem fazer nova aquisicao fisica. */
    const LeituraLinha& ultimaLeituraLinha() const;

    /** @brief Retorna os milissegundos desde a ultima linha detectada. */
    uint32_t tempoSemLinha() const;

    /**
     * @brief Define quantas mudancas de padrao o historico conserva.
     * @param tamanho Valor de 0 a MAX_HISTORICO_LINHA (32); zero desativa.
     * @note A alteracao limpa o historico. O tamanho inicial e 8.
     */
    void configurarHistoricoLinha(uint8_t tamanho);

    /** @brief Apaga todos os casos registrados no historico de linha. */
    void limparHistoricoLinha();

    /** @brief Retorna a capacidade configurada, de 0 a 32 casos. */
    uint8_t capacidadeHistoricoLinha() const;

    /** @brief Retorna quantos casos validos existem atualmente no historico. */
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

    /** @brief Converte um PadraoLinha para texto P/B terminado em zero. */
    static void padraoLinhaTexto(PadraoLinha padrao, char saida[9]);

    /** @brief Calibra o QTR durante um intervalo bloqueante em milissegundos. */
    void calibrarLinha(uint32_t durationMs = 5000);

    /** @brief Calibra o QTR por uma quantidade bloqueante de amostras. */
    void calibrarLinhaAmostras(uint16_t samples);

    /** @brief Define o limiar binario dos valores calibrados, entre 0 e 1000. */
    void configurarLimiarLinha(uint16_t threshold);

    /** @brief Retorna posicao menos centro; o centro padrao e 3500. */
    int16_t erroLinha(int16_t center = 3500) const;

    /** @brief Configura os ganhos proporcional, integral e derivativo. */
    void configurarPID(float kp, float ki, float kd);

    /** @brief Limpa as memorias integral, derivativa e temporal do PID. */
    void resetPID();

    /** @brief Calcula e limita a correcao PID para o erro informado. */
    float calcularPID(float error);

    /** @brief Le a linha, calcula PID e comanda os motores automaticamente. */
    void seguirLinha(int velocidadeBase);

    /** @brief Mede a distancia do HC-SR04 em centimetros ou retorna NAN. */
    float distanciaCM();

    /** @brief Retorna true para uma medida valida menor ou igual ao limite. */
    bool temObstaculo(float maximumCm);

    /** @brief Retorna RGB, HSV, HSL, dados crus e classificacao de um TCS34725. */
    LeituraCor lerDadosCor(SensorCor sensor);

    /** @brief Retorna somente RGB normalizado na faixa de 0 a 255. */
    RGB lerRGB(SensorCor sensor);

    /** @brief Retorna HSV com H em graus e S/V em porcentagem. */
    HSV lerHSV(SensorCor sensor);

    /** @brief Retorna HSL com H em graus e S/L em porcentagem. */
    HSL lerHSL(SensorCor sensor);

    /** @brief Classifica a leitura conforme as faixas configuradas. */
    CorDetectada lerCor(SensorCor sensor);

    /** @brief Altera apenas os limites de hue de uma cor. */
    void definirFaixaH(SensorCor sensor, CorDetectada color, float hMin, float hMax);

    /** @brief Define manualmente todos os limites HSV usados na classificacao. */
    void definirFaixaHSV(SensorCor sensor, CorDetectada color, float hMin, float hMax, float sMin,
                         float sMax, float vMin, float vMax);

    /** @brief Mede amostras e cria automaticamente uma faixa HSV para a cor. */
    void calibrarCor(SensorCor sensor, CorDetectada color, uint16_t samples = 30);

    /** @brief Configura o ganho 1X, 4X, 16X ou 60X do TCS34725. */
    void setGanhoCor(SensorCor sensor, GanhoCor gain);

    /** @brief Configura o tempo de integracao do TCS34725. */
    void setTempoIntegracaoCor(SensorCor sensor, TempoIntegracaoCor time);

    /** @brief Atualiza e retorna aceleracao, giro, temperatura e angulos da IMU. */
    LeituraIMU lerIMU();

    /** @brief Atualiza somente a estimativa interna da IMU. */
    void atualizarIMU();

    /** @brief Retorna a inclinacao lateral em graus. */
    float roll() const;

    /** @brief Retorna a inclinacao longitudinal em graus. */
    float pitch() const;

    /** @brief Retorna o yaw/Z relativo em graus; nao e uma bussola. */
    float anguloZ() const;

    /** @brief Define o angulo Z atual como zero relativo. */
    void zerarAnguloZ();

    /** @brief Testa se abs(roll) ou abs(pitch) atingiu o limite. */
    bool estaInclinado(float graus) const;

    /** @brief Calcula offsets da IMU com o robo imovel durante um tempo bloqueante. */
    void calibrarIMU(uint32_t durationMs = 3000);

    /** @brief Salva calibracoes de linha, motores, cores e IMU no NVS. */
    bool salvarCalibracao();

    /** @brief Carrega e aplica a calibracao salva; false se ela nao for valida. */
    bool carregarCalibracao();

    /** @brief Apaga a calibracao persistida no NVS. */
    bool apagarCalibracao();

    /** @brief Informa se o modulo QTR iniciou corretamente. */
    bool linhaDisponivel() const
    {
        return status_.linha;
    }

    /** @brief Informa se o modulo ultrassonico foi configurado. */
    bool ultrassomDisponivel() const
    {
        return status_.ultrassom;
    }

    /** @brief Informa se o TCS34725 selecionado respondeu ao begin(). */
    bool corDisponivel(SensorCor sensor) const;

    /** @brief Informa se a MPU6050 respondeu ao begin(). */
    bool imuDisponivel() const
    {
        return status_.imu;
    }

    /** @brief Retorna as flags individuais da ultima inicializacao. */
    const StatusRobo& status() const
    {
        return status_;
    }

    /** @brief Informa se todos os modulos previstos responderam. */
    bool inicializacaoCompleta() const
    {
        return status_.inicializacaoCompleta();
    }

    /** @brief Informa se motores e QTR estao disponiveis. */
    bool operacaoBasicaDisponivel() const
    {
        return status_.operacaoBasicaDisponivel();
    }

    /** @brief Imprime o estado de todos os modulos e da calibracao. */
    void printStatus(Print& out = Serial) const;

    /** @brief Imprime apenas os modulos que falharam durante begin(). */
    void printErrosInicializacao(Print& out = Serial) const;

    /** @brief Imprime bruto, calibrado, mascara, posicao e tempo da linha. */
    void printLinha(Print& out = Serial) const;

    /** @brief Le e imprime os dois sensores de cor. */
    void printCores(Print& out = Serial);

    /** @brief Imprime a ultima leitura mantida pela IMU. */
    void printIMU(Print& out = Serial) const;

    /** @brief Imprime comandos e calibracoes atuais dos motores. */
    void printMotores(Print& out = Serial) const;

    /** @brief Seleciona DEBUG_OFF, DEBUG_ERROR, DEBUG_INFO ou DEBUG_VERBOSE. */
    void setDebugLevel(DebugLevel level)
    {
        debug_.setLevel(level);
    }

  private:
    friend class robos3::SerialCalibration;
    friend class robos3::WebCalibration;
    robos3::ColorSensor& color(SensorCor sensor);
    const robos3::ColorSensor& color(SensorCor sensor) const;
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
