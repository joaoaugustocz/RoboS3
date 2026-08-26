#include <RoboS3.h>

RoboS3 robo;

enum ModoSeguidor
{
    SEGUIR_POR_CASOS,
    SEGUIR_COM_PID
};

// Troque para SEGUIR_POR_CASOS para estudar primeiro os padroes P/B.
const ModoSeguidor MODO_SEGUIDOR = SEGUIR_COM_PID;

// Ative somente quando for medir e salvar novamente verde e vermelho.
// Depois da calibracao, volte para false: begin() carrega os dados salvos.
const bool CALIBRAR_CORES_AO_INICIAR = false;

// Se o sinal do angulo Z ficar contrario na sua montagem, troque para true.
const bool INVERTER_ANGULO_Z = false;

// Troque para false para usar somente o giro simples baseado em tempo.
const bool USAR_IMU_NAS_CURVAS = true;

// Comece com valores baixos e ajuste no robo real.
const int VELOCIDADE_BASE = 45;
const int VELOCIDADE_CURVA = 38;

void calibrarCoresDaPista();
void desviarObstaculo();
bool tratarCasosEspeciais(const LeituraLinha& linha);
void tratarEncruzilhada();
void tratarPossivelCurva90(bool direita);
void seguirPorCasos(const LeituraLinha& linha);
void seguirComPID(const LeituraLinha& linha);
bool virar90(bool direita);
void virar90EsquerdaSemIMU();
void virar90DireitaSemIMU();

void setup()
{
    // A velocidade e escolhida pelo programa. RoboS3::begin() nao chama
    // Serial.begin() e nao impede usar 9600, 115200 ou outro baud rate.
    Serial.begin(115200);
    delay(500);

    // begin() usa a Serial ja iniciada para informar quais modulos falharam.
    robo.begin();

    if (!robo.operacaoBasicaDisponivel())
    {
        Serial.println("Motores ou sensores de linha indisponiveis.");
        while (true)
        {
            delay(1000);
        }
    }

    robo.configurarPID(0.02f, 0.0f, 0.05f);
    robo.inverterAnguloZ(INVERTER_ANGULO_Z);

    if (CALIBRAR_CORES_AO_INICIAR)
    {
        calibrarCoresDaPista();
    }
}

void loop()
{
    // O limite e apenas um ponto de partida. Confirme no robo real.
    if (robo.temObstaculo(12))
    {
        desviarObstaculo();
        return;
    }

    LeituraLinha linha = robo.lerLinha();

    // Gap, encruzilhada e possiveis curvas de 90 graus sao tratados antes.
    // Assim, a mesma logica serve para o seguidor por casos e para o PID.
    if (tratarCasosEspeciais(linha))
    {
        return;
    }

    switch (MODO_SEGUIDOR)
    {
        case SEGUIR_POR_CASOS:
            seguirPorCasos(linha);
            break;

        case SEGUIR_COM_PID:
            seguirComPID(linha);
            break;
    }
}

void calibrarCoresDaPista()
{
    if (!robo.corDisponivel(COR_ESQUERDA) || !robo.corDisponivel(COR_DIREITA))
    {
        Serial.println("Calibracao cancelada: confira os dois sensores de cor.");
        return;
    }

    Serial.println("Coloque VERDE sob o sensor ESQUERDO. Medicao em 3 segundos.");
    delay(3000);
    robo.calibrarCor(COR_ESQUERDA, COR_VERDE, 30);

    Serial.println("Coloque VERDE sob o sensor DIREITO. Medicao em 3 segundos.");
    delay(3000);
    robo.calibrarCor(COR_DIREITA, COR_VERDE, 30);

    Serial.println("Coloque VERMELHO sob o sensor ESQUERDO. Medicao em 3 segundos.");
    delay(3000);
    robo.calibrarCor(COR_ESQUERDA, COR_VERMELHA, 30);

    Serial.println("Coloque VERMELHO sob o sensor DIREITO. Medicao em 3 segundos.");
    delay(3000);
    robo.calibrarCor(COR_DIREITA, COR_VERMELHA, 30);

    if (robo.salvarCalibracao())
    {
        Serial.println("Verde e vermelho salvos na memoria.");
    }
    else
    {
        Serial.println("Falha ao salvar a calibracao.");
    }
}

void desviarObstaculo()
{
    robo.parar();
    delay(150);

    // Esqueleto simples: afasta, contorna pela direita e procura a linha.
    // Distancias e tempos precisam ser medidos no robo da equipe.
    robo.tras(35);
    delay(250);
    if (!virar90(true))
    {
        return;
    }
    robo.frente(35);
    delay(450);
    if (!virar90(false))
    {
        return;
    }

    // Continue a rotina ate reencontrar a linha. Uma solucao real deve usar
    // lerLinha() e o ultrassom, em vez de depender somente de delay().
    robo.parar();
}

bool tratarCasosEspeciais(const LeituraLinha& linha)
{
    if (linha.caso == BBBBBBBB && linha.tempoSemLinhaMs > 120)
    {
        // Possivel gap. Avance por pouco tempo e confirme se a linha reaparece.
        robo.frente(30);
        delay(100);
        robo.parar();
        return true;
    }

    if (linha.caso == PPPPPPPP || linha.quantidadePretos >= 7)
    {
        tratarEncruzilhada();
        return true;
    }

    if (linha.caso == PPBBBBBB || linha.caso == PPPBBBBB)
    {
        tratarPossivelCurva90(false);
        return true;
    }

    if (linha.caso == BBBBBBPP || linha.caso == BBBBBPPP)
    {
        tratarPossivelCurva90(true);
        return true;
    }

    return false;
}

void tratarEncruzilhada()
{
    robo.parar();
    delay(100);

    const bool verdeEsquerda = robo.lerCor(COR_ESQUERDA) == COR_VERDE;
    const bool verdeDireita = robo.lerCor(COR_DIREITA) == COR_VERDE;

    if (verdeEsquerda && verdeDireita)
    {
        // Na regra adotada pela equipe, implemente aqui a meia-volta.
        // Exemplo com IMU: robo.girarAngulo(40, 180);
        robo.parar();
    }
    else if (verdeEsquerda)
    {
        virar90(false);
    }
    else if (verdeDireita)
    {
        virar90(true);
    }
    else
    {
        // Sem verde: atravesse a encruzilhada e continue reto.
        robo.frente(35);
        delay(250);
        robo.parar();
    }
}

void tratarPossivelCurva90(bool direita)
{
    robo.parar();

    // Avanca alem do risco lateral para descobrir se existe linha em frente.
    // Se existir, o desenho provavelmente e um T; se nao, pode ser curva de 90.
    robo.frente(30);
    delay(180);
    robo.parar();
    const LeituraLinha leituraAdiante = robo.lerLinha();
    const bool existeLinhaEmFrente = leituraAdiante.detectada;

    // Volta ao ponto onde os sensores de cor conseguem observar a marcacao.
    robo.tras(30);
    delay(180);
    robo.parar();

    const SensorCor sensorDoLado = direita ? COR_DIREITA : COR_ESQUERDA;
    const bool temVerde = robo.lerCor(sensorDoLado) == COR_VERDE;

    if (existeLinhaEmFrente)
    {
        // Formato T: verde manda entrar no ramo; sem verde, segue reto.
        if (temVerde)
        {
            virar90(direita);
        }
        else
        {
            robo.frente(35);
            delay(250);
            robo.parar();
        }
    }
    else
    {
        // Sem continuidade em frente: trata como curva de 90 graus.
        virar90(direita);
    }
}

void seguirPorCasos(const LeituraLinha& linha)
{
    // Estes casos sao exemplos iniciais. Os valores devem ser ajustados para
    // largura da pista, altura dos sensores e velocidade do robo.
    switch (linha.caso)
    {
        case BBBPPBBB:
            robo.mover(45, 45);
            break;

        case BBBPBBBB:
            robo.mover(35, 45);
            break;

        case BBPPBBBB:
            robo.mover(25, 45);
            break;

        case BPPBBBBB:
        case BPBBBBBB:
            robo.mover(10, 45);
            break;

        case BBBBPBBB:
            robo.mover(45, 35);
            break;

        case BBBBPPBB:
            robo.mover(45, 25);
            break;

        case BBBBBPPB:
        case BBBBBBPB:
            robo.mover(45, 10);
            break;

        default:
            // Um padrao nao listado ainda pode possuir posicao valida.
            if (linha.posicaoValida)
            {
                const int ajuste = ((int)linha.posicao - 3500) / 100;
                robo.mover(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste);
            }
            else
            {
                robo.parar();
            }
            break;
    }
}

void seguirComPID(const LeituraLinha& linha)
{
    if (!linha.posicaoValida)
    {
        robo.parar();
        return;
    }

    // Os casos especiais ja foram removidos por tratarCasosEspeciais().
    // O PID cuida apenas das correcoes normais ao redor da linha.
    const int erro = (int)linha.posicao - 3500;
    const float correcao = robo.calcularPID(erro);
    robo.mover(VELOCIDADE_BASE + correcao, VELOCIDADE_BASE - correcao);
}

bool virar90(bool direita)
{
    if (USAR_IMU_NAS_CURVAS && robo.imuDisponivel())
    {
        const float angulo = direita ? 90.0f : -90.0f;
        if (robo.girarAngulo(VELOCIDADE_CURVA, angulo))
        {
            return true;
        }

        // Um timeout pode significar sinal invertido ou IMU sem calibracao.
        // Nao some outro giro de 90 graus a uma manobra que ja foi iniciada.
        Serial.println("Giro pela IMU incompleto. Robo mantido parado.");
        return false;
    }

    // Alternativa didatica quando a IMU nao sera usada.
    if (direita)
    {
        virar90DireitaSemIMU();
    }
    else
    {
        virar90EsquerdaSemIMU();
    }
    return true;
}

void virar90EsquerdaSemIMU()
{
    robo.girarEsquerda(VELOCIDADE_CURVA);
    delay(430); // Meça este tempo no piso e na bateria usados pela equipe.
    robo.parar();
}

void virar90DireitaSemIMU()
{
    robo.girarDireita(VELOCIDADE_CURVA);
    delay(430); // Meça este tempo no piso e na bateria usados pela equipe.
    robo.parar();
}
