#include <RoboS3.h>

RoboS3 robo;

// A equipe escolhe IDs curtos e com significado no seu proprio codigo.
const uint8_t SENSOR_FRENTE = 10;
const uint8_t SENSOR_TRAS = 11;
const uint8_t SENSOR_LATERAL = 20;

void setup()
{
    Serial.begin(115200);

    // Cadastra os dois ultrassons existentes na PCB com IDs escolhidos.
    robo.adicionarSensorDistancia(SENSOR_FRENTE, SENSOR_ULTRASSONICO, 17, 18);
    robo.adicionarSensorDistancia(SENSOR_TRAS, SENSOR_ULTRASSONICO, 16, 42);

    // O VL53L0X e o TCS34725 usam o mesmo endereco I2C (0x29). Para colocar
    // um VL53 no I2C1, desativamos o sensor de cor que ocupava esse barramento.
    robo.habilitarSensorCor(COR_DIREITA, false);
    robo.adicionarSensorDistancia(SENSOR_LATERAL, SENSOR_TOF_VL53L0X,
                                  BARRAMENTO_I2C1, 100);

    // Para cadastrar outro VL53 no I2C0, desative tambem COR_ESQUERDA.
    // Atencao: na pinagem provisoria atual, o SCL do I2C0 usa GPIO42, que ja
    // e o ECHO do segundo ultrassom. Remapeie I2C0 ou desative esse ultrassom.

    robo.begin();
}

void loop()
{
    // A mesma estrutura serve para qualquer tecnologia de sensor.
    LeituraDistancia frente = robo.lerDistancia(SENSOR_FRENTE);
    LeituraDistancia lateral = robo.lerDistancia(SENSOR_LATERAL);

    if (frente.valida)
    {
        Serial.print("Frente: ");
        Serial.print(frente.centimetros);
        Serial.println(" cm");
    }

    if (lateral.valida)
    {
        Serial.print("Lateral: ");
        Serial.print(lateral.milimetros);
        Serial.println(" mm");
    }
    else if (lateral.timeout)
    {
        Serial.println("VL53 lateral excedeu o timeout");
    }

    // O teste de obstaculo tambem usa apenas o ID, sem depender do tipo.
    if (robo.temObstaculo(SENSOR_FRENTE, 15.0f))
    {
        robo.parar();
    }

    delay(100);
}
