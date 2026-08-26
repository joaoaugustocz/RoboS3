#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.definePinsQTR(1, 2, 3, 5, 6, 7, 8, 9);
    robo.definePinsPonteH(14, 15, 11, 12, 13, 21);

    // A MPU pode estar em qualquer I2C. Aqui ela compartilha I2C1 com o TCS direito.
    robo.definirBarramentoIMU(BARRAMENTO_I2C1);

    // Os dois TCS34725 possuem o mesmo endereco 0x29 e devem permanecer separados.
    robo.definirBarramentoCor(COR_ESQUERDA, BARRAMENTO_I2C0);
    robo.definirBarramentoCor(COR_DIREITA, BARRAMENTO_I2C1);

    // Ajuste estes GPIOs para os valores eletricos confirmados da sua placa.
    // Nunca reutilize GPIO de QTR, ultrassom ou ponte H em um barramento I2C.
    robo.begin();
}
void loop()
{
}
