#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    robo.begin();
    robo.configurarMinimoMovimento(MOTOR_ESQUERDO, FRENTE, 18);
    robo.configurarCompensacaoMotor(MOTOR_DIREITO, FRENTE, 0.96f);
    robo.salvarCalibracao();
}
void loop()
{
}
