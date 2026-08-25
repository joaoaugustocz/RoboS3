#include <RoboS3.h>
RoboS3 robo;
enum Estado
{
    NORMAL,
    INICIAR_CURVA,
    GIRANDO
};
Estado estado = INICIAR_CURVA;
void curvaBloqueante()
{
    robo.zerarAnguloZ();
    robo.mover(40, -40);
    while (abs(robo.anguloZ()) < 90)
    {
        robo.atualizarIMU();
    }
    robo.frear();
}
void setup()
{
    robo.begin();
}
void loop()
{
    switch (estado)
    {
    case NORMAL:
        break;
    case INICIAR_CURVA:
        robo.zerarAnguloZ();
        robo.mover(40, -40);
        estado = GIRANDO;
        break;
    case GIRANDO:
        robo.atualizarIMU();
        if (abs(robo.anguloZ()) >= 90)
        {
            robo.frear();
            estado = NORMAL;
        }
        break;
    }
}
