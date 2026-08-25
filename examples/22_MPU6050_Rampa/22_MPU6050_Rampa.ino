#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    LeituraIMU imu = robo.lerIMU();
    Serial.print("Pitch: ");
    Serial.println(imu.pitch);
    if (imu.pitch > 10)
    {
        Serial.println("Possivel subida");
    }
    delay(30);
}
