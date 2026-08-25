#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    uint8_t m = robo.lerLinhaBinaria();
    for (int i = 7; i >= 0; --i)
    {
        Serial.print((m >> i) & 1);
    }
    Serial.println();
    delay(50);
}
