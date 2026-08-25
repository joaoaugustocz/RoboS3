#include <RoboS3.h>
RoboS3 robo;
void setup()
{
    Serial.begin(115200);
    robo.begin();
}
void loop()
{
    uint16_t v[8];
    robo.lerLinhaBruta(v);
    for (auto x : v)
    {
        Serial.print(x);
        Serial.print(' ');
    }
    Serial.println();
    delay(50);
}
