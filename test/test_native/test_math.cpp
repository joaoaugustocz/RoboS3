#include <unity.h>
#include <Wire.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "modules/RoboMath.h"

using namespace robos3;

void setUp()
{
}
void tearDown()
{
}

static void test_motor_pipeline()
{
    CalibracaoMotor c{1.0f, 1.0f, 20, 25, false};
    TEST_ASSERT_EQUAL_INT16(100, clampCommand(150));
    TEST_ASSERT_EQUAL_INT16(-100, clampCommand(-150));
    TEST_ASSERT_EQUAL_UINT16(0, motorOutput(0, c, 1023).duty);
    TEST_ASSERT_UINT16_WITHIN(1, 205, motorOutput(1, c, 1023).duty);
    TEST_ASSERT_EQUAL_UINT16(1023, motorOutput(100, c, 1023).duty);
    c.compensacaoFrente = 0.5f;
    TEST_ASSERT_TRUE(motorOutput(100, c, 1023).duty < 600);
    c.invertido = true;
    TEST_ASSERT_FALSE(motorOutput(50, c, 1023).forward);
}

static void test_line_encoding()
{
    uint16_t values[8] = {0, 0, 0, 800, 900, 0, 0, 0};
    const uint8_t mask = lineMask(values, 500, LINHA_PRETA);
    char pattern[9];
    linePattern(mask, pattern);
    TEST_ASSERT_EQUAL_HEX8(0x18, mask);
    TEST_ASSERT_EQUAL_STRING("BBBPPBBB", pattern);
    TEST_ASSERT_EQUAL_UINT8(2, countBits(mask));
    TEST_ASSERT_EQUAL_HEX8(0x18, lineMask(values, 500, LINHA_BRANCA));
    TEST_ASSERT_EQUAL_INT16(8, lineError(3508));
    TEST_ASSERT_EQUAL_UINT32(0, elapsedWithoutDetection(500, 400, true));
    TEST_ASSERT_EQUAL_UINT32(100, elapsedWithoutDetection(500, 400, false));
}

static void test_colors_and_circular_hue()
{
    HSV red = rgbToHsv({255, 0, 0});
    HSL green = rgbToHsl({0, 255, 0});
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, red.h);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, red.s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, green.h);
    TEST_ASSERT_TRUE(hueInRange(359, 345, 15));
    TEST_ASSERT_TRUE(hueInRange(1, 345, 15));
    TEST_ASSERT_FALSE(hueInRange(180, 345, 15));
    FaixaHSV range{345, 15, 30, 100, 20, 100, true};
    TEST_ASSERT_TRUE(hsvInRange({359, 80, 80}, range));
    RGB normalized = normalizeRgb(50, 25, 0, 100);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 127.5f, normalized.r);
}

static int runTests()
{
    UNITY_BEGIN();
    RUN_TEST(test_motor_pipeline);
    RUN_TEST(test_line_encoding);
    RUN_TEST(test_colors_and_circular_hue);
    return UNITY_END();
}

#ifdef ARDUINO
void setup()
{
    delay(100);
    runTests();
}
void loop()
{
}
#else
int main(int, char**)
{
    return runTests();
}
#endif
