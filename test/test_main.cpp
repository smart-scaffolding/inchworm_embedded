
#include <Arduino.h>
#include <unity.h>

#include "JointMotor2.h"
#include "pins.h"

void test_motor_init() {
    JointMotor2 JointMotor = JointMotor2(0, 1, 2, 0xee, 20, 0.3, 20, 30, 0.35, 20, 27.81, true, 0);

    TEST_ASSERT_EQUAL(JointMotor.get_pwmForwardPin(), (0));
    TEST_ASSERT_EQUAL(JointMotor.get_pwmReversePin(), 1);
    TEST_ASSERT_EQUAL(JointMotor.get_pinEnable(), 2);
    TEST_ASSERT_EQUAL(JointMotor.get_encoderAddress(), 0xee);
    TEST_ASSERT_EQUAL(JointMotor.get_kP(), 20);
    TEST_ASSERT_EQUAL(JointMotor.get_kI(), 0.3);
    TEST_ASSERT_EQUAL(JointMotor.get_kD(), 20);
    TEST_ASSERT_EQUAL(JointMotor.get_kP2(), 30);
    TEST_ASSERT_EQUAL(JointMotor.get_kI2(), 0.35);
    TEST_ASSERT_EQUAL(JointMotor.get_kD2(), 20);
    TEST_ASSERT_EQUAL(JointMotor.get_angle_offset(), 27.81);
    TEST_ASSERT_EQUAL(JointMotor.get_id(), 0);
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_motor_init);
}

void loop() {
    UNITY_END();
}