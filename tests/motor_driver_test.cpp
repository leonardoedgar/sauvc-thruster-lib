#include "gtest/gtest.h"
#include "motor_driver.h"
/**
 * Test MotorDriver is able to initialise and sets its own id correctly.
 */
TEST(MotorDriver, correct_id_initialisation) {
    int test_pin=5;
    MotorDriver sample_motor {test_pin};
    ASSERT_EQ(sample_motor.retrieve_motor_pin(), test_pin);
}

/**
 * Test MotorDriver's function run drives the motor with the desired speed.
 */
TEST(MotorDriver, run_with_correct_speed) {
    int test_pin {1};
    MotorDriver sample_motor {test_pin};
    double test_speed {3.5};
    std::string desired_output = "Motor with pin: " + std::to_string(test_pin) + " is running at speed: " +
            std::to_string(test_speed)+ ".\n";
    testing::internal::CaptureStdout();
    sample_motor.run(test_speed);
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}