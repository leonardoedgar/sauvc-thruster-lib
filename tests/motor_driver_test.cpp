# include "gtest/gtest.h"
# define private public
# include "motor_driver.h"
# include "../config/config.h"

class MotorDriverTest:public::testing::Test {
protected:
    int test_pin {MOTOR_NO_3_PIN};
    MotorDriver sample_driver {MOTOR_NO_3_PIN};
};

/**
 * Test that MotorDriver is able to initialise and sets its own id correctly.
 */
TEST_F(MotorDriverTest, correct_id_initialisation) {
    ASSERT_EQ(sample_driver.get_motor_pin(), test_pin);
}

/**
 * Test that MotorDriver is able to run the motor with the desired ESC input.
 */
TEST_F(MotorDriverTest, run_with_correct_speed) {
    int esc_input {1600};
    std::string desired_output = "Motor with pin: " + std::to_string(test_pin) + " is running with ESC input: " +
            std::to_string(esc_input)+ ".\n";
    testing::internal::CaptureStdout();
    sample_driver.run(esc_input);
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that MotorDriver is able to convert ESC value to be within safety range.
 */
TEST_F(MotorDriverTest, get_safe_esc_input_value) {
    ASSERT_EQ(sample_driver.get_safe_esc_input(1600),1600);
    ASSERT_EQ(sample_driver.get_safe_esc_input(1900), MAX_ESC_INPUT);
    ASSERT_EQ(sample_driver.get_safe_esc_input(1100), MIN_ESC_INPUT);
}

/**
 * Test that MotorDriver stops correctly.
 */
TEST_F(MotorDriverTest, stop_motor) {
    std::string desired_output = "Motor with pin: " + std::to_string(test_pin) + " is running with ESC input: " +
            std::to_string(ESC_INPUT_FOR_STOP_SIGNAL) + ".\n";
    testing::internal::CaptureStdout();
    sample_driver.stop();
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}
