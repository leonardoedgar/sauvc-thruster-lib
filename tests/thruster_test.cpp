# include "gtest/gtest.h"
# define private public
# include "thruster.h"
# include "../config/config.h"

class ThrusterTest:public::testing::Test {
protected:
    int test_pin {THRUSTER_ID_3_PIN};
    int test_id {3};
    Thruster sample_thruster {test_id, test_pin};
};

/**
 * Test that thruster is able to initialise correctly.
 */
TEST_F(ThrusterTest, initialisation) {
    ASSERT_EQ(sample_thruster.get_id(), test_id);
    ASSERT_EQ(sample_thruster.get_pin(), test_pin);
}

/**
 * Test that thruster is able to run with the desired ESC input.
 */
TEST_F(ThrusterTest, run_with_correct_speed) {
    int safe_esc_input {1600};
    std::string desired_output = "Thruster with id: " + std::to_string(test_id) +
            " with pin: " + std::to_string(test_pin) + " is running with ESC input: " +
            std::to_string(safe_esc_input)+ ".\n";
    testing::internal::CaptureStdout();
    sample_thruster.run(safe_esc_input);
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that thruster is able to convert ESC value to be within safety range.
 */
TEST_F(ThrusterTest, get_safe_esc_input_value) {
    ASSERT_EQ(sample_thruster.get_safe_esc_input(1600),1600);
    ASSERT_EQ(sample_thruster.get_safe_esc_input(1900), MAX_ESC_INPUT);
    ASSERT_EQ(sample_thruster.get_safe_esc_input(1100), MIN_ESC_INPUT);
}

/**
 * Test that thruster is able to stop correctly.
 */
TEST_F(ThrusterTest, stop_motor) {
    std::string desired_output = "Thruster with id: " + std::to_string(test_id) +
            " with pin: " + std::to_string(test_pin) + " is running with ESC input: " +
            std::to_string(ESC_INPUT_FOR_STOP_SIGNAL) + ".\n";
    testing::internal::CaptureStdout();
    sample_thruster.stop();
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}
