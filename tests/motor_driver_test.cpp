# include "gtest/gtest.h"
# define private public
# include "motor_driver.h"
class MotorDriverTest:public::testing::Test {
protected:
    int test_pin {3};
    MotorDriver sample_driver {test_pin};
};

/**
 * Test that MotorDriver is able to initialise and sets its own id correctly.
 */
TEST_F(MotorDriverTest, correct_id_initialisation) {
    ASSERT_EQ(sample_driver.get_motor_pin(), test_pin);
}

/**
 * Test that MotorDriver is able to run the motor with the desired speed.
 */
TEST_F(MotorDriverTest, run_with_correct_speed) {
    int min_esc_input = 1500, max_esc_input = 1700;
    double test_speed_percentage {50};
    std::string test_direction {"negative"};
    double esc_input;
    if (test_direction == "positive") {
        esc_input = min_esc_input + (max_esc_input-min_esc_input)*test_speed_percentage/100;
    }
    else if (test_direction == "negative") {
        esc_input = min_esc_input - (max_esc_input-min_esc_input)*test_speed_percentage/100;
    }
    std::string desired_output = "Motor with pin: " + std::to_string(test_pin) + " is running with ESC input: " +
            std::to_string(esc_input)+ ".\n";
    testing::internal::CaptureStdout();
    sample_driver.run(test_speed_percentage, test_direction);
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that MotorDriver maps speed percentage to esc input correctly.
 */
TEST_F(MotorDriverTest, map_speed_percentage_to_esc_input) {
    ASSERT_EQ(sample_driver.map_speed_percentage_to_esc_input(50),1600);
    ASSERT_EQ(sample_driver.map_speed_percentage_to_esc_input(200), sample_driver.map_speed_percentage_to_esc_input(100));
    ASSERT_EQ(sample_driver.map_speed_percentage_to_esc_input(-200), sample_driver.map_speed_percentage_to_esc_input(-100));
}