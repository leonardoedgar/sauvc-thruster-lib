# include "gtest/gtest.h"
# define private public
# include "motor_controller.h"
# include <vector>
# include <string>
# include <map>
class MotorControllerTest:public::testing::Test {
protected:
    std::vector<int> motor_pins_to_register {1, 2, 3, 4, 5, 6, 7, 8};
    MotorController sample_controller {motor_pins_to_register};
};

/**
 * Test that MotorController registers motors with correct id according to the number of motors needed to be registered.
 */
TEST_F(MotorControllerTest, motor_registration) {
    ASSERT_EQ(int(sample_controller.motors.size()), motor_pins_to_register.size());
    for (auto &registered_motor: sample_controller.motors) {
        ASSERT_EQ(registered_motor.first, registered_motor.second.get_motor_pin());
    }
}

/**
 * Test that MotorController creates a correct map based on the specified motion type and motors needed to drive specified.
 */
TEST_F(MotorControllerTest, mapping_of_motion_to_motors) {
    std::vector<int> motor_pins_to_run_on_forward {1, 2, 3, 4};
    ASSERT_EQ(sample_controller.motor_pins_for_motion["forward"]["positive"], motor_pins_to_run_on_forward);
}

/**
 * Test that MotorController is able to drive required motors with the specified speed in forward motion.
 */
TEST_F(MotorControllerTest, move_forward_with_desired_motors_and_speed) {
    double test_speed_percentage {60};
    int min_esc_input = 1500, max_esc_input = 1700;
    double esc_input {min_esc_input + (max_esc_input-min_esc_input)*test_speed_percentage/100};
    std::map<std::string, std::vector<int>> motor_pins_for_motion_forward = sample_controller.motor_pins_for_motion["forward"];
    std::string desired_output = "";
    for (const auto &[direction, motor_pins_to_run]: motor_pins_for_motion_forward) {
        for (int motor_pin_to_run: motor_pins_to_run) {
            desired_output += "Motor with pin: " + std::to_string(motor_pin_to_run) + " is running with ESC input: " +
                              std::to_string(esc_input) + ".\n";
        }
    }
    testing::internal::CaptureStdout();
    sample_controller.move("forward", test_speed_percentage);
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that MotorController is able to load all predefined motion configuration.
 */
TEST_F(MotorControllerTest, load_motion_config) {
    ASSERT_EQ(typeid(sample_controller.load_motion_config()),
            typeid(std::map<std::string, std::map<std::string, std::vector<int>>>));
}