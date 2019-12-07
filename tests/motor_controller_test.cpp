# include "gtest/gtest.h"
#define private public
# include "motor_controller.h"
# include <vector>
# include <string>

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
        ASSERT_EQ(registered_motor.first, registered_motor.second.retrieve_motor_pin());
    }
}

/**
 * Test that MotorController creates a correct map based on the specified motion type and motors needed to drive specified.
 */
TEST_F(MotorControllerTest, mapping_of_motion_to_motors) {
    std::vector<int> motor_pins_to_run_on_forward {1, 2, 3, 4};
    ASSERT_EQ(sample_controller.motor_pins_for_motion["forward"], motor_pins_to_run_on_forward);
}

/**
 * Test that MotorController is able to drive required motors with the specified speed in forward motion.
 */
TEST_F(MotorControllerTest, move_forward_with_desired_motors_and_speed) {
    double test_speed {4.5};
    int test_id {1};
    std::vector<int> motor_ids_to_run = sample_controller.motor_pins_for_motion["forward"];
    std::string desired_output = "";
    for (int motor_id_to_run: motor_ids_to_run) {
        desired_output += "Motor with pin: " + std::to_string(motor_id_to_run) + " is running at speed: " +
                std::to_string(test_speed)+ ".\n";
    }
    testing::internal::CaptureStdout();
    sample_controller.move("forward", test_speed);
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}