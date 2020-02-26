# include "gtest/gtest.h"
# define private public
# include "motor_controller.h"
# include <vector>
# include <string>
# include <map>
# include <algorithm>
# include "../config/config.h"
class MotorControllerTest:public::testing::Test {
protected:
    std::vector<int> motor_pins_to_register {MOTOR_PINS_TO_REGISTER};
    MotorController sample_controller;
    virtual void SetUp() {
        sample_controller.setup();
    }
};

/**
 * Test that MotorController registers motors with correct id according to the number of motors needed to be registered.
 */
TEST_F(MotorControllerTest, motor_registration) {
    ASSERT_EQ(int(sample_controller.motor_pin_to_instance_mapping.size()), motor_pins_to_register.size());
    for (auto &registered_motor: sample_controller.motor_pin_to_instance_mapping) {
        ASSERT_EQ(registered_motor.first, registered_motor.second.get_motor_pin());
    }
}

/**
 * Test that MotorController creates a correct map based on the specified motion type and motors needed to drive specified.
 */
TEST_F(MotorControllerTest, mapping_of_motion_to_motors) {
    std::vector<int> motor_pins_to_run_forward {1, 2, 3, 4}, motor_pins_to_stop {5, 6, 7, 8};
    for (const auto &[motor_pin_to_run, esc_input]: sample_controller.motion_to_motor_mapping["forward"]) {
        if (esc_input > ESC_INPUT_FOR_STOP_SIGNAL) {
            ASSERT_TRUE(std::find(motor_pins_to_run_forward.begin(), motor_pins_to_run_forward.end(),
                    motor_pin_to_run) != motor_pins_to_run_forward.end());
        }
        else if (esc_input == ESC_INPUT_FOR_STOP_SIGNAL) {
            ASSERT_TRUE(std::find(motor_pins_to_stop.begin(), motor_pins_to_stop.end(), motor_pin_to_run)
                    != motor_pins_to_stop.end());
        }
    }
}

/**
 * Test that MotorController is able to drive required motors with the specified ESC input in forward motion.
 */
TEST_F(MotorControllerTest, move_forward_with_desired_motors_and_speed) {
    std::map<int, int> motor_pins_for_motion_forward = sample_controller.motion_to_motor_mapping["forward"];
    std::string desired_output = "";
    for (const auto &[motor_pin_to_run, esc_input]: motor_pins_for_motion_forward) {
        if (esc_input > ESC_INPUT_FOR_STOP_SIGNAL) {
            desired_output += "Motor with pin: " + std::to_string(motor_pin_to_run) + " is running with ESC input: " +
                    std::to_string(esc_input) + ".\n";
        }
        else if (esc_input == ESC_INPUT_FOR_STOP_SIGNAL) {
            desired_output += "Motor with pin: " + std::to_string(motor_pin_to_run) + " is running with ESC input: " +
                    std::to_string(ESC_INPUT_FOR_STOP_SIGNAL) + ".\n";;
        }
    }
    testing::internal::CaptureStdout();
    sample_controller.move("forward");
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that MotorController is able to drive correct motors with a customisable speed
 */
 TEST_F(MotorControllerTest, move_with_custom_speed) {
     std::map<int, int> motor_id_to_speed_mapping = {{1, 1500},
                                                     {2, 1600},
                                                     {3, 1700},
                                                     {4, 1800},
                                                     {5, 1850},
                                                     {6, 1750},
                                                     {7, 1650},
                                                     {8, 1600}};
     std::string desired_output = "";
     for (const auto &[motor_id_to_run, esc_input]: motor_id_to_speed_mapping) {
         if (esc_input > MAX_ESC_INPUT) {
             desired_output += "Motor with pin: " + std::to_string(sample_controller.motor_id_to_pin_mapping[motor_id_to_run]) + " is running with ESC input: " +
                               std::to_string(MAX_ESC_INPUT) + ".\n";
         }
         else if (esc_input < MIN_ESC_INPUT) {
             desired_output += "Motor with pin: " + std::to_string(sample_controller.motor_id_to_pin_mapping[motor_id_to_run]) + " is running with ESC input: " +
                               std::to_string(MIN_ESC_INPUT) + ".\n";
         }
         else if (esc_input == ESC_INPUT_FOR_STOP_SIGNAL) {
             desired_output += "Motor with pin: " + std::to_string(sample_controller.motor_id_to_pin_mapping[motor_id_to_run]) + " is running with ESC input: " +
                     std::to_string(ESC_INPUT_FOR_STOP_SIGNAL) + ".\n";
         }
         else {
             desired_output += "Motor with pin: " + std::to_string(sample_controller.motor_id_to_pin_mapping[motor_id_to_run]) + " is running with ESC input: " +
                               std::to_string(esc_input) + ".\n";
         }
     }
     testing::internal::CaptureStdout();
     sample_controller.move(motor_id_to_speed_mapping);
     std::string actual_output = testing::internal::GetCapturedStdout();
     ASSERT_EQ(actual_output, desired_output);
 }

/**
 * Test that MotorController is able to load all predefined motion configuration.
 */
TEST_F(MotorControllerTest, load_motion_config) {
    ASSERT_EQ(typeid(sample_controller.load_motion_to_motor_config()),
            typeid(std::map<std::string, std::map<int, int>>));
}

/**
 * Test that MotorController is able to stop all motors.
 */
TEST_F(MotorControllerTest, stop_all_motors) {
    std::string desired_output = "";
    for (int motor_pin: motor_pins_to_register) {
        desired_output += "Motor with pin: " + std::to_string(motor_pin) + " is running with ESC input: " +
                std::to_string(ESC_INPUT_FOR_STOP_SIGNAL) + ".\n";
    }
    testing::internal::CaptureStdout();
    sample_controller.stop();
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}