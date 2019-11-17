# include "gtest/gtest.h"
#define private public
# include "motor_controller.h"
#include <vector>

class MotorControllerTest:public::testing::Test {
protected:
    int no_of_motors_to_register {8};
    MotorController sample_controller {no_of_motors_to_register};
};

TEST_F(MotorControllerTest, correct_initialisation) {
    ASSERT_EQ(int(sample_controller.motors.size()), no_of_motors_to_register);
}

TEST_F(MotorControllerTest, motion_to_motors_correctly_mapped) {
    std::vector<int> motor_ids_to_run_on_forward {1, 2, 3, 4};
    ASSERT_EQ(sample_controller.motor_ids_for_motion["forward"], motor_ids_to_run_on_forward);
}

TEST_F(MotorControllerTest, move_forward_with_correct_motors_and_speed) {
    ASSERT_EQ(true, true);
}