# include "gtest/gtest.h"
#define private public
# include "motor_controller.h"

TEST(MotorController, correct_initialisation) {
    int no_of_motors_to_register {8};
    MotorController sample_controller {no_of_motors_to_register};
    ASSERT_EQ(int(sample_controller.motors.size()), no_of_motors_to_register);
}
