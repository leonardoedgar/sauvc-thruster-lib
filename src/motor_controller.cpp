#include "motor_controller.h"
#include "motor_driver.h"
#include <string>
#include <vector>

MotorController::MotorController(int no_of_motors) {
    register_motor(no_of_motors);
    store_motion_to_motor_mapping();
}

/**
 * The implementation of the function to register motors to be run
 * @param no_of_motors {int} represents the number of motors to be registered
 * @return {bool} indicates whether the registration was successful or not
 */
bool MotorController::register_motor(int no_of_motors) {
    for (int id_to_register=1; id_to_register<=no_of_motors; id_to_register++) {
        motors.insert(std::pair<int, MotorDriver>(id_to_register, MotorDriver(id_to_register)));
    }
}

/**
 * The implementation of the function to drive the robot forward
 * @param speed {double} represents the speed for the robot to move forward
 * @return {bool} indicates whether the execution of moving forward was successful or not
 */
bool MotorController::move_forward(double speed) {
    return true;
}

/**
 * The implementation of the function to store the mapping of motion type to run
 * @return {bool} indicates whether the storing was successful or not
 */
bool MotorController::store_motion_to_motor_mapping() {
    std::vector<int> motor_ids_to_run_on_forward {1, 2, 3, 4};
    motor_ids_for_motion.insert(std::pair<std::string, std::vector<int>>("forward", motor_ids_to_run_on_forward));
}