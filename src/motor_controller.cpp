# include "motor_controller.h"
# include "motor_driver.h"
# include <string>
# include <vector>
MotorController::MotorController(std::vector<int> motor_pins) {
    register_motor(motor_pins);
    store_motion_to_motor_mapping();
}

/**
 * The implementation of the function to register motors to run
 * @param motor_pins {std::vector<int>} represents motor's pins to register
 * @return {bool} indicates whether the registration was successful or not
 */
bool MotorController::register_motor(std::vector<int> motor_pins) {
    for (int motor_pin: motor_pins) {
        motors.insert(std::pair<int, MotorDriver>(motor_pin, MotorDriver(motor_pin)));
    }
    return motors.size() == motor_pins.size();
}

/**
 * The implementation of the function to drive the robot forward
 * @param motion {string} indicates the motion for the robot to produce
 * @param speed {double} represents the speed for the robot to move forward
 * @return {bool} indicates whether the execution of moving forward was successful or not
 */
bool MotorController::move(std::string motion, double speed) {
    if (motor_pins_for_motion.find(motion) != motor_pins_for_motion.end()) {
        for (int motor_pin_to_run: motor_pins_for_motion[motion]) {
            motors[motor_pin_to_run].run(speed);
        }
    }
    else {
        return false;
    }
    return true;
}

/**
 * The implementation of the function to store the mapping of motion type to run
 * @return {bool} indicates whether the storing was successful or not
 */
bool MotorController::store_motion_to_motor_mapping() {
    std::vector<int> motor_pins_to_run_on_forward {1, 2, 3, 4};
    motor_pins_for_motion.insert(std::pair<std::string, std::vector<int>>("forward", motor_pins_to_run_on_forward));
}