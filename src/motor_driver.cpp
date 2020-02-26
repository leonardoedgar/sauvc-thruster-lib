# include "motor_driver.h"
# include <iostream>
# include "../config/config.h"
MotorDriver::MotorDriver(int pin): pin{pin}{

}

MotorDriver::MotorDriver() {
    std::cout << "Motor driver without args is called\n.";
}

/**
 * The implementation of the function to get the pin of a motor.
 * @return {int} represents the pin of the motor
 */
int MotorDriver::get_motor_pin() {
    return this->pin;
}

/**
 * The implementation of the function to run a motor at a certain ESC input value.
 * @param esc_input {int} represents the desired ESC input value
 * @return {bool} indicates whether the motor runs successfully or not
 */
bool MotorDriver::run(int esc_input) {
    std::cout << std::fixed;
    std::cout << "Motor with pin: " << this->pin << " is running with ESC input: " << get_safe_esc_input(esc_input) << ".\n";
    return true;
}

/**
 * The implementation of the function to stop the motor.
 * @return {bool} indicates whether the stopping was successful or not
 */
bool MotorDriver::stop() {
    this->run(ESC_INPUT_FOR_STOP_SIGNAL);
    return true;
}

/**
 * The implementation of the function to get a safe esc input value
 * @param esc_input_value {int} indicates the desired esc input
 * @return {int} represents the safe esc input
 */
int MotorDriver::get_safe_esc_input(int esc_input) {
    if (esc_input > MAX_ESC_INPUT) {
        return MAX_ESC_INPUT;
    }
    else if (esc_input < MIN_ESC_INPUT){
        return MIN_ESC_INPUT;
    }
    return esc_input;
}
