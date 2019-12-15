# include "motor_driver.h"
# include <iostream>
# include <string>
MotorDriver::MotorDriver(int pin): pin{pin}{

}

MotorDriver::MotorDriver() {
    std::cout << "Motor driver without args is called\n.";
}

/**
 * The implementation of a function to get the pin of a motor.
 * @return {int} represents the pin of the motor
 */
int MotorDriver::get_motor_pin() {
    return pin;
}

/**
 * The implementation of the function to run a motor at a certain speed.
 * @param speed {double} represents the speed_percentage to run the motor
 * @return {bool} indicates whether the motor runs successfully or not
 */
bool MotorDriver::run(double speed_percentage, std::string direction) {
    if (direction == "forward") {
        std::cout << std::fixed;
        std::cout << "Motor with pin: " << pin << " is running with ESC input: " << map_speed_percentage_to_esc_input(speed_percentage) << ".\n";
    }
    else if (direction == "reverse") {
        std::cout << std::fixed;
        std::cout << "Motor with pin: " << pin << " is running with ESC input: " << map_speed_percentage_to_esc_input(-speed_percentage) << ".\n";
    }
    else if (direction == "stop") {
        std::cout << std::fixed;
        std::cout << "Motor with pin: " << pin << " is running with ESC input: " << map_speed_percentage_to_esc_input(0) << ".\n";
    }
    else {
        std::cout << "Motor with pin: " << pin << " is running with unknown direction.\n";
    }
}

/**
 * A function to stop the motor.
 * @return {bool} indicates whether the stopping was successful or not
 */
bool MotorDriver::stop() {
    run(0, "stop");
    return true;
}

/**
 * The implementation of the function to map speed percentage to the esc input
 * @param speed_percentage {double} indicates the speed percentage to run the motor
 * @return {int} represents the esc input
 */
int MotorDriver::map_speed_percentage_to_esc_input(double speed_percentage) {
    int max_esc_input = 1700, min_esc_input = 1500;
    int max_speed_percentage = 100, min_speed_percentage = -100;
    if (speed_percentage > max_speed_percentage) {
        speed_percentage = max_speed_percentage;
    }
    else if (speed_percentage < min_speed_percentage){
        speed_percentage = min_speed_percentage;
    }
    return int(min_esc_input + (max_esc_input-min_esc_input)*speed_percentage/100);
}