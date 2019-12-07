# include "motor_driver.h"
# include <iostream>
MotorDriver::MotorDriver(int pin): pin{pin}{

}

MotorDriver::MotorDriver() {
    std::cout << "Motor driver without args is called\n.";
    MotorDriver(-1);
}

/**
 * The implementation of a function to retrieve the pin of a motor.
 * @return {int} represents the id of the motor
 */
int MotorDriver::retrieve_motor_pin() {
    return pin;
}

/**
 * The implementation of the function to run a motor at a certain speed.
 * @param speed {double} represents the speed to run the motor
 * @return {bool} indicates whether the motor runs successfully or not
 */
bool MotorDriver::run(double speed) {
    std::cout << std::fixed;
    std::cout << "Motor with pin: " << pin << " is running at speed: " << speed << ".\n";
}