#include "motor_driver.h"
#include <iostream>
MotorDriver::MotorDriver(int id): id{id}{

}

MotorDriver::MotorDriver() {
    std::cout << "Motor driver without args is called\n";
    MotorDriver(-1);
}

/**
 * The implementation of a function to retrieve the id of a motor.
 * @return {int} represents the id of the motor
 */
int MotorDriver::retrieve_motor_id() {
    return id;
}

/**
 * The implementation of the function to run a motor at a certain speed.
 * @param speed {double} represents the speed to run the motor
 * @return {bool} indicates whether the motor is run successfully or not
 */
bool MotorDriver::run(double speed) {
    std::cout << "Motor with id: " << id << " is running at speed: " << std::fixed << speed << "\n";
}