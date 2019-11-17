#include "motor_driver.h"
#include <iostream>
MotorDriver::MotorDriver(int id): id{id=id}{

}

int MotorDriver::retrieve_motor_id() {
    return id;
}

void MotorDriver::run(double speed) {
    std::cout << "Motor with id: " << id << " is running at speed: " << std::fixed << speed << "\n";
}