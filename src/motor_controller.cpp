#include "motor_controller.h"
#include "motor_driver.h"
#include <vector>
MotorController::MotorController(int no_of_motors) {
    RegisterMotor(no_of_motors);
}

void MotorController::RegisterMotor(int no_of_motors) {
    for (int id_to_register=1; id_to_register<no_of_motors; id_to_register++) {
        motors.push_back(MotorDriver(id_to_register));
    }
}

bool MotorController::forward(double speed) {
    return true;
}