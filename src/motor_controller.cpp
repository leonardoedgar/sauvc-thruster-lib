#include "motor_controller.h"
#include "motor_driver.h"
#include <map>
#include <string>
#include <vector>

MotorController::MotorController(int no_of_motors) {
    register_motor(no_of_motors);
    map_motion_to_motor_to_run();
}

void MotorController::register_motor(int no_of_motors) {
    for (int id_to_register=1; id_to_register<=no_of_motors; id_to_register++) {
        motors.insert(std::pair<int, MotorDriver>(id_to_register, MotorDriver(id_to_register)));
    }
}

bool MotorController::move_forward(double speed) {
    return true;
}

bool MotorController::map_motion_to_motor_to_run() {
    std::vector<int> motor_ids_to_run_on_forward {1, 2, 3, 4};
    motor_ids_for_motion.insert(std::pair<std::string, std::vector<int>>("forward", motor_ids_to_run_on_forward));
}