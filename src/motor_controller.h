#ifndef SAUVC2020_MOTOR_CONTROLLER_H
#define SAUVC2020_MOTOR_CONTROLLER_H
# include "motor_driver.h"
# include <vector>
#include <map>

class MotorController {
private:
    std::map <int, MotorDriver> motors;
    std::map <std::string, std::vector<int>> motor_ids_for_motion;
    void register_motor(int no_of_motors);
    bool map_motion_to_motor_to_run();
public:
    MotorController(int no_of_motors);
    bool move_forward(double speed);
};

#endif //SAUVC2020_MOTOR_CONTROLLER_H
