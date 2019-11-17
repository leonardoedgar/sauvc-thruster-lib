#ifndef SAUVC2020_MOTOR_CONTROLLER_H
#define SAUVC2020_MOTOR_CONTROLLER_H
# include "motor_driver.h"
# include <vector>

class MotorController {
private:
    std::vector <MotorDriver> motors {0};
    void RegisterMotor(int no_of_motors);
public:
    MotorController(int no_of_motors);
    bool forward(double speed);
};

#endif //SAUVC2020_MOTOR_CONTROLLER_H
