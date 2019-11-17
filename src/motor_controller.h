#ifndef SAUVC2020_MOTOR_CONTROLLER_H
#define SAUVC2020_MOTOR_CONTROLLER_H
# include "motor_driver.h"
# include <vector>
# include <map>

class MotorController {
private:
    // A hash table to contain all motors objects used as the value with its respective id as the key.
    std::map <int, MotorDriver> motors;

    // A hash table to map a motion type to motors to run
    std::map <std::string, std::vector<int>> motor_ids_for_motion;

    /**
     * A function to register motors needed with its respective id.
     * @param no_of_motors {int} represents the number of motors to be registered
     * @return {bool} indicates whether the registration was successful or not
     */
    bool register_motor(int no_of_motors);

    /**
     * A function to store the mapping of a motion type to motor to be run.
     * @return {bool} indicates whether the storing was successful or not
     */
    bool store_motion_to_motor_mapping();
public:
    MotorController(int no_of_motors);

    /**
     * A function to drive the robot forward.
     * @param speed {double} represents the speed for the robot to move forward
     * @return {bool} indicates whether the execution of moving forward was successful or not
     */
    bool move_forward(double speed);
};

#endif //SAUVC2020_MOTOR_CONTROLLER_H
