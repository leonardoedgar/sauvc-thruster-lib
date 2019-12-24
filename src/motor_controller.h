# ifndef SAUVC2020_MOTOR_CONTROLLER_H
# define SAUVC2020_MOTOR_CONTROLLER_H
# include "motor_driver.h"
# include <vector>
# include <map>
# include <string>

class MotorController {
private:
    std::map <int, MotorDriver> motor_pin_to_instance_mapping;
    std::map <std::string, std::map<int, int>> motion_to_motor_mapping;

    /**
     * A function to register motors needed with its respective pins.
     * @param motor_pins_to_register {std::vector<int>} represents motors' pins to register
     * @return {bool} indicates whether the registration was successful or not
     */
    bool register_motor(std::vector<int> motor_pins_to_register);

    /**
     * A function to store the mapping of a motion type to motor to be run.
     * @return {bool} indicates whether the storing was successful or not
     */
    bool store_motion_to_motor_mapping();

    /**
    * A function to load pre-defined motors' motion.
    * @return {map} indicates the mapping of motion to motors
    */
    std::map<std::string, std::map<int, int>> load_motion_to_motor_config();

public:
    /**
    * A function to setup the motor controller
    * @return {bool} indicates whether the setup was successful or not
    */
    bool setup();

    /**
     * A function to move the robot.
     * @param motion {string} indicates the motion to for the robot to produce
     * @return {bool} indicates whether the execution of movement was successful or not
     */
    bool move(std::string motion);

    /**
     * A function to stop the robot from moving.
     * @return {bool} indicates whether the stopping was successful or not.
     */
    bool stop();
};

#endif //SAUVC2020_MOTOR_CONTROLLER_H
