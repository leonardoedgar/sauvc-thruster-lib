# ifndef SAUVC2020_MOTOR_CONTROLLER_H
# define SAUVC2020_MOTOR_CONTROLLER_H
# include "motor_driver.h"
# include <vector>
# include <map>
# include <string>

class MotorController {
private:
    // A hash table to contain all motors objects used as the value with its respective pin as the key.
    std::map <int, MotorDriver> motors;

    // A hash table to map a motion type to motors to run
    std::map <std::string, std::map<std::string, std::vector<int>>> motor_pins_for_motion;

    /**
     * A function to register motors needed with its respective pins.
     * @param motor_pins {std::vector<int>} represents motors' pins to register
     * @return {bool} indicates whether the registration was successful or not
     */
    bool register_motor(std::vector<int> motor_pins);

    /**
     * A function to store the mapping of a motion type to motor to be run.
     * @return {bool} indicates whether the storing was successful or not
     */
    bool store_motion_to_motor_mapping();
    /**
    * A function to load pre-defined motors' motion.
    * @return {map} indicates the mapping of motion to motors
    */
    std::map<std::string, std::map<std::string, std::vector<int>>> load_motion_config();
public:
    MotorController(std::vector<int> motor_pins);

    /**
     * A function to move the robot.
     * @param motion {string} indicates the motion to for the robot to produce
     * @param speed_percentage {double} represents the speed percentage for the robot to move
     * @return {bool} indicates whether the execution of moving forward was successful or not
     */
    bool move(std::string motion, double speed_percentage);
    /**
     * A function to stop the robot from moving.
     * @return {bool} indicates whether the stopping was successful or not.
     */
    bool stop();
};

#endif //SAUVC2020_MOTOR_CONTROLLER_H
