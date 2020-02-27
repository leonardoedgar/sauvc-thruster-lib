# ifndef SAUVC2020_MOTOR_CONTROLLER_H
# define SAUVC2020_MOTOR_CONTROLLER_H
# include "motor_driver.h"
# include <vector>
# include <map>
# include <string>
# include <ros.h>
# include <sauvc2020_msgs/MotorSpeed.h>

class MotorController {
private:
    std::map <byte, MotorDriver> motor_pin_to_instance_mapping;
    std::map <std::string, std::map<byte, int>> motion_to_motor_mapping;
    std::map <int, byte> motor_id_to_pin_mapping;
    std::map <int, int> motor_id_to_stabilised_speed_mapping;
    std::string prev_motion = "stop";
    ros::Subscriber<sauvc2020_msgs::MotorSpeed> stabilised_speed_subscriber;
    ros::Publisher robot_motion_publisher;
    std::map<int, int> motors_speed = {{1, 0}, {2, 0},
                                       {3, 0}, {4, 0},
                                       {5, 0}, {6, 0},
                                       {7, 0}, {8, 0}};

    /**
     * A function to register motors needed with its respective pins.
     * @param motor_pins_to_register {std::vector<byte>} represents motors' pins to register
     * @return {bool} indicates whether the registration was successful or not
     */
    bool register_motor(std::vector<byte> motor_pins_to_register);

    /**
     * A function to store the mapping of a motion type to motor to be run.
     * @return {bool} indicates whether the storing was successful or not
     */
    bool store_motion_to_motor_mapping();

    /**
     * A function to store the mapping of motor id to motor pin.
     * @return {bool} indicates whether the storing was successful or not
     */
    bool store_motor_id_to_pin_mapping();

    /**
    * A function to load pre-defined motors' motion.
    * @return {map} indicates the mapping of motion to motors
    */
    std::map<std::string, std::map<byte, int>> load_motion_to_motor_config();

    /**
     * A function to stabilise the robot with respect to the current motion.
     * @return {bool} indicates whether the execution of the stabilised motion was successful or not
     */
    bool stabilise ();

public:
    /**
    * A function to setup the motor controller.
    * @return {bool} indicates whether the setup was successful or not
    */
    bool setup();

    /**
     * A function to initialise ROS communication with the motor controller.
     * @param robot_motion_publisher {ros::Publisher} indicates the publisher of the robot motion
     * @param stabilise_speed_subscriber {ros::Subscriber<sauvc2020_msgs::MotorSpeed>} indicates the subscriber for
     *        robot stabilised speed
     * @return {bool} indicates whether the initialisation was successful or not
     */
    bool init_ros_communication(ros::Publisher robot_motion_publisher,
                                ros::Subscriber<sauvc2020_msgs::MotorSpeed> stabilise_speed_subscriber);

    /**
     * A function to update the motor stabilised speed.
     * @param data {sauvc2020_msgs::MotorSpeed} indicates the stabilised speed data
     * @return {bool} indicates whether the update was successful or not
     */
    bool update_motor_stabilised_speed(sauvc2020_msgs::MotorSpeed& data);

    /**
     * A function to move the robot.
     * @param motion {string} indicates the motion to for the robot to produce
     * @return {bool} indicates whether the execution of movement was successful or not
     */
    bool move(std::string motion);

    /**
     * A function to move the robot.
     * @param map_motor_id_to_speed {std::map<int, int>} indicates motor id and the speed to drive
     * @return {bool} indicates whether the execution of movement was successful or not
     */
    bool move(std::map<int, int> map_motor_id_to_speed);

    /**
     * A function to stop the robot from moving.
     * @return {bool} indicates whether the stopping was successful or not
     */
    bool stop();

    /**
     * A function to publish the robot motion.
     * @return {bool} indicates whether the publish was successful or not
     */
    bool publish_robot_motion();
};

#endif //SAUVC2020_MOTOR_CONTROLLER_H
