# include "motor_controller.h"
# include "motor_driver.h"
# include <string>
# include <vector>
# include <map>
# include <Arduino.h>
# include "./config/config.h"
# include <sauvc2020_msgs/MotionData.h>

/***
 * The implementation of the function to setup the motor controller
 * @return {bool} indicates whether the setup was successful or not
 */
bool MotorController::setup() {
    bool registration_success = this->register_motor(MOTOR_PINS_TO_REGISTER);
    this->store_motion_to_motor_mapping();
    this->store_motor_id_to_pin_mapping();
    delay(1000);
    std::string registration_status {"false"};
    if (registration_success) {
        registration_status = "true";
    }
    else {
        registration_status = "false";
    }
    Serial.println("Motor registration's success : " + String(registration_status.c_str()));
    return registration_success;
}

/**
 * The implementation of the function to register motors to run
 * @param motor_pins_to_register {std::vector<byte>} represents motor's pins to register
 * @return {bool} indicates whether the registration was successful or not
 */
bool MotorController::register_motor(std::vector<byte> motor_pins_to_register) {
    for (byte motor_pin: motor_pins_to_register) {
        this->motor_pin_to_instance_mapping.insert(std::pair<byte, MotorDriver>(motor_pin, MotorDriver(motor_pin)));
    }
    return this->motor_pin_to_instance_mapping.size() == motor_pins_to_register.size();
}

/**
 * The implementation of the function to move the robot
 * @param motion {string} indicates the motion for the robot to produce
 * @return {bool} indicates whether the execution of movement was successful or not
 */
bool MotorController::move(std::string motion) {
    if (this->motion_to_motor_mapping.find(motion) != this->motion_to_motor_mapping.end()) {
        if (this->prev_motion != motion) {
            Serial.println("====================================================");
            Serial.println("Robot executing motion: [" + String(motion.c_str()) + "].");
            for (const auto &[motor_id_to_run, esc_input]: this->motion_to_motor_mapping[motion]) {
                this->motor_pin_to_instance_mapping[this->motor_id_to_pin_mapping[motor_id_to_run]].run(esc_input);
                this->motors_speed[motor_id_to_run] = esc_input;
            }
            Serial.println("====================================================");
            this->prev_motion = motion;
        }
        this->stabilise();
        return true;
    }
    return false;
}

/**
 * The implementation of the function to move the robot.
 * @param map_motor_id_to_speed {std::map<int, int>} indicates motor id and the speed to drive.
 * @return {bool} indicates whether the execution of movement was successful or not
 */
bool MotorController::move(std::map<int, int> map_motor_id_to_speed) {
    if(map_motor_id_to_speed.size() == this->motor_pin_to_instance_mapping.size()) {
        for (const auto &[motor_id, esc_input]: map_motor_id_to_speed) {
            this->motor_pin_to_instance_mapping[this->motor_id_to_pin_mapping[motor_id]].run(esc_input);
            this->motors_speed[motor_id] = esc_input;
        }
        return true;
    }
    return false;
}


/**
 * The implementation of the function to stop the robot from moving.
 * @return {bool} indicates whether the stopping was successful or not.
 */
bool MotorController::stop() {
    Serial.println("====================================================");
    Serial.println("Robot executing motion: [stop].");
    for (const auto &[pin, motor_driver]: this->motor_pin_to_instance_mapping) {
        this->motor_pin_to_instance_mapping[pin].stop();
    }
    Serial.println("====================================================");
    this->prev_motion = "stop";
    return true;
}

/**
 * The implementation of the function to store the mapping of motion type to run
 * @return {bool} indicates whether the storing was successful or not
 */
bool MotorController::store_motion_to_motor_mapping() {
    std::map<std::string, std::map<byte, int>> motion_to_motor_config = this->load_motion_to_motor_config();
    for ( const auto &[motion, motor_with_esc_input]: motion_to_motor_config ) {
        this->motion_to_motor_mapping.insert(std::pair<std::string, std::map<byte, int>>(motion, motor_with_esc_input));
    }
    return this->motion_to_motor_mapping.size() == motion_to_motor_config.size();
}

/**
* A function to store the mapping of motor id to motor pin.
* @return {bool} indicates whether the storing was successful or not
*/
bool MotorController::store_motor_id_to_pin_mapping() {
    for(const auto &[motor_id, motor_pin]: std::map<int, byte> {MOTOR_ID_TO_PIN_MAPPING}) {
        this->motor_id_to_pin_mapping.insert(std::pair<int, byte>(motor_id, motor_pin));
    }
    return this->motor_id_to_pin_mapping.size() == this->motor_pin_to_instance_mapping.size();
}


/**
 * The implementation of the function to load pre-defined motors' motion.
 * @return {map} indicates the mapping of motion to motors
 */
std::map<std::string, std::map<byte, int>> MotorController::load_motion_to_motor_config() {
    std::map<std::string, std::map<byte, int>> motion_to_motor_pins_map{
            {"forward",         {MOTOR_AND_ESC_INPUT_FOR_FORWARD}},
            {"backward",        {MOTOR_AND_ESC_INPUT_FOR_BACKWARD}},
            {"submerge",        {MOTOR_AND_ESC_INPUT_FOR_SUBMERGE}},
            {"surface",         {MOTOR_AND_ESC_INPUT_FOR_SURFACE}},
            {"rotate-left",     {MOTOR_AND_ESC_INPUT_FOR_ROTATE_LEFT}},
            {"rotate-right",    {MOTOR_AND_ESC_INPUT_FOR_ROTATE_RIGHT}},
            {"translate-left",  {MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_LEFT}},
            {"translate-right", {MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_RIGHT}},
            {"roll-left",       {MOTOR_AND_ESC_INPUT_FOR_ROLL_LEFT}},
            {"roll-right",      {MOTOR_AND_ESC_INPUT_FOR_ROLL_RIGHT}},
            {"pitch-forward",   {MOTOR_AND_ESC_INPUT_FOR_PITCH_FORWARD}},
            {"pitch-backward",  {MOTOR_AND_ESC_INPUT_FOR_PITCH_BACKWARD}}
    };
    return motion_to_motor_pins_map;
}

/**
* The implementation of the function to stabilise the robot with respect to the current motion.
* @return {bool} indicates whether the execution of the stabilised motion was successful or not
*/
bool MotorController::stabilise() {
    return this->move(this->motor_id_to_stabilised_speed_mapping);
}

/**
* The implementation of the function to initialise ROS communication with the motor controller.
* @param robot_motion_publisher {ros::Publisher} indicates the publisher of the robot motion
* @param stabilise_speed_subscriber {ros::Subscriber<sauvc2020_msgs::MotorSpeed>} indicates the subscriber for
*        robot stabilised speed
* @return {bool} indicates whether the initialisation was successful or not
*/
bool MotorController::init_ros_communication(ros::Publisher robot_motion_publisher,
        ros::Subscriber<sauvc2020_msgs::MotorSpeed> stabilise_speed_subscriber) {
    this->stabilised_speed_subscriber = stabilise_speed_subscriber;
    this->robot_motion_publisher = robot_motion_publisher;
    return true;
}

/**
 * The implementation of the function to update the motor stabilised speed.
 * @param data {sauvc2020_msgs::MotorSpeed} indicates the stabilised speed data
 * @return {bool} indicates whether the update was successful or not
 */
bool MotorController::update_motor_stabilised_speed(sauvc2020_msgs::MotorSpeed& data) {
    this->motor_id_to_stabilised_speed_mapping[1] = int(data.motor_id1_speed);
    this->motor_id_to_stabilised_speed_mapping[2] = int(data.motor_id2_speed);
    this->motor_id_to_stabilised_speed_mapping[3] = int(data.motor_id3_speed);
    this->motor_id_to_stabilised_speed_mapping[4] = int(data.motor_id4_speed);
    this->motor_id_to_stabilised_speed_mapping[5] = int(data.motor_id5_speed);
    this->motor_id_to_stabilised_speed_mapping[6] = int(data.motor_id6_speed);
    this->motor_id_to_stabilised_speed_mapping[7] = int(data.motor_id7_speed);
    this->motor_id_to_stabilised_speed_mapping[8] = int(data.motor_id8_speed);
    return true;
}

/**
* The implementation of the function to publish the robot motion.
* @return {bool} indicates whether the publish was successful or not
*/
bool MotorController::publish_robot_motion() {
    sauvc2020_msgs::MotionData data;
    data.motion = this->prev_motion;
    data.motors_speed.motor_id1_speed = this->motors_speed[1];
    data.motors_speed.motor_id2_speed = this->motors_speed[2];
    data.motors_speed.motor_id3_speed = this->motors_speed[3];
    data.motors_speed.motor_id4_speed = this->motors_speed[4];
    data.motors_speed.motor_id5_speed = this->motors_speed[5];
    data.motors_speed.motor_id6_speed = this->motors_speed[6];
    data.motors_speed.motor_id7_speed = this->motors_speed[7];
    data.motors_speed.motor_id1_speed = this->motors_speed[8];
    this->robot_motion_publisher.publish(&data);
    return true;
}
