# include "motor_controller.h"
# include "motor_driver.h"
# include <string>
# include <vector>
# include <map>
# include <iostream>
# include "../config/config.h"
/***
 * The implementation of the function to setup the motor controller
 * @return {bool} indicates whether the setup was successful or not
 */
bool MotorController::setup() {
    bool registration_success = this->register_motor(MOTOR_PINS_TO_REGISTER);
    this->store_motion_to_motor_mapping();
    this->store_motor_id_to_pin_mapping();
    std::string registration_status {"false"};
    if (registration_success) {
        registration_status = "true";
    }
    else {
        registration_status = "false";
    }
    std::cout << "Motor registration's success : "  << registration_status << ".\n";
    return registration_success;
}

/**
 * The implementation of the function to register motors to run
 * @param motor_pins_to_register {std::vector<int>} represents motor's pins to register
 * @return {bool} indicates whether the registration was successful or not
 */
bool MotorController::register_motor(std::vector<int> motor_pins_to_register) {
    for (int motor_pin: motor_pins_to_register) {
        this->motor_pin_to_instance_mapping.insert(std::pair<int, MotorDriver>(motor_pin, MotorDriver(motor_pin)));
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
        for (const auto &[motor_pin_to_run, esc_input]: this->motion_to_motor_mapping[motion]) {
            this->motor_pin_to_instance_mapping[motor_pin_to_run].run(esc_input);
        }
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
    for (const auto &[pin, motor_driver]: this->motor_pin_to_instance_mapping) {
        this->motor_pin_to_instance_mapping[pin].stop();
    }
    return true;
}

/**
 * The implementation of the function to store the mapping of motion type to run
 * @return {bool} indicates whether the storing was successful or not
 */
bool MotorController::store_motion_to_motor_mapping() {
    std::map<std::string, std::map<int, int>> motion_to_motor_config = this->load_motion_to_motor_config();
    for ( const auto &[motion, motor_with_esc_input]: motion_to_motor_config ) {
        this->motion_to_motor_mapping.insert(std::pair<std::string, std::map<int, int>>(motion, motor_with_esc_input));
    }
    return this->motion_to_motor_mapping.size() == motion_to_motor_config.size();
}

/**
* A function to store the mapping of motor id to motor pin.
* @return {bool} indicates whether the storing was successful or not
*/
bool MotorController::store_motor_id_to_pin_mapping() {
    for(const auto &[motor_id, motor_pin]: std::map<int,int> {MOTOR_ID_TO_PIN_MAPPING}) {
        this->motor_id_to_pin_mapping.insert(std::pair<int, int>(motor_id, motor_pin));
    }
    return this->motor_id_to_pin_mapping.size() == this->motor_pin_to_instance_mapping.size();
}

/**
 * The implementation of the function to load pre-defined motors' motion.
 * @return {map} indicates the mapping of motion to motors
 */
std::map<std::string, std::map<int, int>> MotorController::load_motion_to_motor_config() {
    std::map<std::string, std::map<int, int>> motion_to_motor_pins_map {
            {"forward", {MOTOR_AND_ESC_INPUT_FOR_FORWARD}},
            {"backward", {MOTOR_AND_ESC_INPUT_FOR_BACKWARD}},
            {"submerge", {MOTOR_AND_ESC_INPUT_FOR_SUBMERGE}},
            {"surface", {MOTOR_AND_ESC_INPUT_FOR_SURFACE}},
            {"rotate-left", {MOTOR_AND_ESC_INPUT_FOR_ROTATE_LEFT}},
            {"rotate-right", {MOTOR_AND_ESC_INPUT_FOR_ROTATE_RIGHT}},
            {"translate-left", {MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_LEFT}},
            {"translate-right", {MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_RIGHT}},
            {"roll-left", {MOTOR_AND_ESC_INPUT_FOR_ROLL_LEFT}},
            {"roll-right", {MOTOR_AND_ESC_INPUT_FOR_ROLL_RIGHT}},
            {"pitch-forward", {MOTOR_AND_ESC_INPUT_FOR_PITCH_FORWARD}},
            {"pitch-backward", {MOTOR_AND_ESC_INPUT_FOR_PITCH_BACKWARD}}
    };
    return motion_to_motor_pins_map;
}
