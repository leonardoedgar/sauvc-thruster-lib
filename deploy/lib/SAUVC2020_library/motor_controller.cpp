# include "motor_controller.h"
# include "motor_driver.h"
# include <string>
# include <vector>
# include <map>
# include <Arduino.h>
MotorController::MotorController(std::vector<int> motor_pins) {
    register_motor(motor_pins);
    store_motion_to_motor_mapping();
}

/**
 * The implementation of the function to register motors to run
 * @param motor_pins {std::vector<int>} represents motor's pins to register
 * @return {bool} indicates whether the registration was successful or not
 */
bool MotorController::register_motor(std::vector<int> motor_pins) {
    for (int motor_pin: motor_pins) {
        motors.insert(std::pair<int, MotorDriver>(motor_pin, MotorDriver(motor_pin)));
    }
    return motors.size() == motor_pins.size();
}

/**
 * The implementation of the function to drive the robot forward
 * @param motion {string} indicates the motion for the robot to produce
 * @param speed_percentage {double} represents the speed percentage for the robot to move
 * @return {bool} indicates whether the execution of moving forward was successful or not
 */
bool MotorController::move(std::string motion, double speed_percentage) {
    if (motor_pins_for_motion.find(motion) != motor_pins_for_motion.end()) {
        Serial.println("====================================================");
        String string_to_print = "Robot executing motion: [" + String(motion.c_str()) + "].";
        Serial.println(string_to_print);
        for (const auto &[direction, motor_pins_to_run]: motor_pins_for_motion[motion]) {
            for (int motor_pin_to_run: motor_pins_to_run) {
                motors[motor_pin_to_run].run(speed_percentage, direction);
            }
        }
        Serial.println("====================================================");
    }
    else {
        return false;
    }
    return true;
}

/**
 * The implementation of the function to stop the robot from moving.
 * @return {bool} indicates whether the stopping was successful or not.
 */
bool MotorController::stop() {
    Serial.println("====================================================");
    Serial.println("Robot executing motion: [stop].");
    for (const auto &[pin, motor_driver]: motors) {
        motors[pin].stop();
    }
    Serial.println("====================================================");
    return true;
}

/**
 * The implementation of the function to store the mapping of motion type to run
 * @return {bool} indicates whether the storing was successful or not
 */
bool MotorController::store_motion_to_motor_mapping() {
    std::map<std::string, std::map<std::string, std::vector<int>>> motion_to_motor_config = load_motion_config();
    for ( const auto &[motion, motor_config]: motion_to_motor_config ) {
        motor_pins_for_motion.insert(std::pair<std::string, std::map<std::string, std::vector<int>>>(motion, motor_config));
    }
    return motor_pins_for_motion.size() == motion_to_motor_config.size();
}

/**
 * The implementation of the function to load pre-defined motors' motion.
 * @return {map} indicates the mapping of motion to motors
 */
std::map<std::string, std::map<std::string, std::vector<int>>> MotorController::load_motion_config() {
    std::map<std::string, std::map<std::string, std::vector<int>>> motion_to_motor_pins_map {
            {"forward", {{"positive", {1, 2, 3, 4}}}},
            {"backward", {{"negative", {1, 2, 3, 4}}}},
            {"submerge", {{"positive", {6, 7}}, {"negative", {5, 8}}}},
            {"surface", {{"positive", {5, 8}}, {"negative", {6, 7}}}},
            {"rotate-left", {{"positive", {1, 4}}, {"negative", {2, 3}}}},
            {"rotate-right", {{"positive", {2, 3}}, {"negative", {1, 4}}}},
            {"roll-left", {{"positive", {5, 6}}, {"negative", {7, 8}}}},
            {"roll-right", {{"positive", {7, 8}}, {"negative", {5, 6}}}},
            {"pitch-forward", {{"positive", {5, 7}}, {"negative", {6, 8}}}},
            {"pitch-backward", {{"positive", {6, 8}}, {"negative", {5, 7}}}}
    };
    return motion_to_motor_pins_map;
}