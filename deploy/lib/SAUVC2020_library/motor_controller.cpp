# include "motor_controller.h"
# include "motor_driver.h"
# include <string>
# include <vector>
# include <map>
# include <Arduino.h>
/***
 * The implementation of the function to setup the motor controller
 * @param motor_pins {std::vector<byte>} indicates motor pins to register
 * @return {bool} indicates whether the setup was successful or not
 */
bool MotorController::setup(std::vector<byte> motor_pins) {
    bool success_bool = register_motor(motor_pins);
    store_motion_to_motor_mapping();
    delay(1000);
    std::string success_str {"false"};
    if (success_bool) {
        success_str = "true";
    }
    else {
        success_str = "false";
    }
    Serial.println("Motor registration's success : " + String(success_str.c_str()));
    return success_bool;
}

/**
 * The implementation of the function to register motors to run
 * @param motor_pins {std::vector<byte>} represents motor's pins to register
 * @return {bool} indicates whether the registration was successful or not
 */
bool MotorController::register_motor(std::vector<byte> motor_pins) {
    for (byte motor_pin: motor_pins) {
        motors.insert(std::pair<byte, MotorDriver>(motor_pin, MotorDriver(motor_pin)));
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
            for (byte motor_pin_to_run: motor_pins_to_run) {
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
    std::map<std::string, std::map<std::string, std::vector<byte>>> motion_to_motor_config = load_motion_config();
    for ( const auto &[motion, motor_config]: motion_to_motor_config ) {
        motor_pins_for_motion.insert(std::pair<std::string, std::map<std::string, std::vector<byte>>>(motion, motor_config));
    }
    return motor_pins_for_motion.size() == motion_to_motor_config.size();
}

/**
 * The implementation of the function to load pre-defined motors' motion.
 * @return {map} indicates the mapping of motion to motors
 */
std::map<std::string, std::map<std::string, std::vector<byte>>> MotorController::load_motion_config() {
    byte motor_id_1_pin {3}, motor_id_2_pin {4}, motor_id_3_pin {5}, motor_id_4_pin {6}, motor_id_5_pin {7},
            motor_id_6_pin {8}, motor_id_7_pin {9}, motor_id_8_pin {10};
    std::map<std::string, std::map<std::string, std::vector<byte>>> motion_to_motor_pins_map {
            {"forward", {{"positive", {motor_id_1_pin, motor_id_2_pin, motor_id_3_pin, motor_id_4_pin}}}},
            {"backward", {{"negative", {motor_id_1_pin, motor_id_2_pin, motor_id_3_pin, motor_id_4_pin}}}},
            {"submerge", {{"positive", {motor_id_6_pin, motor_id_7_pin}},
                                {"negative", {motor_id_5_pin, motor_id_8_pin}}}},
            {"surface", {{"positive", {motor_id_5_pin, motor_id_8_pin}},
                                {"negative", {motor_id_6_pin, motor_id_7_pin}}}},
            {"rotate-left", {{"positive", {motor_id_1_pin, motor_id_3_pin}},
                                {"negative", {motor_id_2_pin, motor_id_4_pin}}}},
            {"rotate-right", {{"positive", {motor_id_2_pin, motor_id_4_pin}},
                                {"negative", {motor_id_1_pin, motor_id_3_pin}}}},
            {"roll-right", {{"positive", {motor_id_5_pin, motor_id_6_pin}},
                                {"negative", {motor_id_7_pin, motor_id_8_pin}}}},
            {"roll-left", {{"positive", {motor_id_7_pin, motor_id_8_pin}},
                                {"negative", {motor_id_5_pin, motor_id_6_pin}}}},
            {"pitch-backward", {{"positive", {motor_id_5_pin, motor_id_7_pin}},
                                {"negative", {motor_id_6_pin, motor_id_8_pin}}}},
            {"pitch-forward", {{"positive", {motor_id_6_pin, motor_id_8_pin}},
                                {"negative", {motor_id_5_pin, motor_id_7_pin}}}}
    };
    return motion_to_motor_pins_map;
}
