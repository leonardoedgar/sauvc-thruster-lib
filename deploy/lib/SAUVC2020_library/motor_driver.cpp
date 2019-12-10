# include "motor_driver.h"
# include <Arduino.h>
# include <string>
# include <Servo.h>
MotorDriver::MotorDriver(int pin): pin{pin}{
//     servo_object.attach(pin);
    // servo_object.writeMicroseconds(1500);
}

MotorDriver::MotorDriver() {
    Serial.println("Motor driver without args is called.");
    MotorDriver(-1);
}

/**
 * The implementation of a function to get the pin of a motor.
 * @return {int} represents the pin of the motor
 */
int MotorDriver::get_motor_pin() {
    return pin;
}

/**
 * The implementation of the function to run a motor at a certain speed.
 * @param speed {double} represents the speed_percentage to run the motor
 * @return {bool} indicates whether the motor runs successfully or not
 */
bool MotorDriver::run(double speed_percentage, std::string direction) {
    if (direction == "positive") {
        String string_to_print = "Motor with pin: " + String(pin) + " is running with ESC input: " +
                String(map_speed_percentage_to_esc_input(speed_percentage)) + ".";
        Serial.println(string_to_print);
        // servo_object.writeMicroseconds(map_speed_percentage_to_esc_input(speed_percentage));
    }
    else if (direction == "negative") {
        String string_to_print = "Motor with pin: " + String(pin) + " is running with ESC input: " +
                String(map_speed_percentage_to_esc_input(-speed_percentage)) + ".";
        Serial.println(string_to_print);
        // servo_object.writeMicroseconds(map_speed_percentage_to_esc_input(-speed_percentage));
    }
}

/**
 * A function to stop the motor.
 * @return {bool} indicates whether the stopping was successful or not
 */
bool MotorDriver::stop() {
    run(0, "positive");
    return true;
}

/**
 * The implementation of the function to map speed percentage to the esc input
 * @param speed_percentage {double} indicates the speed percentage to run the motor
 * @return {double} represents the esc input
 */
double MotorDriver::map_speed_percentage_to_esc_input(double speed_percentage) {
    int max_esc_input = 1700, min_esc_input = 1500;
    int max_speed_percentage = 100, min_speed_percentage = -100;
    if (speed_percentage > max_speed_percentage) {
        speed_percentage = max_speed_percentage;
    }
    else if (speed_percentage < min_speed_percentage){
        speed_percentage = min_speed_percentage;
    }
    return min_esc_input + (max_esc_input-min_esc_input)*speed_percentage/100;
}