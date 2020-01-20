# include "motor_driver.h"
# include <Arduino.h>
# include <string>
# include <Servo.h>
# include "config/config.h"
MotorDriver::MotorDriver(byte pin): pin{pin}{
     servo_driver.attach(pin);
     servo_driver.writeMicroseconds(1500);
}

MotorDriver::MotorDriver() {
    Serial.println("Motor driver without args is called.");
}

/**
 * The implementation of a function to get the pin of a motor.
 * @return {byte} represents the pin of the motor
 */
byte MotorDriver::get_motor_pin() {
    return pin;
}

/**
 * The implementation of the function to run a motor at a certain ESC input value.
 * @param esc_input_value {int} represents the desired ESC input value
 * @return {bool} indicates whether the motor runs successfully or not
 */
bool MotorDriver::run(int esc_input) {
    Serial.println(
            "Motor with pin: " + String(pin) + " is running with ESC input: " +
            String(get_safe_esc_input(esc_input)) + ".");
    servo_driver.writeMicroseconds(get_safe_esc_input(esc_input));
    return true;
}

/**
 * A function to stop the motor.
 * @return {bool} indicates whether the stopping was successful or not
 */
bool MotorDriver::stop() {
    run(ESC_INPUT_FOR_STOP_SIGNAL);
    return true;
}

/**
 * The implementation of the function to get a safe esc input value
 * @param esc_input_value {int} indicates the desired esc input
 * @return {int} represents the safe esc input
 */
int MotorDriver::get_safe_esc_input(int esc_input) {
    if (esc_input > MAX_ESC_INPUT) {
        return MAX_ESC_INPUT;
    }
    else if (esc_input < MIN_ESC_INPUT){
        return MIN_ESC_INPUT;
    }
    return esc_input;
}
