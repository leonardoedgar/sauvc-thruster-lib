# ifndef SAUVC2020_MOTOR_DRIVER_H
# define SAUVC2020_MOTOR_DRIVER_H
# include <string>
# include <Arduino.h>
# include <Servo.h>
class MotorDriver {
private:
    byte pin;
    Servo servo_driver;
    /**
     * A function to get a safe esc input value
     * @param esc_input {int} indicates the desired esc input
     * @return {int} represents the safe esc input
     */
    int get_safe_esc_input(int esc_input);
public:
    MotorDriver(byte pin);
    MotorDriver();
    /**
     * A function to get the pin of the motor.
     * @return {byte} the pin of the motor
     */
    byte get_motor_pin();

    /**
     * The function to run a motor at a certain ESC input.
     * @param esc_input {int} represents the desired ESC input
     * @return {bool} indicates whether the motor runs successfully or not
     */
    bool run(int esc_input);
    /**
     * A function to stop the motor.
     * @return {bool} indicates whether the stopping was successful or not
     */
    bool stop();
};

#endif //SAUVC2020_MOTOR_DRIVER_H
