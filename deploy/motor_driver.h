# ifndef SAUVC2020_MOTOR_DRIVER_H
# define SAUVC2020_MOTOR_DRIVER_H
# include <string>
class MotorDriver {
private:
    // An integer that represents the pin of the motor
    int pin;
    /**
    * The implementation of the function to map speed percentage to the esc input
    * @param speed_percentage {double} indicates the speed percentage to run the motor
    * @return {double} represents the esc input
    */
    double map_speed_percentage_to_esc_input(double speed_percentage);
public:
    MotorDriver();
    MotorDriver(int pin);
    /**
     * A function to get the pin of the motor.
     * @return {int} the pin of the motor
     */
    int get_motor_pin();

    /**
     * A function to run the motor at a certain speed.
     * @param speed {double} that represents the speed to run the motor
     * @return {bool} that indicates whether the motor runs successfully or not
     */
    bool run(double speed, std::string direction);
    /**
     * A function to stop the motor.
     * @return {bool} indicates whether the stopping was successful or not
     */
    bool stop();
};

#endif //SAUVC2020_MOTOR_DRIVER_H
