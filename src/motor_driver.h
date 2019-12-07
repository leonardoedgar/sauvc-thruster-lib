# ifndef SAUVC2020_MOTOR_DRIVER_H
# define SAUVC2020_MOTOR_DRIVER_H

class MotorDriver {
private:
    // An integer that represents the pin of the motor
    int pin;
public:
    MotorDriver();
    MotorDriver(int pin);
    /**
     * A function to retrieve the pin of the motor.
     * @return {int} the id of the motor
     */
    int retrieve_motor_pin();

    /**
     * A function to run the motor at a certain speed.
     * @param speed {double} that represents the speed to run the motor
     * @return {bool} that indicates whether the motor runs successfully or not
     */
    bool run(double speed);
};

#endif //SAUVC2020_MOTOR_DRIVER_H
