#ifndef SAUVC2020_MOTOR_DRIVER_H
#define SAUVC2020_MOTOR_DRIVER_H

class MotorDriver {
private:
    // An integer that represents the id of the motor
    int id;
public:
    MotorDriver(int id);
    /**
     * A function to retrieve the id of the motor.
     * @return {int} the id of the motor
     */
    int retrieve_motor_id();

    /**
     * A function to run the motor at a certain speed.
     * @param speed {double} that represents the speed to run the motor
     * @return {bool} that indicates whether the motor is run successfully or not
     */
    bool run(double speed);
};

#endif //SAUVC2020_MOTOR_DRIVER_H
