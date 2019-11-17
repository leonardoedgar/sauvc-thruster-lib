#ifndef SAUVC2020_MOTOR_DRIVER_H
#define SAUVC2020_MOTOR_DRIVER_H

class MotorDriver {
private:
    int id;
public:
    MotorDriver(int id);
    int retrieve_motor_id();
    void run(double speed);
};

#endif //SAUVC2020_MOTOR_DRIVER_H
