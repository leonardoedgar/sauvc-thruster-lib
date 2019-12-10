#include <motor_driver.h>
#include <motor_controller.h>
#include <ArduinoSTL.h>

int pin_motor1 = 1, 
    pin_motor2 = 2, 
    pin_motor3 = 3, 
    pin_motor4 = 4,
    pin_motor5 = 5,
    pin_motor6 = 6,
    pin_motor7 = 7,
    pin_motor8 = 8;
std::vector<int> motor_pins {pin_motor1, pin_motor2, pin_motor3, pin_motor4, pin_motor5, pin_motor6, pin_motor7, pin_motor8}; 
MotorController motor_controller {motor_pins};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor_controller.move("forward", 75);
  delay(1000);
  motor_controller.stop();
  delay(1000);
}
