#include <motor_driver.h>
#include <motor_controller.h>
#include <ArduinoSTL.h>

std::vector<int> motor_pins {1, 2, 3, 4, 5, 6, 7, 8}; 
MotorController motor_controller {motor_pins};

void setup() {
  // Delay 1 second to finish setting up all motor pins.
  delay(1000);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor_controller.move("forward", 75);
  delay(1000);
  motor_controller.stop();
  delay(1000);
  motor_controller.move("rotate-left", 100);
  delay(1000);
  motor_controller.stop();
  delay(1000);
}
