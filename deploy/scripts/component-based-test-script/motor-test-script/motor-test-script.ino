# include <motor_driver.h>
# include <motor_controller.h>

// Initialise Motor Controller
MotorController motor_controller;

void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup motor controller
  motor_controller.setup();
}

void loop() {
  // Code to test thrusters.
  motor_controller.move("forward");
  delay(1000);
  motor_controller.stop();
  delay(2000);
}
