# include <motor_driver.h>
# include <motor_controller.h>

// Initialise Motor Controller
MotorController motor_controller;
std::map<int, int> customised_motor_speed = {{1, 1500}, 
                                             {2, 1550}, 
                                             {3, 1600}, 
                                             {4, 1550}, 
                                             {5, 1500}, 
                                             {6, 1450}, 
                                             {7, 1400}, 
                                             {8, 1450}};
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
  motor_controller.move(customised_motor_speed);
  delay(1000);
  motor_controller.stop();
  delay(2000);
}
