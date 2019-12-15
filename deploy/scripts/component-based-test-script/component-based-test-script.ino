#include <motor_driver.h>
#include <motor_controller.h>
#include <ArduinoSTL.h>
#include <Wire.h>
#include <MS5837.h>

// Initialise Motor Controller
std::vector<byte> motor_pins {3, 4, 5, 6, 7, 8, 9, 10}; 
MotorController motor_controller;

// Initialise e-stop pin
const byte e_stop_pin = 10;

// Initialise depth sensor
MS5837 depth_sensor;
float operating_depth = -1, depth_tolerance = 0.25, current_depth = 0;

// Initialise the state of the robot
std::map <String, std::map<String, String>> state_map = {
  {"stop", {{"next", "run"}}},
  {"run", {{"next", "stop"}}}
};
String state = "stop";
void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup motor controller
  motor_controller.setup(motor_pins);

  // Setup e-stop pin
//   pinMode(e_stop_pin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(e_stop_pin), transit_state, CHANGE);

   // Setup depth sensor
//   Wire.begin();
//  while (!depth_sensor.init()) {
//    Serial.println("Init failed!");
//    Serial.println("Are SDA/SCL connected correctly?");
//    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
//    Serial.println("\n\n\n");
//    delay(5000);
//  }
//  depth_sensor.setModel(MS5837::MS5837_30BA);
//  depth_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Code to test thrusters' functionality.
  motor_controller.move("forward", 50);
  delay(1000);
  motor_controller.stop();
  delay(2000);

    // Code to test e-stop's functionality.
//    Serial.println(state);
//    delay(500);

    // Code to test depth sensor's functionality.
//
//    current_depth = get_current_depth();
//    if (operating_depth + depth_tolerance < current_depth) {
//      Serial.println("Robot go up"); 
//    }
//    else if (operating_depth-depth_tolerance > current_depth) {
//      Serial.println("Robot go down");
//    }
//    else {
//      Serial.println("Robot stays vertically");
//    }
//    delay(500);
}

/**
 * A function to transit state when interrupt is triggered..
 * return {bool} that indicates whether the interrupt callback is successful or not.
 */
bool transit_state () {
  state = state_map[state]["next"];
  return true;
}

/**
 * A function to get the current robot operating level depth
 * return {float} that indicates the depth where the robot is operating at
 */
float get_current_depth() {
  depth_sensor.read();
  Serial.println("Depth sensor reading: " + String(depth_sensor.depth()));
  return depth_sensor.depth();
}
