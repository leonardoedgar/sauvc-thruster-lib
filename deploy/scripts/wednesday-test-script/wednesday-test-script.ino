# include <motor_driver.h>
# include <motor_controller.h>
# include <ArduinoSTL.h>
# include <Wire.h>
# include <MS5837.h>
# include "depth_sensor_config.h"

// Initialise Motor Controller
MotorController motor_controller;

// Initialise e-stop pin
const byte e_stop_pin = 10;

// Initialise depth sensor
MS5837 depth_sensor;
float current_depth;

// Initialise the state of the robot
std::map <String, std::map<String, String>> state_map = {
  {"stop", {{"fail", "submerge"}}},
  {"submerge", {{"success", "move"}, {"fail", "stop"}}},
  {"move", {{"success", "surface"}, {"fail", "stop"}}},
  {"surface", {{"success", "stop"}, {"fail", "stop"}}}
};
String state = "stop";

// Initialise timer
unsigned long start_time;
unsigned long submerge_timeout = 3, move_timeout = 3, surface_timeout = 3;

void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup motor controller
  motor_controller.setup();

  // Setup e-stop pin
  pinMode(e_stop_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(e_stop_pin), transit_state, CHANGE);

   // Setup depth sensor
   Wire.begin();
   while (!depth_sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  depth_sensor.setModel(DEPTH_SENSOR_MODEL);
  depth_sensor.setFluidDensity(FLUID_DENSITY); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Code to test thrusters' functionality.
  if (state == "stop") {
    Serial.println("state: stop");
    start_time = millis();
  }
  else if (state == "submerge") {
    Serial.println("state: submerge");
    if (millis() - start_time > submerge_timeout) {
      state = state_map[state]["success"];
    }
  }
  else if (state == "move") {
    Serial.println("state: move");
    if (millis() - start_time > move_timeout) {
      state = state_map[state]["success"];
    }
  }
  else if (state == "surface") {
    Serial.println("state: surface");
    if (millis() - start_time > surface_timeout) {
      state = state_map[state]["success"];
    }
  }
}

/**
 * A function to transit state when interrupt is triggered..
 * return {bool} that indicates whether the interrupt callback is successful or not.
 */
bool transit_state () {
  state = state_map[state]["fail"];
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
