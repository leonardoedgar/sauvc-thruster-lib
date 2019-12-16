#include <motor_driver.h>
#include <motor_controller.h>
#include <ArduinoSTL.h>
#include <Wire.h>
#include <MS5837.h>

// Initialise Motor Controller
std::vector<byte> motor_pins {3, 4, 5, 6, 7, 8, 9, 10}; 
MotorController motor_controller;

// Initialise variable to setup timer during robot operation
unsigned long start_time;
unsigned long operating_timeout = 15000;
unsigned long time_elapsed;

// Initialise e-stop pin
const byte e_stop_pin = 2;

// Initialise depth sensor
MS5837 depth_sensor;
float initial_depth;
float operating_depth = -1, depth_tolerance = 0.25;

// Initialise the state of the robot
volatile byte state = LOW;

void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup motor controller
  motor_controller.setup(motor_pins);

  // Setup e-stop pin
   pinMode(e_stop_pin, INPUT);
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
  depth_sensor.setModel(MS5837::MS5837_30BA);
  depth_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  if (state == LOW) {
    Serial.println("Robot state: stop");
    motor_controller.stop();
    start_time = millis();
  }
  else if (state == HIGH) {
    Serial.println("Robot state: run");
    time_elapsed = millis() - start_time;
    initial_depth = get_current_depth();
    if (time_elapsed > operating_timeout) {
      motor_controller.stop();
    }
    else {
      if (operating_depth - depth_tolerance < get_current_depth() - initial_depth < operating_depth + depth_tolerance) {
        motor_controller.move("forward", 50);
      }
      else {
        motor_controller.move("submerge", 50);
      }
    }
  }
  else {
    Serial.println("Robot state: unknown");
  }
}

/**
 * A function to transit state when interrupt is triggered..
 * return {bool} that indicates whether the interrupt callback is successful or not.
 */
bool transit_state () {
  state = !state;
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
