# include <motor_driver.h>
# include <motor_controller.h>
# include <ArduinoSTL.h>
# include <Wire.h>
# include <MS5837.h>
# include "depth_sensor_config.h"
# include "config.h"

// Initialise Motor Controller
MotorController motor_controller;

// Initialise depth sensor
MS5837 depth_sensor;
float current_depth;

// Initialise the state of the estop
volatile byte e_stop_state = LOW;

// Initialise timer
unsigned long start_time;

/**
 * A callback function when estop is pressed.
 * return {bool} that indicates whether the callback function successfully executed.
 */
bool callback_estop_pressed() {
  e_stop_state != e_stop_state;
}

/**
 * A function to get the current robot operating level depth.
 * return {float} that indicates the depth where the robot is operating at
 */
float get_current_depth() {
  depth_sensor.read();
  Serial.println("Depth sensor reading: " + String(depth_sensor.depth()));
  return depth_sensor.depth();
}

void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup motor controller
  motor_controller.setup();

  // Setup e-stop pin
  pinMode(ESTOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), callback_estop_pressed, CHANGE);

   // Setup depth sensor
//   Wire.begin();
//   while (!depth_sensor.init()) {
//    Serial.println("Init failed!");
//    Serial.println("Are SDA/SCL connected correctly?");
//    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
//    Serial.println("\n\n\n");
//    delay(5000);
//  }
//  depth_sensor.setModel(DEPTH_SENSOR_MODEL);
//  depth_sensor.setFluidDensity(FLUID_DENSITY); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  if (e_stop_state == LOW) {
    Serial.println("state: [stop]");
    motor_controller.stop();
    start_time = millis();
  }
  else {
    if (get_current_depth() > OPERATING_DEPTH - DEPTH_TOLERANCE) {
      if (millis() - start_time > MOVE_TIMEOUT) {
        if(get_current_depth() < DEPTH_TOLERANCE) {
          Serial.println("state: [stop]");
          motor_controller.stop();
        }
        else {
          Serial.println("state: [surface]");
          motor_controller.move("surface");
        }
      }
      else {
        Serial.println("state: [move]");
        motor_controller.move("forward");
      }
    }
    else {
      Serial.println("state: [submerge]");
      motor_controller.move("submerge");
      start_time = millis();
    }
  }
}
