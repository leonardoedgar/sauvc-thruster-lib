#include <motor_driver.h>
#include <motor_controller.h>
#include <ArduinoSTL.h>
#include <Wire.h>
#include <MS5837.h>

std::vector<int> motor_pins {1, 2, 3, 4, 5, 6, 7, 8}; 
MotorController motor_controller {motor_pins};

int e_stop_pin = 7;

MS5837 depth_sensor;
float operating_depth = -1, depth_tolerance = 0.25, current_depth = 0;

void setup() {
  // Delay 1 second to finish setting up all motor pins.
  delay(1000);
  
//   pinMode(e_stop_pin, INPUT);
   
   Serial.begin(9600);
   
//   Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
//  while (!depth_sensor.init()) {
//    Serial.println("Init failed!");
//    Serial.println("Are SDA/SCL connected correctly?");
//    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
//    Serial.println("\n\n\n");
//    delay(5000);
//  }
  depth_sensor.setModel(MS5837::MS5837_30BA);
  depth_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Code to test thrusters' functionality.
  Serial.println("main loop");
//  motor_controller.move("forward", 75);
//  delay(1000);
//  motor_controller.stop();
//  delay(1000);
//  motor_controller.move("rotate-left", 100);
//  delay(1000);
//  motor_controller.stop();
//  delay(1000);

    // Code to test e-stop's functionality.

    int val = digitalRead(e_stop_pin);
    Serial.println(val);
    if (is_estop_pressed()) {
      Serial.println("Pressed.");
    }
    else {
      Serial.println("Not pressed.");
    }
    delay(500);

    // Code to test depth sensor's functionality.
//
//    current_depth = get_current_depth();
//    if (operating_depth + depth_tolerance < current_depth) {
//      Serial.println("Motor go up"); 
//    }
//    else if (operating_depth-depth_tolerance > current_depth) {
//      Serial.println("Motor go down");
//    }
//    else {
//      Serial.println("Do nothing");
//    }
    delay(500);
}

/**
 * A function to check whether the e-stop is pressed.
 * return {bool} that indicates true if the e-stop is pressed
 */
bool is_estop_pressed () {
  int val = digitalRead(e_stop_pin);
  Serial.println(val);
  if (val == 1) {
    return true;
  }
  else {
    return false;
  }
}

/**
 * A function to get the current robot operating level depth
 * return {float} that indicates the depth where the robot is operating at
 */
float get_current_depth() {
  depth_sensor.read();
  Serial.println(depth_sensor.depth());
  return depth_sensor.depth();
}
