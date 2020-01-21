# include <Wire.h>
# include <MS5837.h>
# include "depth_sensor_config.h"

// Initialise depth sensor
MS5837 depth_sensor;
float current_depth;

/**
 * A function to get the current robot operating level depth
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
    // Code to test depth sensor.
    current_depth = get_current_depth();
    delay(1000);
}
