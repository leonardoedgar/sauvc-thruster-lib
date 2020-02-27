# include <motor_driver.h>
# include <motor_controller.h>
# include <ArduinoSTL.h>
# include <Wire.h>
# include <MS5837.h>
# include "depth_sensor_config.h"
# include "config.h"
# include <ros.h>
# include <sauvc2020_msgs/MotionData.h>
# include <sauvc2020_msgs/MotorSpeed.h>

// Initialise Motor Controller
MotorController motor_controller;

// Initialise depth sensor
MS5837 depth_sensor;
float current_depth;

// Initialise the state of the estop
volatile byte e_stop_state = LOW;

// Initialise timer
unsigned long start_time;

ros::NodeHandle nh;

sauvc2020_msgs::MotionData motion_data;
ros::Publisher pub("/motor/motion", &motion_data);
void publish_motion() {
  std::map<int, int> motors_speed = motor_controller.get_motors_speed();
  motion_data.motion = motor_controller.get_motion().c_str();
  motion_data.motors_speed.motor_id1_speed = motors_speed[1];
  motion_data.motors_speed.motor_id2_speed = motors_speed[2];
  motion_data.motors_speed.motor_id3_speed = motors_speed[3];
  motion_data.motors_speed.motor_id4_speed = motors_speed[4];
  motion_data.motors_speed.motor_id5_speed = motors_speed[5];
  motion_data.motors_speed.motor_id6_speed = motors_speed[6];
  motion_data.motors_speed.motor_id7_speed = motors_speed[7];
  motion_data.motors_speed.motor_id8_speed = motors_speed[8];
  pub.publish(&motion_data);
  delay(500);
}

void update_stabilised_speed( const sauvc2020_msgs::MotorSpeed& msg){
  std::map<int, int> stabilised_speed = {
    {1, int(msg.motor_id1_speed)}, {2, int(msg.motor_id2_speed)}, {3, int(msg.motor_id3_speed)},
    {4, int(msg.motor_id1_speed)}, {5, int(msg.motor_id1_speed)}, {6, int(msg.motor_id1_speed)},
    {7, int(msg.motor_id7_speed)}, {8, int(msg.motor_id8_speed)}};  
  motor_controller.set_stabilised_speed(stabilised_speed); 
}
ros::Subscriber<sauvc2020_msgs::MotorSpeed> sub("/motor/stabilised_speed", update_stabilised_speed );

/**
 * A callback function when estop is pressed.
 * return {bool} that indicates whether the callback function successfully executed.
 */
bool callback_estop_pressed() {
  e_stop_state = !e_stop_state;
  return true;
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
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  // Setup serial monitor
  Serial.begin(9600);

  // Setup motor controller
  motor_controller.setup();

  // Setup e-stop pin
  pinMode(ESTOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), callback_estop_pressed, CHANGE);

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
        current_depth = get_current_depth();
        if(current_depth < OPERATING_DEPTH - DEPTH_TOLERANCE) {
          motor_controller.move("submerge");
        }
        else if (current_depth > OPERATING_DEPTH + DEPTH_TOLERANCE){
          motor_controller.move("surface");
        }
        else {
          motor_controller.move("forward");
        }
      }
    }
    else {
      Serial.println("state: [submerge]");
      motor_controller.move("submerge");
      start_time = millis();
    }
  }
  publish_motion();
}
