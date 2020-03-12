# include <ros.h>
# include <sauvc_msgs/ThrustersSpeed.h>
# include <sauvc_msgs/MotionData.h>
# include <auv.h>
# include "config.h"
# include <Wire.h>
# include <MS5837.h>
# include "depth_sensor_config.h"

// Initialise ROS Node Handle
ros::NodeHandle nh;
sauvc_msgs::MotionData motion_data;

//Initialise AUV
AUV auv;

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

void messageCb(sauvc_msgs::ThrustersSpeed& msg){
  std::map<int, int> thrusters_id_to_stabilised_speed_map {
    {1, int(msg.thruster_id1_speed)},
    {2, int(msg.thruster_id2_speed)},
    {3, int(msg.thruster_id3_speed)},
    {4, int(msg.thruster_id4_speed)},
    {5, int(msg.thruster_id5_speed)},
    {6, int(msg.thruster_id6_speed)},
    {7, int(msg.thruster_id7_speed)},
    {8, int(msg.thruster_id8_speed)}};
  auv.update_stabilised_speed(thrusters_id_to_stabilised_speed_map);
}
 
ros::Subscriber<sauvc_msgs::ThrustersSpeed> sub("/thrusters/stabilised_speed", &messageCb );
ros::Publisher chatter("/auv/motion", &motion_data); 

void setup() {
  // Setup ROS Communications
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  Serial.begin(57600);
  
  //Setup AUV
  auv.setup();
  
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
    auv.stop();
    start_time = millis();
  }
  else {
    if (get_current_depth() > OPERATING_DEPTH - DEPTH_TOLERANCE) {
      if (millis() - start_time > MOVE_TIMEOUT) {
        if(get_current_depth() < DEPTH_TOLERANCE) {
          Serial.println("state: [stop]");
          auv.stop();
        }
        else {
          Serial.println("state: [surface]");
          auv.move("surface");
        }
      }
      else {
        Serial.println("state: [move]");
        current_depth = get_current_depth();
        if(current_depth < OPERATING_DEPTH - DEPTH_TOLERANCE) {
          auv.move("submerge");
        }
        else if (current_depth > OPERATING_DEPTH + DEPTH_TOLERANCE){
          auv.move("surface");
        }
        else {
          auv.move("forward");
        }
      }
    }
    else {
      Serial.println("state: [submerge]");
      auv.move("submerge");
      start_time = millis();
    }
  }
  update_motion_data();
  chatter.publish(&motion_data);
  nh.spinOnce();
  delay(100);
}

void update_motion_data() {
  if (auv.get_motion() == "forward") {
    motion_data.motion = "forward";
  }
  else if (auv.get_motion() == "submerge"){
    motion_data.motion = "submerge";
  }
  else if (auv.get_motion() == "surface") {
    motion_data.motion = "surface";
  }
  else if (auv.get_motion() == "stop") {
    motion_data.motion = "stop";
  }
  std::map <int, int> thrusters_id_to_actual_speed_map = auv.get_actual_thrusters_speed();
  motion_data.thrusters_speed.thruster_id1_speed = thrusters_id_to_actual_speed_map[1];
  motion_data.thrusters_speed.thruster_id2_speed = thrusters_id_to_actual_speed_map[2];
  motion_data.thrusters_speed.thruster_id3_speed = thrusters_id_to_actual_speed_map[3];
  motion_data.thrusters_speed.thruster_id4_speed = thrusters_id_to_actual_speed_map[4];
  motion_data.thrusters_speed.thruster_id5_speed = thrusters_id_to_actual_speed_map[5];
  motion_data.thrusters_speed.thruster_id6_speed = thrusters_id_to_actual_speed_map[6];
  motion_data.thrusters_speed.thruster_id7_speed = thrusters_id_to_actual_speed_map[7];
  motion_data.thrusters_speed.thruster_id8_speed = thrusters_id_to_actual_speed_map[8];
}
