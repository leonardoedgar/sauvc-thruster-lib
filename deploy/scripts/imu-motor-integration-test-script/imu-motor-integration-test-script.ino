# include <motor_driver.h>
# include <motor_controller.h>
# include <ArduinoSTL.h>
# include <ros.h>
# include <sauvc2020_msgs/MotionData.h>
# include <sauvc2020_msgs/MotorSpeed.h>

// Initialise Motor Controller
MotorController motor_controller;

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

void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  // Setup serial monitor
  Serial.begin(9600);

  // Setup motor controller
  motor_controller.setup();
}

void loop() {
  motor_controller.move("forward");
  publish_motion();
}
