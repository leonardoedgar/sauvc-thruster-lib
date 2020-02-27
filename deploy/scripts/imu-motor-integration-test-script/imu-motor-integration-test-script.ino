/*
* rosserial Subscriber Example
*/
# include <ros.h>
# include <sauvc2020_msgs/MotorSpeed.h>
# include <geometry_msgs/QuaternionStamped.h>
# include <motor_driver.h>
# include <motor_controller.h>

// Initialise Motor Controller
MotorController motor_controller; 
ros::NodeHandle nh;

geometry_msgs::QuaternionStamped str_msg;
ros::Subscriber<sauvc2020_msgs::MotorSpeed> sub("chatter", &motor_controller.update_motor_stabilised_speed);
ros::Publisher chatter("chatter_resp", &str_msg); 

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(9600);
  motor_controller.setup();
  motor_controller.init_ros_communication(chatter, sub);
  Serial.begin(9600);
}
 
void loop() {
  motor_controller.move("forward");
  motor_controller.publish_robot_motion();
  nh.spinOnce();
  delay(500);
}
