/*
* rosserial Subscriber Example
*/
# include <ros.h>
# include <motor_driver.h>
# include <motor_controller.h>

// Initialise Motor Controller
MotorController motor_controller; 
ros::NodeHandle nh;

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(9600);
  motor_controller.setup();
  nh.advertise(motor_controller.robot_motion_publisher);
  nh.subscribe(motor_controller.stabilised_speed_subscriber);
  Serial.begin(9600);
}
 
void loop() {
  motor_controller.move("forward");
  motor_controller.publish_robot_motion();
  nh.spinOnce();
  delay(500);
}
