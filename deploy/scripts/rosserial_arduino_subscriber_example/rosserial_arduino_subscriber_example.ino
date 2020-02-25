/*
* rosserial Subscriber Example
*/
# include <ros.h>
# include <sauvc2020_msgs/MotorSpeed.h>
 
ros::NodeHandle nh;
int counter = 0;
sauvc2020_msgs::MotorSpeed motor_speed_data;

void messageCb(sauvc2020_msgs::MotorSpeed& msg){
  counter = int(msg.motor_id1_input_speed);
  motor_speed_data = msg;
}
 
ros::Subscriber<sauvc2020_msgs::MotorSpeed> sub("chatter", &messageCb );
ros::Publisher chatter("chatter_resp", &motor_speed_data); 

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  nh.getHardware()->setBaud(9600);
  Serial.begin(9600);
}
 
void loop() {
  chatter.publish(&motor_speed_data);
  nh.spinOnce();
  delay(500);
}
