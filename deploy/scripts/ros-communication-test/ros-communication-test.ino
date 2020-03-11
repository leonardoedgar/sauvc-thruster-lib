# include <ros.h>
# include <sauvc_msgs/ThrustersSpeed.h>
# include <sauvc_msgs/MotionData.h>

ros::NodeHandle nh;
sauvc_msgs::MotionData motion_data;

void messageCb(sauvc_msgs::ThrustersSpeed& msg){
  motion_data.motion = "forward";
  if (int(msg.thruster_id1_speed) > 1700 ) {
    msg.thruster_id1_speed = 1700;
  }
  else if (int(msg.thruster_id1_speed) < 1300) {
    msg.thruster_id1_speed = 1300;
  }
  if (int(msg.thruster_id2_speed) > 1700 ) {
    msg.thruster_id2_speed = 1700;
  }
  else if (int(msg.thruster_id2_speed) < 1300) {
    msg.thruster_id2_speed = 1300;
  }
  motion_data.thrusters_speed.thruster_id1_speed = int(msg.thruster_id1_speed);
  motion_data.thrusters_speed.thruster_id2_speed = int(msg.thruster_id2_speed);
  motion_data.thrusters_speed.thruster_id3_speed = int(msg.thruster_id3_speed);
  motion_data.thrusters_speed.thruster_id4_speed = int(msg.thruster_id4_speed);
  motion_data.thrusters_speed.thruster_id5_speed = int(msg.thruster_id5_speed);
  motion_data.thrusters_speed.thruster_id6_speed = int(msg.thruster_id6_speed);
  motion_data.thrusters_speed.thruster_id7_speed = int(msg.thruster_id7_speed);
  motion_data.thrusters_speed.thruster_id8_speed = int(msg.thruster_id8_speed);
}
 
ros::Subscriber<sauvc_msgs::ThrustersSpeed> sub("/thrusters/stabilised_speed", &messageCb );
ros::Publisher chatter("/auv/motion", &motion_data); 

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  Serial.begin(57600);
}
 
void loop() {
  chatter.publish(&motion_data);
  nh.spinOnce();
  delay(500);
}
