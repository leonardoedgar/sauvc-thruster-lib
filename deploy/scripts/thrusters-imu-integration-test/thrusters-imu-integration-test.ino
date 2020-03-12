# include <ros.h>
# include <sauvc_msgs/ThrustersSpeed.h>
# include <sauvc_msgs/MotionData.h>
# include <auv.h>

ros::NodeHandle nh;
sauvc_msgs::MotionData motion_data;

AUV auv;
unsigned long time;
unsigned long time_out = 5000;

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
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  Serial.begin(57600);
  auv.setup();
  time = millis();
}
 
void loop() {
  if (millis() - time < time_out) {
    auv.move("forward");
  }
  else {
    auv.stop();
    
  }
  update_motion_data();
  chatter.publish(&motion_data);
  nh.spinOnce();
  delay(500);

}

void update_motion_data() {
  motion_data.motion = auv.get_motion().c_str();
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
