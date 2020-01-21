# include <Servo.h>

// Initialise the gripper
Servo gripper_arm;
const byte gripper_arm_pin = 11;

void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup gripper
  gripper_arm.attach(gripper_arm_pin);
  gripper_arm.writeMicroseconds(1500);
  delay(1000);
}

void loop() {
    // Code to test gripper
    gripper_arm.writeMicroseconds(1550);
    Serial.println("Gripper arm: running");
    delay(1000);
    gripper_arm.writeMicroseconds(1500);
    Serial.println("Gripper arm: stop");
    delay(3000);
}
