# include <Servo.h>

// Initialise the gripper
Servo gripper_clamp;
const byte gripper_clamp_pin = 12;

void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup gripper
  gripper_clamp.attach(gripper_clamp_pin);
  gripper_clamp.writeMicroseconds(1500);
  delay(1000);
}

void loop() {
    // Code to test gripper
    gripper_clamp.writeMicroseconds(1550);
    Serial.println("Gripper clamp: running");
    delay(1000);
    gripper_clamp.writeMicroseconds(1500);
    Serial.println("Gripper clamp: stop");
    delay(3000);
}
