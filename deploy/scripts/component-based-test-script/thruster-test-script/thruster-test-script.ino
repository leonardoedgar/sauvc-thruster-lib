# include <auv.h>

// Initialise AUV
AUV auv;

void setup() {
  // Setup serial monitor
  Serial.begin(9600);
  auv.setup();
}

void loop() {
  // Code to test thrusters.
  auv.move("forward");
  delay(1000);
  auv.stop();
  delay(2000);
}
