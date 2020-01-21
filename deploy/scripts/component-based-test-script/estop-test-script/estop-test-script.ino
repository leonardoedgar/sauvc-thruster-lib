# include <motor_driver.h>
# include <motor_controller.h>
// Initialise e-stop pin
const byte e_stop_pin = 2;

// Initialise the state of the robot
volatile byte state = LOW;

/**
 * A function to transit state when interrupt is triggered..
 * return {bool} that indicates whether the interrupt callback is successful or not.
 */
bool transit_state () {
  state = !state;
  return true;
}

void setup() {
  // Setup serial monitor
  Serial.begin(9600);

  // Setup e-stop pin
  pinMode(e_stop_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(e_stop_pin), transit_state, CHANGE);
}

void loop() {
    // Code to test e-stop.
    if (state == LOW) {
      Serial.println("State: stop");
    }
    else {
      Serial.println("State: run");
    }
    delay(500);
}
