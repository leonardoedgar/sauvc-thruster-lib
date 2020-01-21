# include <motor_driver.h>
# include <motor_controller.h>
// Initialise e-stop pin
const byte e_stop_pin = 2;

// Initialise the state of the robot
std::map <String, std::map<String, String>> state_map = {
  {"stop", {{"next", "run"}}},
  {"run", {{"next", "stop"}}}
};
String state = "stop";

/**
 * A function to transit state when interrupt is triggered..
 * return {bool} that indicates whether the interrupt callback is successful or not.
 */
bool transit_state () {
  state = state_map[state]["next"];
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
    Serial.println("State: [" + state + "]");
    delay(500);
}
