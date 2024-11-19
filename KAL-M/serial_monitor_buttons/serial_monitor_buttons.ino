#include <Bluepad32.h>

const int ledPin = 2; // LED connected to GPIO 2
GamepadPtr myGamepad;

Bluepad32 bp32; // Declare Bluepad32 as an object

void onConnect(GamepadPtr gp) {
  myGamepad = gp;
  Serial.println("Controller connected!");
}

void onDisconnect(GamepadPtr gp) {
  myGamepad = nullptr;
  Serial.println("Controller disconnected!");
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  bp32.setup(&onConnect, &onDisconnect); // Use bp32 object to call setup
}

void loop() {
  bp32.update(); // Use bp32 object to call update

  // Check if controller is connected
if (myGamepad && myGamepad->isConnected()) {
    bool ledState = LOW; // Default LED state

    // Check each button individually and print its name
    if (myGamepad->a()) {
      Serial.println("X button pressed");
      ledState = HIGH; // Turn LED on
    }
    if (myGamepad->b()) {
      Serial.println("Circle button pressed");
      ledState = HIGH; // Turn LED on
    }
    if (myGamepad->x()) {
      Serial.println("Square button pressed");
      ledState = HIGH; // Turn LED on
    }
    if (myGamepad->y()) {
      Serial.println("Triangle button pressed");
      ledState = HIGH; // Turn LED on
    }

    digitalWrite(ledPin, ledState); // Set LED to the determined state
  } else {
    digitalWrite(ledPin, LOW); // Turn LED off if controller is not connected
  }
}
