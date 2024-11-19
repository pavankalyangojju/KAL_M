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

    // Check D-pad state
    int dpadState = myGamepad->dpad(); // Get the current D-pad state

    if (dpadState & DPAD_UP) {
      Serial.println("Up button pressed");
      ledState = HIGH; // Turn LED on
    }
    if (dpadState & DPAD_DOWN) {
      Serial.println("Down button pressed");
      ledState = HIGH; // Turn LED on
    }
    if (dpadState & DPAD_LEFT) {
      Serial.println("Left button pressed");
      ledState = HIGH; // Turn LED on
    }
    if (dpadState & DPAD_RIGHT) {
      Serial.println("Right button pressed");
      ledState = HIGH; // Turn LED on
    }

    digitalWrite(ledPin, ledState); // Set LED to the determined state
  } else {
    digitalWrite(ledPin, LOW); // Turn LED off if controller is not connected
  }
}
