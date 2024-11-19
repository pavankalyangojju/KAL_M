#include <Bluepad32.h>

// Define LED pin
int ledPin = 2;  // LED pin
GamepadPtr myGamepad;
bool ledState = LOW;  // Variable to store LED state

void setup() {
  // Set LED pin as output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);  // Initialize LED to be off

  Serial.begin(115200);
  // Initialize Bluepad32 library
  BP32.setup(&onConnect, &onDisconnect);
  Serial.println("Waiting for PS4 controller to connect...");
}

void loop() {
  BP32.update();  // Update the controller state
  if (myGamepad) {
    // Check if L1 button is pressed
    if (myGamepad->l1()) {
      Serial.println("L1 button pressed");
      // Toggle LED state
      ledState = !ledState;
      digitalWrite(ledPin, ledState);  // Update LED state
      // Wait for button release to avoid multiple toggles from a single press
      while (myGamepad->l1()) {
        BP32.update();  // Keep updating controller state
      }
    }
    
    // Check if R1 button is pressed
    if (myGamepad->r1()) {
      Serial.println("R1 button pressed");
      // Set LED state to LOW
      ledState = LOW;  // Set LED state to LOW
      digitalWrite(ledPin, ledState);  // Update LED state
      // Wait for button release to avoid multiple toggles from a single press
      while (myGamepad->r1()) {
        BP32.update();  // Keep updating controller state
      }
    }

    if (myGamepad->l2()) {
      Serial.println("L2 button pressed");
      // Toggle LED state
      ledState = !ledState;
      digitalWrite(ledPin, ledState);  // Update LED state
      // Wait for button release to avoid multiple toggles from a single press
      while (myGamepad->l1()) {
        BP32.update();  // Keep updating controller state
      }
    }
    
    // Check if R1 button is pressed
    if (myGamepad->r2()) {
      Serial.println("R2 button pressed");
      // Set LED state to LOW
      ledState = LOW;  // Set LED state to LOW
      digitalWrite(ledPin, ledState);  // Update LED state
      // Wait for button release to avoid multiple toggles from a single press
      while (myGamepad->r1()) {
        BP32.update();  // Keep updating controller state
      }
    }
  }
  delay(20);  // Small delay to make the code run smoother
}

// Called when a Gamepad connects
void onConnect(GamepadPtr gp) {
  Serial.println("Controller connected!");
  myGamepad = gp;
}

// Called when a Gamepad disconnects
void onDisconnect(GamepadPtr gp) {
  Serial.println("Controller disconnected!");
  myGamepad = nullptr;
}
