#include <Bluepad32.h>

// Define LED pin
int ledPin = 2;  // LED pin
GamepadPtr myGamepad;

// Timer variables
unsigned long ledOnTime = 0; // Time when the LED was turned on
const unsigned long ledDuration = 2000; // Duration to keep the LED on (in milliseconds)

void setup() {
  // Set LED pin as output
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(115200);

  // Initialize Bluepad32 library
  BP32.setup(&onConnect, &onDisconnect);

  Serial.println("Waiting for PS4 controller to connect...");
}

void loop() {
  BP32.update();  // Update the controller state

  if (myGamepad) {
    // Read the joystick positions for control
    int16_t rightStickY = myGamepad->axisRY(); // Right joystick vertical movement
    int16_t rightStickX = myGamepad->axisRX(); // Right joystick horizontal movement
    int16_t leftStickY = myGamepad->axisY();   // Left joystick vertical movement
    int16_t leftStickX = myGamepad->axisX();   // Left joystick horizontal movement

    // Control the LED based on right joystick vertical movement
    if (abs(rightStickY) > 20) { // Vertical movement detected
      digitalWrite(ledPin, HIGH);  // Turn the LED on
      ledOnTime = millis(); // Record the time the LED was turned on
      Serial.println("Right joystick moved vertically."); // Serial print
    }

    // Control the LED based on left joystick vertical movement
    if (abs(leftStickY) > 20) { // Vertical movement detected
      digitalWrite(ledPin, HIGH);  // Turn the LED on
      ledOnTime = millis(); // Record the time the LED was turned on
      Serial.println("Left joystick moved vertically."); // Serial print
    }

    // Control the LED based on horizontal movements of either joystick
    if (abs(rightStickX) > 20) { // Horizontal movement detected
      digitalWrite(ledPin, LOW);   // Turn the LED off
      Serial.println("Right joystick moved horizontally."); // Serial print
    }
    if (abs(leftStickX) > 20) { // Horizontal movement detected
      digitalWrite(ledPin, LOW);   // Turn the LED off
      Serial.println("Left joystick moved horizontally."); // Serial print
    }

    // Turn off the LED after the specified duration, unless horizontal movement is detected
    if (digitalRead(ledPin) == HIGH && (millis() - ledOnTime >= ledDuration)) {
      digitalWrite(ledPin, LOW); // Turn the LED off after duration
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
