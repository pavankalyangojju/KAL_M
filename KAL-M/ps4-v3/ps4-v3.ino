#include <Bluepad32.h>

// Define motor pins (using GPIO 12, 14, 27, and 26)
int motor1Pin1 = 12;  // Motor 1 direction pin 1
int motor1Pin2 = 14;  // Motor 1 direction pin 2
int motor2Pin1 = 27;  // Motor 2 direction pin 1
int motor2Pin2 = 26;  // Motor 2 direction pin 2

// Define LED pin
int ledPin = 2;  // LED pin
GamepadPtr myGamepad;

void setup() {
  // Set motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

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
    // Read left joystick position for motor control
    int16_t leftStickY = myGamepad->axisY(); // Use axisY() for left joystick vertical movement

    // Motor control based on left joystick Y-axis
    if (abs(leftStickY) > 20) { // Deadband to avoid small joystick movements
      if (leftStickY > 0) {
        // Move motor forward
        setMotorDirection(true);
      } else {
        // Move motor backward
        setMotorDirection(false);
      }
    } else {
      // Stop all motors
      stopAllMotors();
    }

    // Read right joystick position for LED control
    int16_t rightStickY = myGamepad->axisRY(); // Use axisRY() for right joystick vertical movement
    if (abs(rightStickY) > 20) { // Deadband to avoid small joystick movements
      digitalWrite(ledPin, HIGH);  // Turn the LED on
      delay(100);                   // LED on duration
      digitalWrite(ledPin, LOW);   // Turn the LED off
      delay(100);                   // LED off duration
    } else {
      // If the joystick is centered, turn the LED off
      digitalWrite(ledPin, LOW);
    }
  }

  delay(20);  // Small delay to make the code run smoother
}

// Set direction for the motor
void setMotorDirection(bool forward) {
  if (forward) {
    // Set the motor to move forward
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    // Set the motor to move backward
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
}

// Stop all motors
void stopAllMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
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
