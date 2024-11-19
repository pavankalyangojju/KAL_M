#include <Bluepad32.h>
#include <ESP32Servo.h>  // Servo library for ESP32

// Define motor pins (using GPIO 12, 14, 27, and 26)
int motor1Pin1 = 12;  // Motor 1 direction pin 1
int motor1Pin2 = 14;  // Motor 1 direction pin 2
int motor2Pin1 = 27;  // Motor 2 direction pin 1
int motor2Pin2 = 26;  // Motor 2 direction pin 2

// Define the servo motor pin
int servoPin = 19;  // Servo motor pin
Servo myServo;

GamepadPtr myGamepad;

void setup() {
  // Set motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Attach servo to the specified pin
  myServo.attach(servoPin);
  myServo.write(90);  // Initialize servo to 90 degrees

  Serial.begin(115200);

  // Initialize Bluepad32 library
  BP32.setup(&onConnect, &onDisconnect);

  Serial.println("Waiting for PS4 controller to connect...");
}

void loop() {
  BP32.update();  // Update the controller state

  if (myGamepad) {
    // Read left joystick position
    int16_t leftStickX = myGamepad->axisX(); // Horizontal movement (left/right)
    int16_t leftStickY = myGamepad->axisY(); // Vertical movement (up/down)

    // Motor control based on left joystick Y-axis
    if (abs(leftStickY) > 20) { // Deadband to avoid small joystick movements
      if (leftStickY > 0) {
        // Move motors forward
        setMotorDirection(true);
      } else {
        // Move motors backward
        setMotorDirection(false);
      }
    } else {
      // Stop all motors
      stopAllMotors();
    }

    // Servo motor control based on left joystick X-axis
    if (abs(leftStickX) > 20) { // Deadband to avoid small joystick movements
      if (leftStickX > 0) {
        myServo.write(0);  // Rotate servo to 0 degrees (right)
      } else {
        myServo.write(180);  // Rotate servo to 180 degrees (left)
      }
    } else {
      myServo.write(90);  // Rotate servo to 90 degrees (center)
    }
  }

  delay(20);  // Small delay to make the code run smoother
}

// Set direction for all motors
void setMotorDirection(bool forward) {
  if (forward) {
    // Set all motors to move in one direction
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    // Set all motors to move in the opposite direction
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
