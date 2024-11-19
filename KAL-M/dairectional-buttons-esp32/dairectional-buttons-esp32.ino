#include <Bluepad32.h>
#include <ESP32Servo.h>  // Use ESP32 Servo library

// Define the enable pins for the motors
const int motor1_enable_pin = 13;  // Motor 1 enable pin
const int motor2_enable_pin = 25;  // Motor 2 enable pin
// Define the motor control pins
const int motor1_forward_pin = 12;   // Motor 1 forward pin
const int motor1_backward_pin = 14;  // Motor 1 backward pin
const int motor2_forward_pin = 27;   // Motor 2 forward pin
const int motor2_backward_pin = 26;  // Motor 2 backward pin
const int buzzer = 15;               // Buzzer pin

// Define the motor control range
const int motor_range_max = 255;   // Maximum speed (forward)
const int motor_range_min = -255;  // Minimum speed (backward)

// Initialize the controller array
ControllerPtr myControllers[BP32_MAX_GAMEPADS] = { nullptr };


// Flag to track whether a controller is connected
bool controllerConnected = false;

// Function to control motors
void controlMotor(int forwardPin, int backwardPin, int speed) {
  Serial.printf("Controlling motor: forwardPin=%d, backwardPin=%d, speed=%d\n", forwardPin, backwardPin, speed);

  if (speed > 0) {
    // Move the motor forward
    analogWrite(forwardPin, speed);
    analogWrite(backwardPin, 0);
    Serial.println("Motor moving forward.");
  } else if (speed < 0) {
    // Move the motor backward
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, -speed);
    Serial.println("Motor moving backward.");
  } else {
    // Stop the motor
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, 0);
    Serial.println("Motor stopped.");
  }
}

// Function to stop all motors and servos
void stopAllMotorsAndServos() {
  // Stop all motors
  controlMotor(motor1_forward_pin, motor1_backward_pin, 0);
  controlMotor(motor2_forward_pin, motor2_backward_pin, 0);
  digitalWrite(motor1_enable_pin, LOW);
  digitalWrite(motor2_enable_pin, LOW);
  Serial.println("All motors stopped.");
}

// Callback function for connected controller
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
      myControllers[i] = ctl;
      controllerConnected = true;
      printBatteryStatus(ctl);
      break;
    }
  }
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(500);
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(500);
}

// Callback function for disconnected controller
void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      controllerConnected = false;
      stopAllMotorsAndServos();
      break;
    }
  }
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(500);
}

// Function to print battery status of a controller
void printBatteryStatus(ControllerPtr ctl) {
  int batteryLevel = ctl->battery();

  if (batteryLevel >= 0) {
    Serial.printf("Controller index=%d battery level: %d%%\n", ctl->index(), batteryLevel);
  } else {
    Serial.printf("Controller index=%d battery level: Not available\n", ctl->index());
  }
}



// Process gamepad input to control motors and servos
void processGamepad(ControllerPtr ctl) {

  // Check the state of the D-pad buttons
  uint8_t dpadState = ctl->dpad();

  // Handle D-pad input for movement control
  if (dpadState & DPAD_UP) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    Serial.println("DPAD UP pressed: Moving motors forward.");
    controlMotor(motor1_forward_pin, motor1_backward_pin, motor_range_max);
    controlMotor(motor2_forward_pin, motor2_backward_pin, motor_range_max);
  } else if (dpadState & DPAD_DOWN) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    Serial.println("DPAD DOWN pressed: Moving motors backward.");
    controlMotor(motor1_forward_pin, motor1_backward_pin, motor_range_min);
    controlMotor(motor2_forward_pin, motor2_backward_pin, motor_range_min);
  } else if (dpadState & DPAD_LEFT) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    Serial.println("DPAD LEFT pressed: Turning left.");
    controlMotor(motor1_forward_pin, motor1_backward_pin, motor_range_min);
    controlMotor(motor2_forward_pin, motor2_backward_pin, motor_range_max);
  } else if (dpadState & DPAD_RIGHT) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    Serial.println("DPAD RIGHT pressed: Turning right.");
    controlMotor(motor1_forward_pin, motor1_backward_pin, motor_range_max);
    controlMotor(motor2_forward_pin, motor2_backward_pin, motor_range_min);
  } else {
    digitalWrite(motor1_enable_pin, LOW);
    digitalWrite(motor2_enable_pin, LOW);
    Serial.println("No DPAD button pressed: Stopping both motors.");
    controlMotor(motor1_forward_pin, motor1_backward_pin, 0);
    controlMotor(motor2_forward_pin, motor2_backward_pin, 0);
  }

  
}

// Process all controllers
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Controller type not supported");
      }
    }
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());

  // Set up the motor control pins as outputs
  pinMode(motor1_forward_pin, OUTPUT);
  pinMode(motor1_backward_pin, OUTPUT);
  pinMode(motor2_forward_pin, OUTPUT);
  pinMode(motor2_backward_pin, OUTPUT);
  pinMode(motor1_enable_pin, OUTPUT);
  pinMode(motor2_enable_pin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  

  // Ensure motors are stopped initially
  digitalWrite(motor1_enable_pin, LOW);   // Add this line
  digitalWrite(motor2_enable_pin, LOW);   // Add this line
  analogWrite(motor1_forward_pin, 0);     // Add this line
  analogWrite(motor1_backward_pin, 0);    // Add this line
  analogWrite(motor2_forward_pin, 0);     // Add this line
  analogWrite(motor2_backward_pin, 0);    // Add this line


  // Initialize Bluepad32 and set callback functions
  BP32.setup(&onConnectedController, &onDisconnectedController);

  Serial.println("Setup completed. Waiting for controllers...");
}

// Main loop function
void loop() {
  // Update the gamepad data
  BP32.update();

  // Process connected controllers
  processControllers();
}