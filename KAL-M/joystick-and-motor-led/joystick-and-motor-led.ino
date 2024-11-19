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

// Define the servo pins
const int servo1_pin = 2;   // Servo 1 pin (left joystick up and down)
const int servo2_pin = 4;   // Servo 2 pin (left joystick left and right)
const int servo3_pin = 5;   // Servo 3 pin (right joystick up and down)
const int servo4_pin = 18;  // Servo 4 pin (right joystick left and right)
const int servo5_pin = 19;  // Servo 5 pin (throttle and brake buttons)

// Initialize the controller array
ControllerPtr myControllers[BP32_MAX_GAMEPADS] = { nullptr };

// Initialize the Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;  // Servo 5 object for throttle and brake buttons

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

  // Stop all servos
  servo1.write(90);  // Neutral position
  servo2.write(90);  // Neutral position
  servo3.write(90);  // Neutral position
  servo4.write(90);  // Neutral position
  servo5.write(90);  // Neutral position
  Serial.println("All servos stopped.");
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

// Function to perform the BUTTON_X action

// Function to move the bot forward by 8 cm and set servo angles simultaneously
void moveBotAndSetServos() {
  // Define the servo target angles
  int targetServo1Angle = 170; // hand left
  int targetServo2Angle = 60; //shoulder
  int targetServo3Angle = 10; //right hand
  int targetServo4Angle = 120;  //right shoulder              // -edited

  // Define the initial servo angles
  int currentServo1Angle = servo1.read();
  int currentServo2Angle = servo2.read();
  int currentServo3Angle = servo3.read();
  int currentServo4Angle = servo4.read();

  // Define the movement duration and step size
  const int moveDuration = 400; // Adjust this duration to move 8 cm (depends on your bot's speed)
  const int stepDuration = 20;  // Time for each step in milliseconds
  const int steps = moveDuration / stepDuration;  // Number of steps for the movement

  // Calculate the angle increment for each step
  float servo1Increment = (targetServo1Angle - currentServo1Angle) / (float)steps;
  float servo2Increment = (targetServo2Angle - currentServo2Angle) / (float)steps;
  float servo3Increment = (targetServo3Angle - currentServo3Angle) / (float)steps;
  float servo4Increment = (targetServo4Angle - currentServo4Angle) / (float)steps;

  // Enable the motors and set speed
  const int motorSpeed = 255; // Set speed
  digitalWrite(motor1_enable_pin, HIGH);
  digitalWrite(motor2_enable_pin, HIGH);
  controlMotor(motor1_forward_pin, motor1_backward_pin, motorSpeed);
  controlMotor(motor2_forward_pin, motor2_backward_pin, motorSpeed);

  // Move the servos and motors simultaneously
  for (int i = 0; i < steps; i++) {
    // Increment servo angles
    currentServo1Angle += servo1Increment;
    currentServo2Angle += servo2Increment;
    currentServo3Angle += servo3Increment;
    currentServo4Angle += servo4Increment;

    // Write new servo angles
    servo1.write(currentServo1Angle);
    servo2.write(currentServo2Angle);
    servo3.write(currentServo3Angle);
    servo4.write(currentServo4Angle);

    // Delay for the step duration
    delay(stepDuration);
  }

  // Stop the motors after moving forward
  controlMotor(motor1_forward_pin, motor1_backward_pin, 0);
  controlMotor(motor2_forward_pin, motor2_backward_pin, 0);
  digitalWrite(motor1_enable_pin, LOW);
  digitalWrite(motor2_enable_pin, LOW);
}

// Function to perform the BUTTON(triangle) action
void performButtonXAction() {
  // Move forward by 8 cm (assume 2 cm per unit speed)
  digitalWrite(motor1_enable_pin, HIGH);
  digitalWrite(motor2_enable_pin, HIGH);
  controlMotor(motor1_forward_pin, motor1_backward_pin, motor_range_max / 8);
  controlMotor(motor2_forward_pin, motor2_backward_pin, motor_range_max / 8);
  delay(400); // Adjust the delay to achieve the correct 8 cm movement
  controlMotor(motor1_forward_pin, motor1_backward_pin, 0);
  controlMotor(motor2_forward_pin, motor2_backward_pin, 0);
  digitalWrite(motor1_enable_pin, LOW);
  digitalWrite(motor2_enable_pin, LOW);

  // Set servo angles
  servo1.write(170);  // left hand at 45 degrees
  servo2.write(40);   // right shoulder at 45 degrees
  servo3.write(10);   // left hand at 135 degrees
  servo4.write(140);  // right shoulder at 135 degrees -edited
  delay(1500);
}

// Function to perform the BUTTON_Y action
void performButtonYAction() {
  // Move bot forward by 8 cm and set servo angles simultaneously
  moveBotAndSetServos();
}

// Function to perform the BUTTON_A(circle) action
void performButtonAAction() {
  // Set servo angles for a Mortal Kombat position
  servo1.write(90);  // Right hand at 60 degrees
  servo2.write(90); // Left hand at 120 degrees
  servo3.write(90); // Right shoulder at 180 degrees
  servo4.write(90);  // Left shoulder at 90 degrees
  servo5.write(45);  // Move hip 45 degrees left
  servo2.write(40); // Right hand at 60 degrees
  servo1.write(180); // Left hand at 120 degrees
  delay(400);
  servo1.write(90);  // Right hand at 60 degrees
  servo2.write(90); // Left hand at 120 degrees
  servo3.write(90); // Right shoulder at 180 degrees
  servo4.write(90);  // Left shoulder at 90 degrees
  servo5.write(90);  // Move hip 45 degrees left
  delay(100);
  servo5.write(135);
  servo3.write(0); // Right shoulder at 180 degrees
  servo4.write(130);
  delay(400);
  servo1.write(90);  // Right hand at 60 degrees
  servo2.write(90); // Left hand at 120 degrees
  servo3.write(90); // Right shoulder at 180 degrees
  servo4.write(90);  // Left shoulder at 90 degrees
  servo5.write(90);
  delay(100);
  servo5.write(45);
  servo2.write(40); // Right hand at 60 degrees
  servo1.write(180); // Left hand at 120 degrees
  delay(400);
  servo1.write(90);  // Right hand at 60 degrees
  servo2.write(90); // Left hand at 120 degrees
  servo3.write(90); // Right shoulder at 180 degrees
  servo4.write(90);  // Left shoulder at 90 degrees
  servo5.write(90);  // Move hip 45 degrees left
  delay(100);
  servo5.write(135);
  servo3.write(0); // Right shoulder at 180 degrees
  servo4.write(130);
  delay(400);
}

// Function to perform the BUTTON_B(square) action
void performButtonBAction() {
  // john cena
  servo1.write(90);  // Right hand at 60 degrees
  servo2.write(90); // Left hand at 120 degrees
  servo3.write(90); // Right shoulder at 180 degrees
  servo4.write(90);  // Left shoulder at 90 degrees
  servo5.write(45);  // Move hip 45 degrees left
  servo2.write(40);
  delay(800);  // Right hand at 60 degrees
  servo1.write(180); // Left hand at 120 degrees
  delay(400);
  servo1.write(140); // Right hand at 60 degrees
  delay(400);
  servo1.write(180); // Left hand at 120 degrees
  delay(400);
  servo1.write(140); // Right hand at 60 degrees
  delay(400);
}

// Process gamepad input to control motors and servos
void processGamepad(ControllerPtr ctl) {
  // Check if BUTTON_X is pressed
  if (ctl->a()) {
    Serial.println("BUTTON_X pressed: Performing action.");
    performButtonXAction();
    return;
  }

  // Check if BUTTON_Y is pressed
  if (ctl->y()) {
    Serial.println("BUTTON_Y pressed: Performing action.");
    performButtonYAction();
    return;
  }

  // Check if BUTTON_A is pressed
  if (ctl->b()) {
    Serial.println("BUTTON_A pressed: Performing action.");
    performButtonAAction();
    return;
  }

  // Check if BUTTON_B is pressed
  if (ctl->x()) {
    Serial.println("BUTTON_B pressed: Performing action.");
    performButtonBAction();
    return;
  }

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

  // Read the left joystick axes
  int left_axis_x = ctl->axisX();
  int left_axis_y = ctl->axisY();

  // Read the right joystick axes
  int right_axis_x = ctl->axisRX();
  int right_axis_y = ctl->axisRY();

  // Map the joystick values to servo angle ranges
  int servo1_angle = map(left_axis_y, 511, -512, 0, 180);
  int servo2_angle = map(left_axis_x, 511, -512, 0, 180);
  int servo3_angle = map(right_axis_y, -511, 512, 0, 180);
  int servo4_angle = map(right_axis_x, 511, -512, 0, 180);

  // Control servos based on joystick inputs
  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
  servo3.write(servo3_angle);
  servo4.write(servo4_angle);

  // Control Servo 5 using throttle and brake buttons
  if (ctl->brake()) {
    servo5.write(180);
    Serial.println("Brake button pressed: Servo5 set to 0 degrees.");
  } else if (ctl->throttle()) {
    servo5.write(0);
    Serial.println("Throttle button pressed: Servo5 set to 180 degrees.");
  } else {
    servo5.write(90);
  }

  // Print battery status
  printBatteryStatus(ctl);
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
  // Attach the servo objects to their respective pins
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
  servo5.attach(servo5_pin);

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