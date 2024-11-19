#include <Bluepad32.h>
#include <ESP32Servo.h>  // Use ESP32 Servo library

const int buzzer = 15;               // Buzzer pin
// Define the servo pins
// const int servo1_pin = 2;   // Servo 1 pin (left joystick up and down)
const int servo2_pin = 4;   // Servo 2 pin (left joystick left and right)
// const int servo3_pin = 5;   // Servo 3 pin (right joystick up and down)
const int servo4_pin = 18;  // Servo 4 pin (right joystick left and right)
// const int servo5_pin = 19;  // Servo 5 pin (throttle and brake buttons)

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
}

// Function to stop all motors and servos
void stopAllMotorsAndServos() {


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

}

// Function to perform the BUTTON(triangle) actio

// Function to perform the BUTTON_Y action
void performButtonYAction() {
  // Move bot forward by 8 cm and set servo angles simultaneously
  moveBotAndSetServos();
}


// Process gamepad input to control motors and servos
void processGamepad(ControllerPtr ctl) {
  

  // Check if BUTTON_Y is pressed
  if (ctl->y()) {
    Serial.println("BUTTON_Y pressed: Performing action.");
    performButtonYAction();
    return;
  }

  // Check if BUTTON_A is pressed

  // Check if BUTTON_B is pressed

  // Check the state of the D-pad buttons
  uint8_t dpadState = ctl->dpad();

  // Handle D-pad input for movement control
  

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
  pinMode(buzzer, OUTPUT);
  

  // Ensure motors are stopped initially
  // Attach the servo objects to their respective pins
  // servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  // servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
  // servo5.attach(servo5_pin);

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