#include <Bluepad32.h>
#include <ESP32Servo.h>  // Use ESP32 Servo library
// Define the servo pins
const int servo5_pin = 19;  // Servo 5 pin (throttle and brake buttons)

// Initialize the controller array
ControllerPtr myControllers[BP32_MAX_GAMEPADS] = { nullptr };

// Initialize the Servo objects

Servo servo5;  // Servo 5 object for throttle and brake buttons

// Flag to track whether a controller is connected
bool controllerConnected = false;

// Function to control motors
void controlMotor(int forwardPin, int backwardPin, int speed) {
  Serial.printf("Controlling motor: forwardPin=%d, backwardPin=%d, speed=%d\n", forwardPin, backwardPin, speed);
}

// Function to stop all motors and servos
void stopAllMotorsAndServos() {
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
            // -edited

 

  // Define the movement duration and step size
  const int moveDuration = 400; // Adjust this duration to move 8 cm (depends on your bot's speed)
  const int stepDuration = 20;  // Time for each step in milliseconds
  const int steps = moveDuration / stepDuration;  // Number of steps for the movement

  // Calculate the angle increment for each step


  // Move the servos and motors simultaneously
  for (int i = 0; i < steps; i++) {
    // Increment servo angles
  

    // Delay for the step duration
    delay(stepDuration);
  }

}

// Function to perform the BUTTON(triangle) action


// Function to perform the BUTTON_Y action
void performButtonYAction() {
  // Move bot forward by 8 cm and set servo angles simultaneously
  moveBotAndSetServos();
}


// Function to perform the BUTTON_B(square) action

// Process gamepad input to control motors and servos
void processGamepad(ControllerPtr ctl) {
  // Check if BUTTON_X is pressed

  // Check if BUTTON_Y is pressed
  if (ctl->y()) {
    Serial.println("BUTTON_Y pressed: Performing action.");
    performButtonYAction();
    return;
  }

  // Check if BUTTON_A is pressed




  // Check the state of the D-pad buttons
  uint8_t dpadState = ctl->dpad();

  

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


  // Attach the servo objects to their respective pins

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