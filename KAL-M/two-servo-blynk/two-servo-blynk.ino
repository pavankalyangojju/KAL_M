#define BLYNK_TEMPLATE_ID "TMPL3gfj8Eu7S"
#define BLYNK_TEMPLATE_NAME "Temperature and Humidity Monitor"
#define BLYNK_AUTH_TOKEN "JutPlufxkORbnB4EGQKoT7UnjBk9Nsy5"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

char auth[] = BLYNK_AUTH_TOKEN; // Blynk authentication token
char ssid[] = "Gojju"; // Your WiFi SSID
char pass[] = "123456789"; // Your WiFi password

Servo servo1;
Servo servo2;

// Motor control pins
const int motor1_enable_pin = 13;  // Motor 1 enable pin
const int motor2_enable_pin = 25;  // Motor 2 enable pin
const int motor1_forward_pin = 12;   // Motor 1 forward pin
const int motor1_backward_pin = 14;  // Motor 1 backward pin
const int motor2_forward_pin = 27;   // Motor 2 forward pin
const int motor2_backward_pin = 26;  // Motor 2 backward pin

void setup()
{
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  
  // Attach servos
  servo1.attach(19); // Attach servo 1 to GPIO 19
  servo2.attach(18); // Attach servo 2 to GPIO 18
  
  // Set up motor pins
  pinMode(motor1_enable_pin, OUTPUT);
  pinMode(motor2_enable_pin, OUTPUT);
  pinMode(motor1_forward_pin, OUTPUT);
  pinMode(motor1_backward_pin, OUTPUT);
  pinMode(motor2_forward_pin, OUTPUT);
  pinMode(motor2_backward_pin, OUTPUT);
}

void loop()
{
  Blynk.run();
}

// Control Servo 1
BLYNK_WRITE(V0)
{
  int pos1 = param.asInt(); // Get value from slider
  servo1.write(pos1); // Set servo 1 position
}

// Control Servo 2
BLYNK_WRITE(V1)
{
  int pos2 = param.asInt(); // Get value from slider
  servo2.write(pos2); // Set servo 2 position
}

// Control Motor 1
BLYNK_WRITE(V5) // Button widget on V5
{
  int motor1Direction = param.asInt();
  if (motor1Direction == 1) { // Forward
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor1_forward_pin, HIGH);
    digitalWrite(motor1_backward_pin, LOW);
  } else if (motor1Direction == -1) { // Backward
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor1_forward_pin, LOW);
    digitalWrite(motor1_backward_pin, HIGH);
  } else { // Stop
    digitalWrite(motor1_enable_pin, LOW);
    digitalWrite(motor1_forward_pin, LOW);
    digitalWrite(motor1_backward_pin, LOW);
  }
}

// Control Motor 2
BLYNK_WRITE(V7) // Button widget on V6
{
  int motor2Direction = param.asInt();
  if (motor2Direction == 1) { // Forward
    digitalWrite(motor2_enable_pin, HIGH);
    digitalWrite(motor2_forward_pin, HIGH);
    digitalWrite(motor2_backward_pin, LOW);
  } else if (motor2Direction == -1) { // Backward
    digitalWrite(motor2_enable_pin, HIGH);
    digitalWrite(motor2_forward_pin, LOW);
    digitalWrite(motor2_backward_pin, HIGH);
  } else { // Stop
    digitalWrite(motor2_enable_pin, LOW);
    digitalWrite(motor2_forward_pin, LOW);
    digitalWrite(motor2_backward_pin, LOW);
  }
}
