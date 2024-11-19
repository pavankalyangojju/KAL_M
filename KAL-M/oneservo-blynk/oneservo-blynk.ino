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

void setup()
{
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  servo1.attach(19); // Attach servo to GPIO 19
}

void loop()
{
  Blynk.run();
}

BLYNK_WRITE(V0) // Slider Widget for Servo 1 on V0
{
  int pos1 = param.asInt(); // Get value from slider
  servo1.write(pos1); // Set servo position
  Blynk.virtualWrite(V2, pos1);
}
