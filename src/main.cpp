#include <Arduino.h>

// You can declare functions at the top!

// Pin number for the LED
const int ledPin = 13; // Change this if your LED is connected to a different pin

void setup() {
  Serial.begin(115200);
  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Turn the LED on
  digitalWrite(ledPin, HIGH);
  delay(1000); // Wait for 1 second
  Serial.println("LED is on\n");

  // Turn the LED off
  digitalWrite(ledPin, LOW);
  delay(1000); // Wait for 1 second
  Serial.println("LED is on\n");

  Serial.println("Hello World!\n");

}

// and then put the function definitions later!