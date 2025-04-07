#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// Defined pinout in .pio/libdeps/esp32dev/TFT_eSPI/User_Setups/Setup42_ILI9341_ESP32.h
#define TFT_MISO 19  // (leave TFT SDO disconnected if other SPI devices share MISO)
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_RST 4  // Reset pin (could connect to RST pin)
#define TFT_CS 15  // Chip select control pin
#define TFT_DC 2  // Data Command control pin

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI,
  TFT_SCLK, TFT_RST, TFT_MISO);

// Define state machine states
enum State { IDLE, RUNNING, STOPPED };
State currentState = IDLE;

// Pin definitions

const int motorPin = 5; // PWM for speed control
const int buttonPin = 4;
//const int speedPotPin = 21; // Potentiometer input

// Debounce variables
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// // Function to read potentiometer for speed control
// int getSpeed() {
//     int potValue = analogRead(speedPotPin); // Read from potentiometer (0-1023)
//     return map(potValue, 0, 1023, 50, 255); // Map to PWM range (50-255)
// }

// Function to control the motor
void motorRun(int speed) {
    analogWrite(motorPin, speed); // Apply PWM speed
}

// Function to stop the motor
void motorStop() {
    analogWrite(motorPin, 0); // Stop PWM signal
}

// Function to read button with debouncing
bool isButtonPressed() {
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) {
        delay(100);
        return true;
    }


    
    // if (reading != lastButtonState) {
    //     lastDebounceTime = millis();
    // }
    // if ((millis() - lastDebounceTime) > debounceDelay) {
    //     if (reading == LOW) { // Button pressed
    //         lastButtonState = HIGH;
    //         return true;
    //     }
    // }
    return false;
}

// Function to update the state machine
void updateState() {
    int speed = 0; 
    switch (currentState) {
        case IDLE:
            Serial.println("State: IDLE → Press button to start motor.");
            if (isButtonPressed()) {
                Serial.println("Transitioning to RUNNING state...");
                currentState = RUNNING;
            }
            break;

        case RUNNING:
           // speed = getSpeed(); // Read speed from potentiometer
            Serial.print("State: RUNNING → Speed: ");
            Serial.println(speed);
            motorRun(255);

            if (isButtonPressed()) {
                Serial.println("State: RUNNING → Transitioning to STOPPED");
                motorStop();
                currentState = STOPPED;
            }
            break;

        case STOPPED:
            if (isButtonPressed()) {
                Serial.println("State: STOPPED → Transitioning to RUNNING");
                currentState = RUNNING;
            }
            break;
    }
}

const int electromagnetPin = 17; // Pin for the electromagnet

void setup() {
    // currentState = IDLE;
    // Serial.begin(115200);
    // Serial.println("Begin");
  
    // pinMode(motorPin, OUTPUT);
    // pinMode(buttonPin, INPUT_PULLUP); // Internal pull-up enabled
    // //pinMode(speedPotPin, INPUT); // Potentiometer input

    // Serial.println("Starting in IDLE state...");
    pinMode(electromagnetPin, OUTPUT);
    Serial.begin(115200);
    Serial.println("Electromagnet Test");
    
}

void loop() {
      // Turn the MOSFET on (full power)
    digitalWrite(electromagnetPin, HIGH);
    Serial.println("Electromagnet ON");
    delay(5000); // Wait for 1 second
  // Turn the MOSFET off
    digitalWrite(electromagnetPin, LOW);
    Serial.println("Electromagnet OFF");
    delay(5000); // Wait for 1 second
}