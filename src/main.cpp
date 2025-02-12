#include <Arduino.h>

// Define state machine states
enum State { IDLE, RUNNING, STOPPED };
State currentState = IDLE;

// Pin definitions

const int motorPin = 5; // PWM for speed control
const int buttonPin = 4;
const int speedPotPin = 21; // Potentiometer input

// Debounce variables
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Function to read potentiometer for speed control
int getSpeed() {
    int potValue = analogRead(speedPotPin); // Read from potentiometer (0-1023)
    return map(potValue, 0, 1023, 50, 255); // Map to PWM range (50-255)
}

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
            speed = getSpeed(); // Read speed from potentiometer
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

void setup() {
    currentState = IDLE;
    Serial.begin(115200);
    Serial.println("Begin");
  
    pinMode(motorPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP); // Internal pull-up enabled
    pinMode(speedPotPin, INPUT); // Potentiometer input

    Serial.println("Starting in IDLE state...");
}

void loop() {
    updateState();


    delay(100); 
    //Serial.println(digitalRead(buttonPin));
}