#include <Arduino.h>

const int dirPin = 9;
const int stepPin = 8;
const int stepsPerRevolution = 200; // Adjust this to fit the number of steps per revolution for your motor
const int relayPin = 5; // Relay connected to digital pin 5

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  
  // Start the serial communication:
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("m")) {
      int separatorIndex = command.indexOf('_');
      int steps = command.substring(1, separatorIndex).toInt();
      int direction = command.substring(separatorIndex + 1).toInt();
      moveStepper(steps, direction);
      Serial.println(command); 
    }
    if (command.startsWith("l")) {
      controlLaser(command); 
    }
  }
}

void controlLaser(String command) {
  if (command == "l0") {
    digitalWrite(relayPin, HIGH); // Turn the laser on
    Serial.println("Laser ON");
  } else if (command == "l1") {
    digitalWrite(relayPin, LOW); // Turn the laser off
    Serial.println("Laser OFF");
  }
}
void moveStepper(int steps, int direction) {
  // Set the direction:
  if (direction == 1) {
    Serial.println("Forward");
    digitalWrite(dirPin, HIGH); // Forward
  } else {
    Serial.println("Backward");
    digitalWrite(dirPin, LOW); // Backward
  }

  // Make the stepper take the specified number of steps:
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // Adjust this delay to control the speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
}
