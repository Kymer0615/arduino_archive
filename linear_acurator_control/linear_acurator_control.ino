#include <Arduino.h>
#include <TinyStepper.h>

#define HALFSTEPS 4096  // Number of half-steps for a full rotation


// depth motor
const int dirPin = 9;
const int stepPin = 8;
const int stepsPerRevolution = 200; // Adjust this to fit the number of steps per revolution for your motor
const int relayPin = 5; // Relay connected to digital pin 5

// For two tiny motors
// y rotation motor
TinyStepper z_stepper(HALFSTEPS, 10, 11, 12, 13);
TinyStepper y_stepper(HALFSTEPS, 2, 3, 4, 5);

void setup() {
  Serial.begin(38400);
  
//  Serial.begin(9600);
  y_stepper.Enable();
  z_stepper.Enable();
  delay(1000);

  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
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

void z_move(int steps, int direction) {
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














void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // m{step}_{dir}
    if (command.startsWith("m")) {
      int separatorIndex = command.indexOf('_');
      int steps = command.substring(1, separatorIndex).toInt();
      int direction = command.substring(separatorIndex + 1).toInt();
      z_move(steps, direction);
      Serial.println(command); 
    }
    
    if (command.startsWith("l")) {
      controlLaser(command); 
    }
    
    // ry{step}_s{speed}
    if (command.startsWith("ry")) {
      int separatorIndex = command.indexOf("_s");
      if (separatorIndex != -1) {
          String stepStr = command.substring(2, separatorIndex);
          String speedStr = command.substring(separatorIndex + 2);
          int steps = stepStr.toInt();
//          int speed = speedStr.toInt();
//          y_stepper.setSpeed(speed);
          y_stepper.Move(steps);
          Serial.println("Rotated y" + stepStr);
      }
    }
    
    if (command.startsWith("rz")) {
      int separatorIndex = command.indexOf("_s");
      if (separatorIndex != -1) {
          String stepStr = command.substring(2, separatorIndex);
          String speedStr = command.substring(separatorIndex + 2);
          int steps = stepStr.toInt();
//          int speed = speedStr.toInt();
//          z_stepper.setSpeed(speed);
          z_stepper.Move(steps);
//          Serial.println("Rotated z" + stepStr + " steps at speed " + speedStr);
          Serial.println("Rotated z" + stepStr);
      }
    }
  }
}
