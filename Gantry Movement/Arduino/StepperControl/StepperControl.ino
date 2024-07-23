#include "AccelStepper.h"
#include <MultiStepper.h>

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56
#define Y_MIN_PIN 14
#define Y_MAX_PIN 15

AccelStepper stepper(1, X_STEP_PIN, X_DIR_PIN); // 1 = Driver
AccelStepper stepper2(1, Y_STEP_PIN, Y_DIR_PIN);
MultiStepper steppers;
long posx = 0;
long posy = 0;
void setup() {
  pinMode(X_MIN_PIN, INPUT);
  pinMode(X_MAX_PIN, INPUT);
  pinMode(Y_MIN_PIN, INPUT);
  pinMode(Y_MAX_PIN, INPUT);

  Serial.begin(9600);
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(300);

  stepper.setEnablePin(X_ENABLE_PIN);
  stepper.setPinsInverted(false, false, true); // Invert logic of enable pin
  stepper.enableOutputs();

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(300);

  stepper2.setEnablePin(Y_ENABLE_PIN);
  stepper2.setPinsInverted(false, false, true); // Invert logic of enable pin
  stepper2.enableOutputs();

  steppers.addStepper(stepper);
  steppers.addStepper(stepper2);

}

void loop() {
  
  if (Serial.available() > 0) {
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      long xPos = input.substring(0, commaIndex).toInt();
      long yPos = input.substring(commaIndex + 1).toInt();
      xPos = xPos * 80 ;
      yPos = yPos * 80;
      long delta1 = (xPos - posx) + (yPos - posy);
      long delta2 = (xPos - posx) - (yPos - posy);
      Serial.println(String(delta1) + "," + String(delta2));
      long positions[2]; // Array of desired stepper positions
      positions[0] = delta1 + stepper.currentPosition();
      positions[1] = delta2 + stepper2.currentPosition();
      
      steppers.moveTo(positions);
      steppers.runSpeedToPosition(); // Blocks until all are in position
      delay(1000);
      posx = xPos;
      posy = yPos;
      Serial.println("Endeffector at position: " + String(xPos) + " , " + String(yPos)      );
    } else {
      Serial.println("Invalid input. Please provide positions in the format: xPos,yPos");
    }
  }
}
void runToEndstop() {
  // Set the direction to move the steppers
  stepper.move(=100000); // Move in positive direction
  stepper2.move(100000); // Move in positive direction

  // Continuously run the steppers until an endstop is hit
  while (true) {
    stepper.run();
    stepper2.run();

    // Check if any endstop is triggered
    if (digitalRead(X_MIN_PIN) == LOW || digitalRead(X_MAX_PIN) == LOW || 
        digitalRead(Y_MIN_PIN) == LOW || digitalRead(Y_MAX_PIN) == LOW) {
      Serial.println("Endstop hit, stopping steppers");
      stepper.stop();
      stepper2.stop();
      break;
    }
  }
}

