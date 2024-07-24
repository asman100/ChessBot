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
bool flagendstop = false;
long posx = 0;
long posy = 0;
long xmax = 0;
long ymax = 0;
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
  //runToEndstop();
}

void loop() {
  
  if (Serial.available() > 0) {
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      long x = input.substring(0, commaIndex).toInt();
      long y = input.substring(commaIndex + 1).toInt();
      move(x,y);
    
      
}}}

void move(long xPos, long yPos) {
    xPos = xPos * 80;
    yPos = yPos * 80;
    long delta1 = (xPos - posx) + (yPos - posy);
    long delta2 = (xPos - posx) - (yPos - posy);
    Serial.println(String(delta1) + "," + String(delta2));
    
    long positions[2]; // Array of desired stepper positions
    positions[0] = delta1 + stepper.currentPosition();
    positions[1] = delta2 + stepper2.currentPosition();
    
    steppers.moveTo(positions);
    
    // Run the steppers until they reach the target position
    while (stepper.distanceToGo() != 0 || stepper2.distanceToGo() != 0)  {
        steppers.run(); // Non-blocking call to step the motors
        if (digitalRead(X_MIN_PIN) == LOW || 
            digitalRead(X_MAX_PIN) == LOW || 
            digitalRead(Y_MIN_PIN) == LOW || 
            digitalRead(Y_MAX_PIN) == LOW) {
            stepper.stop();
            stepper2.stop();
            flagendstop = true;
            Serial.println("Endstop triggered, stopping movement.");
            break;
    }}
    if (flagendstop){
      long currentPos1 = stepper.currentPosition();
      long currentPos2 = stepper2.currentPosition();
      long actualDelta1 = (currentPos1 + currentPos2) / 2;
      long actualDelta2 = (currentPos1 - currentPos2) / 2;
      posx = actualDelta1 / 80;
      posy = actualDelta2 / 80;
    }
    // do an if endstop triggered with flag, then update pos with inverse math
    else{ 
    posx = xPos;
    posy = yPos;}
    Serial.println("Endeffector at position: " + String(posx) + " , " + String(posy)); 
}


void runToEndstop() {
  // Set the direction to move the steppers
  long pos[2];
  pos[0] = -100000;
  pos[1] = 100000;
  steppers.moveTo(pos); // Move in positive direction

  // Continuously run the steppers until an endstop is hit
  while (true) {
    steppers.run();

    // Check if any endstop is triggered
    if (digitalRead(Y_MIN_PIN) == LOW) {
      Serial.println("Endstop hit, stopping steppers");
      stepper.stop();
      stepper2.stop();
      break;
    }
  }
  pos[0] = 0;
  pos[1] = 100000;
  steppers.moveTo(pos); 
  while (true) {
    steppers.run();

    // Check if any endstop is triggered
    if (digitalRead(X_MIN_PIN) == LOW) {
      Serial.println("Endstop hit, stopping steppers");
      stepper.stop();
      stepper2.stop();
      break;
    }
  }
  stepper.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  pos[0] = 0;
  pos[1] = -100000;
  steppers.moveTo(pos); 
  while (true) {
    steppers.run();

    // Check if any endstop is triggered
    if (digitalRead(X_MAX_PIN) == LOW) {
      Serial.println("Endstop hit, stopping steppers");
      stepper.stop();
      stepper2.stop();
      xmax = stepper2.currentPosition();
      break;
    }
  }
  
}





