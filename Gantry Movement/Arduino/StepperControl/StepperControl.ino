#include "AccelStepper.h"
#include <MultiStepper.h>

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38
#define X_MIN_PIN 3
#define X_MAX_PIN 2

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
long posxmm = 0;
long posymm = 0;

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
  homing();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      long x = input.substring(0, commaIndex).toInt();
      long y = input.substring(commaIndex + 1).toInt();
      move(x, y);
    }
  }
}

void move(long xPos, long yPos) {
  xPos *= 80;
  yPos *= 80;
  long delta1 = (xPos - posx) + (yPos - posy);
  long delta2 = (xPos - posx) - (yPos - posy);
  Serial.println(String(delta1) + "," + String(delta2));
  
  long positions[2];
  positions[0] = delta1 + stepper.currentPosition();
  positions[1] = delta2 + stepper2.currentPosition();
  
  steppers.moveTo(positions);
  
  while ((stepper.distanceToGo() != 0 || stepper2.distanceToGo() != 0)) {
    steppers.run();
    // if ((digitalRead(X_MIN_PIN) == LOW) && !flagendstop ){
    //   stepper.stop();
    //   stepper2.stop();
    //   delay(100);
    //   flagendstop = true;
    //   Serial.println("X min Endstop triggered, stopping movement.");
    //   break;
    // }
  }
  
  // if (flagendstop) {
  //   long currentPos1 = stepper.currentPosition();
  //   long currentPos2 = stepper2.currentPosition();
  //   long actualDelta1 = (currentPos1 + currentPos2) / 2;
  //   long actualDelta2 = (currentPos1 - currentPos2) / 2;
  //   posxmm = actualDelta1 / 80;
  //   posymm = actualDelta2 / 80;
  //   posx = stepper.currentPosition();
  //   posy = stepper2.currentPosition();
  //   Serial.println(posymm);
  //   // Move away from the endstop
  //   if (digitalRead(X_MIN_PIN) == LOW) {
  //     move(0,10);
  //   }
  //   posx = stepper.currentPosition();
  //   posy = stepper2.currentPosition();
    
  //   // Reset the flag after moving away
  //   flagendstop = false;
  // } else {
  //   posx = xPos;
  //   posy = yPos;
  //   long currentPos1 = stepper.currentPosition();
  //   long currentPos2 = stepper2.currentPosition();
  //   long actualDelta1 = (currentPos1 + currentPos2) / 2;
  //   long actualDelta2 = (currentPos1 - currentPos2) / 2;
  //   posxmm = actualDelta1 / 80;
  //   posymm = actualDelta2 / 80;
  // }
  
  Serial.println("Endeffector at position: " + String(posxmm) + " , " + String(posymm));
}
void homing (){
  stepper.setSpeed(1000);
  stepper2.setSpeed(-1000);
  while (digitalRead(X_MIN_PIN) == HIGH){
    stepper.runSpeed();
    stepper2.runSpeed();
  }
  stepper.setSpeed(-1000);
  stepper2.setSpeed(-1000);
  while (digitalRead(Y_MIN_PIN) == HIGH){
    stepper.runSpeed();
    stepper2.runSpeed();
  }
  stepper.setCurrentPosition(0);
}

