#include "AccelStepper.h"
#include <MultiStepper.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38
#define X_MIN_PIN 3
#define X_MAX_PIN 2
#define LED_PIN            13
#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56
#define Y_MIN_PIN 14
#define Y_MAX_PIN 15
#define Fanpin 9
#define magnetpin 10

AccelStepper stepper(1, X_STEP_PIN, X_DIR_PIN); // 1 = Driver
AccelStepper stepper2(1, Y_STEP_PIN, Y_DIR_PIN);
MultiStepper steppers;
bool flagendstop = false;
long posx = 0;
long posy = 0;
long posxmm = 0;
long posymm = 0;

ros::NodeHandle nh;

std_msgs::String gantry_state_msg;
ros::Publisher gantry_state_pub("gantrystate", &gantry_state_msg);

// Function to handle magnet control from ROS
void magnetCallback(const std_msgs::Bool& msg) {
  if (msg.data) {
    digitalWrite(magnetpin, HIGH);
  } else {
    digitalWrite(magnetpin, LOW);
  }
}

ros::Subscriber<std_msgs::Bool> magnet_sub("magnet", &magnetCallback);

// Function to handle position control from ROS
void positionCallback(const std_msgs::String& msg) {
  String input = msg.data;
  int commaIndex = input.indexOf(',');
  if (commaIndex > 0) {
    long x = input.substring(0, commaIndex).toInt();
    long y = input.substring(commaIndex + 1).toInt();
    move(x, y);
  }
}

ros::Subscriber<std_msgs::String> position_sub("position", &positionCallback);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(X_MIN_PIN, INPUT);
  pinMode(X_MAX_PIN, INPUT);
  pinMode(Y_MIN_PIN, INPUT);
  pinMode(Y_MAX_PIN, INPUT);
  pinMode(Fanpin, OUTPUT);
  pinMode(magnetpin, OUTPUT);
  digitalWrite(Fanpin, HIGH);
  digitalWrite(magnetpin, LOW);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.begin(57600);
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(300);

  stepper.setEnablePin(X_ENABLE_PIN);
  stepper.setPinsInverted(false, false, true); // Invert logic of enable pin
  stepper.enableOutputs();

  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(300);

  stepper2.setEnablePin(Y_ENABLE_PIN);
  stepper2.setPinsInverted(false, false, true); // Invert logic of enable pin
  stepper2.enableOutputs();

  steppers.addStepper(stepper);
  steppers.addStepper(stepper2);

  nh.initNode();
  nh.subscribe(magnet_sub);
  nh.subscribe(position_sub);
  nh.advertise(gantry_state_pub);
  homing();
  move(10,200);

}

void loop() {
  nh.spinOnce();  // Handle incoming ROS messages
}

void move(long xPos, long yPos) {
  stepper.setMaxSpeed(8000);
  stepper.setAcceleration(5000);
  stepper2.setMaxSpeed(8000);
  stepper2.setAcceleration(5000);
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
  }

  posx = xPos;
  posy = yPos;
  long currentPos1 = stepper.currentPosition();
  long currentPos2 = stepper2.currentPosition();
  long actualDelta1 = (currentPos1 + currentPos2) / 2;
  long actualDelta2 = (currentPos1 - currentPos2) / 2;
  posxmm = actualDelta1 / 80;
  posymm = actualDelta2 / 80;
  
  Serial.println("Endeffector at position: " + String(posxmm) + " , " + String(posymm));

  // Publish gantry state as "done"
  gantry_state_msg.data = "Done";
  gantry_state_pub.publish(&gantry_state_msg);
}

void homing() {
  stepper.setSpeed(-2000);
  stepper2.setSpeed(2000);
  while (digitalRead(X_MIN_PIN) == HIGH) {
    stepper.runSpeed();
    stepper2.runSpeed();
  }
  stepper.stop();
  stepper2.stop();
  delay(1000);
  stepper.setSpeed(-1000);
  stepper2.setSpeed(-1000);
  while (digitalRead(Y_MIN_PIN) == HIGH) {
    stepper.runSpeed();
    stepper2.runSpeed();
  }
  stepper.stop();
  stepper2.stop();
  delay(1000);
  stepper.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  Serial.println("Homing Done");
  gantry_state_msg.data = "Homing Done";
  gantry_state_pub.publish(&gantry_state_msg);

}