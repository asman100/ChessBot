#include <ros.h>                  // ROS library
#include <std_msgs/Int32MultiArray.h> 
int s0 = 10;
int s1 = 11;
int s2 = 12;
int s3 = 13;
// Mux in "SIG" pin
int SIG_pin = 8;

// Enable pins for the PCBs
int enablePins[4] = {A5, A4, A3, A2}; // PCB1, PCB2, PCB3, PCB4

// 8x8 array to represent the sensor data
int sensorArray[8][8];
ros::NodeHandle nh;

// Create a ROS publisher
std_msgs::Int32MultiArray chessboard_msg;
ros::Publisher chessboard_pub("chessboard", &chessboard_msg);
void setup() {
  nh.initNode();
  nh.advertise(chessboard_pub);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(SIG_pin, INPUT_PULLUP);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
 
  // Set up enable pins
  for (int i = 0; i < 4; i++) {
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH); // Default to HIGH (disable all PCBs)
  }
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);  
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  Serial.begin(115200);
  chessboard_msg.data_length = 64;  // 8x8 array = 64 values
  chessboard_msg.data = new int32_t[64];
}

void loop() {
  digitalWrite(2,LOW);
  delay(500);
  digitalWrite(2,HIGH);
  // Iterate through each board and populate the sensorArray
  for (int board = 1; board <= 4; board++) {
    selectBoard(board);

    // Populate the corresponding quadrant in the 8x8 array
    for (int channel = 0; channel < 16; channel++) {
      int val = readMux(channel);
      mapTo8x8Array(board, channel, val);
    }
  }

  // Print the 8x8 sensor array
  Serial.println("Sensor Array:");
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      Serial.print(sensorArray[row][col]);
      if (col < 7) {
        Serial.print(", ");
      }
      chessboard_msg.data[row * 8 + col] = sensorArray[row][col];
    }
    Serial.println();
  }
  chessboard_pub.publish(&chessboard_msg);
  
  // Spin the ROS node
  nh.spinOnce();

  
}

void selectBoard(int board) {
  // Set the selected board's enable pin to LOW and all others to HIGH
  for (int i = 0; i < 4; i++) {
    if (i == (board - 1)) {
      digitalWrite(enablePins[i], LOW);  // Enable the selected board
    } else {
      digitalWrite(enablePins[i], HIGH); // Disable the other boards
    }
  }
}

int readMux(int channel) {
  int controlPin[] = {s0, s1, s2, s3};

  // Mux channel selection array
  int muxChannel[16][4] = {
    {0, 0, 0, 0}, // Channel 0
    {1, 0, 0, 0}, // Channel 1
    {0, 1, 0, 0}, // Channel 2
    {1, 1, 0, 0}, // Channel 3
    {0, 0, 1, 0}, // Channel 4
    {1, 0, 1, 0}, // Channel 5
    {0, 1, 1, 0}, // Channel 6
    {1, 1, 1, 0}, // Channel 7
    {0, 0, 0, 1}, // Channel 8
    {1, 0, 0, 1}, // Channel 9
    {0, 1, 0, 1}, // Channel 10
    {1, 1, 0, 1}, // Channel 11
    {0, 0, 1, 1}, // Channel 12
    {1, 0, 1, 1}, // Channel 13
    {0, 1, 1, 1}, // Channel 14
    {1, 1, 1, 1}  // Channel 15
  };

  // Set the control pins for the selected channel
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  // Small delay to ensure the mux has settled before reading
  delay(5);

  // Read the value at the SIG pin
  int val = digitalRead(SIG_pin);
  val = !val;
  // Return the value
  return val;
}

void mapTo8x8Array(int board, int channel, int val) {
  int row = 0;
  int col = 0;

  // Mapping for Board 1 (Bottom Left)
  if (board == 1) {
    switch(channel) {
      case 11: row = 4; col = 0; break;
      case 15: row = 4; col = 1; break;
      case 4:  row = 4; col = 2; break;
      case 0:  row = 4; col = 3; break;
      case 10: row = 5; col = 0; break;
      case 14: row = 5; col = 1; break;
      case 5:  row = 5; col = 2; break;
      case 1:  row = 5; col = 3; break;
      case 9:  row = 6; col = 0; break;
      case 13: row = 6; col = 1; break;
      case 6:  row = 6; col = 2; break;
      case 2:  row = 6; col = 3; break;
      case 8:  row = 7; col = 0; break;
      case 12: row = 7; col = 1; break;
      case 7:  row = 7; col = 2; break;
      case 3:  row = 7; col = 3; break;
    }
  } 
  
  // Correct Mapping for Board 2 (Top Left - 180-degree rotation)
  else if (board == 2) {
    switch(channel) {
      case 3:  row = 0; col = 0; break;
      case 7:  row = 0; col = 1; break;
      case 12: row = 0; col = 2; break;
      case 8:  row = 0; col = 3; break;
      case 2:  row = 1; col = 0; break;
      case 6:  row = 1; col = 1; break;
      case 13: row = 1; col = 2; break;
      case 9:  row = 1; col = 3; break;
      case 1:  row = 2; col = 0; break;
      case 5:  row = 2; col = 1; break;
      case 14: row = 2; col = 2; break;
      case 10: row = 2; col = 3; break;
      case 0:  row = 3; col = 0; break;
      case 4:  row = 3; col = 1; break;
      case 15: row = 3; col = 2; break;
      case 11: row = 3; col = 3; break;
    }
  } 
  
  // Correct Mapping for Board 3 (Top Right - 180-degree rotation)
  else if (board == 3) {
    switch(channel) {
      case 3:  row = 0; col = 4; break;
      case 7:  row = 0; col = 5; break;
      case 12: row = 0; col = 6; break;
      case 8:  row = 0; col = 7; break;
      case 2:  row = 1; col = 4; break;
      case 6:  row = 1; col = 5; break;
      case 13: row = 1; col = 6; break;
      case 9:  row = 1; col = 7; break;
      case 1:  row = 2; col = 4; break;
      case 5:  row = 2; col = 5; break;
      case 14: row = 2; col = 6; break;
      case 10: row = 2; col = 7; break;
      case 0:  row = 3; col = 4; break;
      case 4:  row = 3; col = 5; break;
      case 15: row = 3; col = 6; break;
      case 11: row = 3; col = 7; break;
    }
  } 
  
  // Mapping for Board 4 (Bottom Right - Same as Board 1)
  else if (board == 4) {
    switch(channel) {
      case 11: row = 4; col = 4; break;
      case 15: row = 4; col = 5; break;
      case 4:  row = 4; col = 6; break;
      case 0:  row = 4; col = 7; break;
      case 10: row = 5; col = 4; break;
      case 14: row = 5; col = 5; break;
      case 5:  row = 5; col = 6; break;
      case 1:  row = 5; col = 7; break;
      case 9:  row = 6; col = 4; break;
      case 13: row = 6; col = 5; break;
      case 6:  row = 6; col = 6; break;
      case 2:  row = 6; col = 7; break;
      case 8:  row = 7; col = 4; break;
      case 12: row = 7; col = 5; break;
      case 7:  row = 7; col = 6; break;
      case 3:  row = 7; col = 7; break;
    }
  }

  // Store the value in the correct position in the 8x8 array
  sensorArray[row][col] = val;
}