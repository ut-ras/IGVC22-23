#include <SabertoothSimplified.h>

/*
  Arduino FS-I6X Demo
  fsi6x-arduino-uno.ino
  Read output ports from FS-IA6B receiver module
  Display values on Serial Monitor
  
  Channel functions by Ricardo Paiva - https://gist.github.com/werneckpaiva/
  
  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

SabertoothSimplified ST;

// Define Input Connections
#define CH1 3
#define CH2 5
#define CH3 6
#define CH4 9
#define CH5 10
#define CH6 11
 
// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
int ch3Value;
int ch4Value; 
int ch5Value;
 
// Boolean to represent switch value
bool ch6Value;
 
// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 
// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
 
void setup(){
  // Set up serial monitor
  // Serial.begin(115200);
  SabertoothTXPinSerial.begin(9600); // this is baud rate, make sure you set up with motor driver correctly
  //make sure you change the baud (115200) in the serial monitor GUI too or you'll get alien language output
  
  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
}
 
 
void loop() {
  
  // Get values for each channel
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -100, 100, -100);
  ch4Value = readChannel(CH4, -100, 100, 0);
  ch5Value = readChannel(CH5, -100, 100, 0);
  ch6Value = readSwitch(CH6, false);
  
  // Print to Serial Monitor
  // Serial.print("Ch1: ");
  // Serial.print(ch1Value);
  // Serial.print(" | Ch2: ");
  // Serial.print(ch2Value);
  // Serial.print(" | Ch3: ");
  // Serial.print(ch3Value);
  // Serial.print(" | Ch4: ");
  // Serial.print(ch4Value);
  // Serial.print(" | Ch5: ");
  // Serial.print(ch5Value);
  // Serial.print(" | Ch6: ");
  // Serial.println(ch6Value);

  //Send and print motor power
  // if (abs(ch3Value) >= 30) {
  //   ST.motor(1, ch3Value);
  //   Serial.print("Motor 1 power: ");
  //   Serial.print(ch3Value);
  // }
  // else {
  //   ST.motor(1, 0);
  //   Serial.print("Motor 1 power: 0");
  // }
  // if (abs(ch2Value) > 3) {
  //   ST.motor(2, ch2Value);
  //   Serial.print("Motor Power:");
  //   Serial.print(ch2Value);
  //   Serial.println("");
  // }

  // """This sets it so the the left toggle left and right is turn, and right toggle is drive. You must drive before turning"""

  //forward and back
  if (abs(ch2Value) > 3) {
    ST.drive(ch2Value);
    // Serial.print("Drive Power: ");
    // Serial.print(ch2Value);
    // Serial.print(", ");
  }

  else {
    ST.drive(0);  
    // Serial.print("Drive power: 0");
    // Serial.print(", ");
  }

  //turn
  if (abs(ch4Value) > 3) {
    ST.turn(ch4Value);
    // Serial.print("Turn Power: ");
    // Serial.print(ch4Value);
    // Serial.println();
  }

  else {
    ST.turn(0);  
    // Serial.print("Turn Power: 0");
    // Serial.println();
  }
  
  delay(50);
}