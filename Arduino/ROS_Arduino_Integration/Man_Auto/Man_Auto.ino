/** 
FUNCTIONALITY:

THIS SCRIPT OPERATES ON ARDUINO #2 (connected to Arduino #1 over I2C and Sabertooth motor driver over serial and receiver)

This script reads the value of a switch on the controller 
Depending on the state of the switch, we determine manual vs. autonomous control 

Case 1: Manual Control
- Continue to receive data over I2C and update global var, but ignore the received motor commands
- If the emergency stop switch on the controller is not toggled:
  -- read the value of the joysticks on the controller 
  -- send values to motor driver over serial 
- Else:
  - send 0 values (i.e. stop) to motor driver

Case 2: Autonomous Control
- Use values received ovr I2C 
- If the emergency stop switch on the controller is not toggled:
  -- use the current value of the global variable (see code) and send to motor driver over serial 
  -- send values to motor driver over serial 
- Else:
  - send 0 values (i.e. stop) to motor driver

 **/

#include <SabertoothSimplified.h>       // Sabertooth library 
#include <Wire.h>

SabertoothSimplified ST;

// Joysticks
#define CH2 5
#define CH3 6
// Switches, could select switches through the RC Settings menu. 
#define CH5 9 // SWA
#define CH6 11 // SWD

#define BIAS 10 // Noise bias rejection  
#define P_S_R 2 // pin select receive 0 for left vcc rail for right
#define BUFFER_SIZE 10

// custom struct to store autonomous command values for [left, right] motors 
typedef struct RECEIVER
{
  int LEFT = 0;
  int RIGHT = 0;

}RECEIVER;

/*
FIFO implementation for smoothing motor commands. Improves drive controllability & performance.
*/
class MotorFIFO {
private:
  int leftMotorBuffer[BUFFER_SIZE];
  int rightMotorBuffer[BUFFER_SIZE];
  int currentIndex = 0;
  int itemCount = 0;

public:
  MotorFIFO() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
      leftMotorBuffer[i] = 0;
      rightMotorBuffer[i] = 0;
    }
  }

  void insertVelocities(int leftVelocity, int rightVelocity) {
    if (itemCount < BUFFER_SIZE) {
      leftMotorBuffer[currentIndex] = leftVelocity;
      rightMotorBuffer[currentIndex] = rightVelocity;
      currentIndex = (currentIndex + 1) % BUFFER_SIZE;
      itemCount++;
    } else {
      // Buffer is full, shift the data
      for (int i = 1; i < BUFFER_SIZE; i++) {
        leftMotorBuffer[i - 1] = leftMotorBuffer[i];
        rightMotorBuffer[i - 1] = rightMotorBuffer[i];
      }
      leftMotorBuffer[BUFFER_SIZE - 1] = leftVelocity;
      rightMotorBuffer[BUFFER_SIZE - 1] = rightVelocity;
    }
  }

  float getLeftMotorAverage() {
    int sum = 0;

    for (int i = 0; i < itemCount; i++) {
      sum += leftMotorBuffer[i];
    }

    return static_cast<float>(sum) / itemCount;
  }

  float getRightMotorAverage() {
    int sum = 0;

    for (int i = 0; i < itemCount; i++) {
      sum += rightMotorBuffer[i];
    }

    return static_cast<float>(sum) / itemCount;
  }
};


RECEIVER R_V; // received values; global variable to store the received autonomous motor commands 
MotorFIFO S_MOTOR; // Smoothing motor
int ch2Value = 0;
int ch3Value = 0;
bool ch5Value = 0;
bool ch6Value = 0;
const int receiverAddress = 8;

int motor1_speed = 0;
int motor2_speed = 0;

// ROS values
int ROS_L = 0;
int ROS_R = 0;


int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue); // reads values from inputted channel and sets it within a range
bool readSwitch(byte channelInput, bool defaultValue); // checks whether the robot is controlled manually or through ROS
void Send_To_Motors(int LEFT, int RIGHT); // sends values to motors using FIFO buffer
void receiveEvent(int byteCount);         // event handler for incoming ros commands from the master arduino

void setup(){
 SabertoothTXPinSerial.begin(9600);      // begin serial comms with sabertooth motor driver 
 
  Wire.begin(receiverAddress);  // join I2C bus with address 8
  Wire.onReceive(receiveEvent); // register receive event handler

  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void loop() {
  // read switch & channel values (functions defined below)
  ch2Value = (int) readChannel(CH2, -126, 126, 0); // right joy stick
  ch3Value = (int) readChannel(CH3, -126, 126, 0); // left joy stick
  ch5Value = readSwitch(CH5, false);
  ch6Value = readSwitch(CH6, false);
  
  // Output the values to the Serial Monitor (debug)
  // Serial.print("\n ");
 /*
 Serial.print("Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" | Ch3: ");
  Serial.print(ch3Value);
    Serial.print(" | Ch5: ");
  Serial.print(ch5Value);
  Serial.print(" | Ch6: ");
  Serial.print(ch6Value);
  Serial.println();

*/
 
if(!ch5Value) // Emergency switch NOT toggled
{
  if(!ch6Value) // Manual mode
  {
     Send_To_Motors(ch3Value,ch2Value);
  }
  else // Autonomous mode 
  {
  //  Serial.println("LM: " + (String) R_V.LEFT + "RM: " + (String) R_V.RIGHT);
    Send_To_Motors(R_V.LEFT,R_V.RIGHT);
  }
}
else  // Emergency switch toggled ---- STOP motors
{
    ST.motor(1, 0);
    ST.motor(2, 0);
    S_MOTOR.insertVelocities(0,0); // Fill buffer with 0, for a full stop when regaining manual control
}
 
}

// read channel and return mapped values in range [minLimit, maxLimit]
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;                     // should this be 1000
  return (int) map(ch, 1000, 2000, minLimit, maxLimit);
}


bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}


// send commands to motor over serial
void Send_To_Motors(int LEFT, int RIGHT)
{
      S_MOTOR.insertVelocities(LEFT,RIGHT);
       // Serial.println("Wrting to motor ...");
       // Drive the motors based on the channel values
      if (abs(S_MOTOR.getLeftMotorAverage()) >= BIAS) {
        ST.motor(1, (int) S_MOTOR.getLeftMotorAverage());
      //  Serial.print(" | Motor 1 power: ");              // PRINT STATEMENTS FOR DEBUG ONLY (comment out for real operation!!!)
      //  Serial.print(ch3Value);
      } else {
        ST.motor(1, 0);
     //   Serial.print(" | Motor 1 power: 0");
      }
    
      if (abs(S_MOTOR.getRightMotorAverage()) >= BIAS) {
        ST.motor(2, (int) S_MOTOR.getRightMotorAverage());
      //  Serial.print(" | Motor 2 power: ");
       // Serial.print(ch2Value);
      } else {
        ST.motor(2, 0);
    //    Serial.print(" | Motor 2 power: 0");
      }
    //  delay(1);
    //  Serial.println(); // Print a new line to improve readability 
}

// Receive I2C data from Arduino #1 and update global variable 
void receiveEvent(int howMany) { // I2C communication to receive ROS Velocity vectors
  // read the motor speeds from the master
  if (howMany >= 2) {
    motor1_speed = Wire.read();    // receive byte as an integer
    motor2_speed = Wire.read();    // receive byte as an integer
  }

  // map the speeds back to range -100 to 100
  int motor1_mapped_speed = map(motor1_speed, 0, 255, -127, 127);
  int motor2_mapped_speed = map(motor2_speed, 0, 255, -127, 127);

  R_V.LEFT = motor1_mapped_speed;
  R_V.RIGHT = motor2_mapped_speed;
}
// this function is registered as an event, see setup()
