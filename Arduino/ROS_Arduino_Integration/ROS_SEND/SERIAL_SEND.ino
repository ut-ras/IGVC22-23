/** 
FUNCTIONALITY:

THIS SCRIPT OPERATES ON ARDUINO #1 (connected to PC over serial and Arduino #2 over I2C)

This script receives data from the robot PC over serial
The received data can be interpreted as (left_motor_cmd, right_motor_cmd)
The script transfers this info to Arduino #2 over I2C
 **/

#include <Wire.h>


#define TA 8 // transmition address
#define P_S 6 // Pin select low for left high for right


void Send_To_Motors(int LEFT, int RIGHT);                 // sends ROS commands to motors thru slave device (arduino)

int ch3Value = 0;                                         // left motor value
int ch2Value = 0;                                         // right motor value 
int receivedInteger;


void setup()
{
  Serial.begin(115200);
  pinMode(P_S, OUTPUT);
  Wire.begin(); // join I2C bus (address optional for master)

}

void loop()
{
  // continuously send current velocity commands to wheel-driving Arduino
  Send_To_Motors(ch3Value, ch2Value);
  delay(100);

  if (Serial.available() >= 8) {  // Adjust the number of bytes based on the data size
    int32_t received_data[2];

    // Read the binary data into an array of 32-bit integers
    Serial.readBytes((char*)received_data, 8);

    // Unpack and print the received integers
    // Serial.print("Received Integers: ");    // debug only
    cmd_left = received_data[0]                      
    cmd_right = received_data[1]

    // scale values to [-127, 127]
    ch3Value = 1.27 * cmd_left;     
    ch2Value = 1.27 * cmd_right;

    // Serial.print("Left velocity: "+" "+cmd_left+ "  "+"Right velocity: "+" "+cmd_right)   // debugging only

    Send_To_Motors(ch3Value, ch2Value);
  }
}


void Send_To_Motors(int LEFT, int RIGHT)
{
  // Map [-127, 127] to [0, 255], as required by Wire  (encode)
  int motor1_mapped_speed = map(LEFT, -127, 127, 0, 255);
  int motor2_mapped_speed = map(RIGHT, -127, 127, 0, 255);
  
  Wire.beginTransmission(TA);
  
  Wire.write(motor1_mapped_speed);
  Wire.write(motor2_mapped_speed);
  Wire.endTransmission();
  
  delay(10);
}
