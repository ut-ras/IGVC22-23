#include <ros.h>
#include <Wire.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
//#include <SabertoothSimplified.h>

#define BIAS 10
#define TA 8 // transmition address
#define P_S 6 // Pin select low for left high for right

const int receiverAddress = 8;



ros::NodeHandle nh;
//SabertoothSimplified ST;

void Send_To_Motors(int LEFT, int RIGHT);
void sendIntValue(int value, byte address);


void motor_cmd_str_callback(const std_msgs::String& msg)
{
  nh.loginfo("received command from ROS!"); 

  String cmd = String(msg.data);

  int cmd_left = cmd.substring(0, 2).toInt();
  int cmd_right = cmd.substring(2, 4).toInt();

  int left_sign = cmd.toInt() / 10 % 10;
  int right_sign = cmd.toInt() % 10; 

  if (left_sign == 1){
    cmd_left = -1 * cmd_left;
  }

  if (right_sign == 1){
    cmd_right = -1 * cmd_right; 
  }

    int ch3Value = cmd_left;
    int ch2Value = cmd_right; 

//    debugging:
    String val_str = String(ch3Value) + " " + String(ch2Value);
    nh.loginfo(val_str.c_str()); 
   // Send_To_Motors(ch3Value, ch2Value);

 }


ros::Subscriber<std_msgs::String> sub("/motor_cmd_str", &motor_cmd_str_callback);

void setup()
{
  //nh.initNode();
 // nh.subscribe(sub);
//  SabertoothTXPinSerial.begin(9600);
  Serial.begin(9600); 
  pinMode(P_S, OUTPUT);
   Wire.begin(); // join I2C bus (address optional for master)

//  ST.motor(1, 0);
//  ST.motor(2, 0);
}

void loop()
{
  Send_To_Motors(127, -127); // just testing
//  nh.spinOnce();
//  delay(100);
}

void Send_To_Motors(int LEFT, int RIGHT)
{
  int motor1_mapped_speed = map(LEFT, -127, 127, 0, 255);
  int motor2_mapped_speed = map(RIGHT, -127, 127, 0, 255);
  Wire.beginTransmission(8);
  Wire.write(motor1_mapped_speed);
  Wire.write(motor2_mapped_speed);
  Wire.endTransmission();
  delay(10);
}

