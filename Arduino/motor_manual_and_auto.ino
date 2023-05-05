/*
  Arduino FS-I6X Demo
  fsi6x-arduino-mega-ibus.ino
  Read iBus output port from FS-IA6B receiver module
  Display values on Serial Monitor

  Channel functions by Ricardo Paiva - https://gist.github.com/werneckpaiva/

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/
#include <ros.h>
#include <std_msgs/String.h>
#include <SabertoothSimplified.h>



#define CH2 5
#define CH3 6
#define CH6 11
#define BIAS 10

ros::NodeHandle nh;
SabertoothSimplified ST;


bool readSwitch(byte channelInput, bool defaultValue); 
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);

int ch2Value = 0;
int ch3Value = 0;
int ch3Value_ros = 0;
int ch2Value_ros = 0; 
bool ch6Value = false;

void motor_cmd_str_callback(const std_msgs::String& msg){

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
  
    ch3Value_ros = cmd_left;
    ch2Value_ros = cmd_right; 
        
}



// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return (int) map(ch, 1000, 2000, minLimit, maxLimit);
}

bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}


ros::Subscriber<std_msgs::String> sub("/motor_cmd_str", &motor_cmd_str_callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  SabertoothTXPinSerial.begin(9600);
  ST.motor(1, 0);
  ST.motor(2, 0);
}


void loop() {
  nh.spinOnce();
  delay(100);

  ch6Value = readSwitch(CH6, false);
//  ch6Value = false; 
  int mode = (ch6Value) ? 1 : 0;

   if (mode == 1){
    
    if (abs(ch3Value_ros) >= BIAS) {
        ST.motor(1, ch3Value_ros);
        } else {
        ST.motor(1, 0);
      }
      
      if (abs(ch2Value_ros) >= BIAS) {
        ST.motor(2, ch2Value_ros);
        } else {
        ST.motor(2, 0);
      }

//     debugging:
//     String val_str = "AUTO: " + String(ch3Value_ros) + " " + String(ch2Value_ros);
//     nh.loginfo(val_str.c_str()); 
   }

  else {
      ch2Value = (int) readChannel(CH2, -126, 126, 0);
      ch3Value = (int) readChannel(CH3, -126, 126, 0);
//      ch2Value = (int) 80;
//      ch3Value = (int) 75;
    if (abs(ch3Value) >= BIAS) {
      ST.motor(1, ch3Value);
    } else {
      ST.motor(1, 0);
    }
    if (abs(ch2Value) >= BIAS) {
    ST.motor(2, ch2Value);
    } else {
    ST.motor(2, 0);
    }

//    debugging:
//    String val_str = "MANUAL: " + String(ch3Value) + " " + String(ch2Value);
//    nh.loginfo(val_str.c_str()); 
  }

      
}
