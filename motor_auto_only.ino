#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <SabertoothSimplified.h>
#define BIAS 10

ros::NodeHandle nh;
SabertoothSimplified ST;


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
//    String val_str = String(ch3Value) + " " + String(ch2Value);
//    nh.loginfo(val_str.c_str()); 
    
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
    delay(100);
 }


ros::Subscriber<std_msgs::String> sub("/motor_cmd_str", &motor_cmd_str_callback);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  SabertoothTXPinSerial.begin(9600);
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void loop()
{
  nh.spinOnce();
  delay(100);
}
