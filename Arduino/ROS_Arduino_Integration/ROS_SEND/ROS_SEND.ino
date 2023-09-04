#include <ros.h>
#include <Wire.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#define BIAS 10
#define TA 8 // transmition address
#define P_S 6 // Pin select low for left high for right

const int receiverAddress = 8;

ros::NodeHandle nh;

void Send_To_Motors(int LEFT, int RIGHT);
void sendIntValue(int value, byte address);

int ch3Value = 0;
int ch2Value = 0;

void motor_cmd_str_callback(const std_msgs::String& msg)
{
  nh.loginfo("received command from ROS!"); 

  String cmd = String(msg.data);

  // extract left and right command info
  int cmd_left = cmd.substring(0, 2).toInt();
  int cmd_right = cmd.substring(2, 4).toInt();

  // get sign info from String for each wheel command 
  int left_sign = cmd.toInt() / 10 % 10;
  int right_sign = cmd.toInt() % 10; 

  // assign values after checking for sign 
  if (left_sign == 1){
    cmd_left = -1 * cmd_left;
  }

  if (right_sign == 1){
    cmd_right = -1 * cmd_right; 
  }

    // scale from -127 to 127 
     ch3Value = 1.27 * cmd_left;
     ch2Value = 1.27 * cmd_right; 

//    debugging: COMMENT OUT FOR FASTER PERFORMANCE
    String val_str = String(ch3Value) + " " + String(ch2Value);
    nh.loginfo(val_str.c_str()); 
//    Send_To_Motors(ch3Value, ch2Value);

 }

// Subscriper to ROS topic sending differential wheel velocity commands as string 
ros::Subscriber<std_msgs::String> sub("/motor_cmd_str", &motor_cmd_str_callback);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(9600); 
  pinMode(P_S, OUTPUT);
  Wire.begin(); // join I2C bus (address optional for master)

}

void loop()
{ 
  // continuously send current velocity commands to wheel-driving Arduino
  Send_To_Motors(ch3Value, ch2Value); 
  nh.spinOnce();
  delay(100);
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
