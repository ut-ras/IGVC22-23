#include <SabertoothSimplified.h>

SabertoothSimplified ST;

#define CH2 5
#define CH3 6
#define CH6 11
#define BIAS 20

int ch2Value = 0;
int ch3Value = 0;
bool ch6Value = 0;
String myCmd;

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

void setup(){
   SabertoothTXPinSerial.begin(9600);
//  Serial.begin(115200);
 
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH6, INPUT);
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void loop() {
  ch2Value = (int) readChannel(CH2, -126, 126, 0);
  ch3Value = (int) readChannel(CH3, -126, 126, 0);
  ch6Value = readSwitch(CH6, false);

  // Output the values to the Serial Monitor
 // Serial.print("\n ");
 // Serial.print("Ch2: ");
 // Serial.print(ch2Value);
 // Serial.print(" | Ch3: ");
 // Serial.print(ch3Value);
 // Serial.print(" | Ch6: ");
 // Serial.print(ch6Value);
   

  // Drive the motors based on the channel values
  if (abs(ch3Value) >= BIAS) {
    ST.motor(1, ch3Value);
  //  Serial.print(" | Motor 1 power: ");
  //  Serial.print(ch3Value);
  } else {
    ST.motor(1, 0);
 //   Serial.print(" | Motor 1 power: 0");
  }

  if (abs(ch2Value) >= BIAS) {
    ST.motor(2, ch2Value);
  //  Serial.print(" | Motor 2 power: ");
   // Serial.print(ch2Value);
  } else {
    ST.motor(2, 0);
//    Serial.print(" | Motor 2 power: 0");
  }

  myCmd=Serial.readStringUntil('\r');
  if (myCmd == "Turn right") {
    ch2Value=50;
   // ST.motor(1, 50);
  }
  if (myCmd == "Turn left") {
    ch3Value=50;
   // ST.motor(2, 50);
  }

//  Serial.println(); // Print a new line to improve readability

  delay(100);
}
