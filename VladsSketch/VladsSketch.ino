//this will recive bluetooth strings


//bluetooth tx == arduino 4
//
/*
Bluetooth module -> Arduino
Grnd -> Grnd
VCC -> 5v
Rx -> pin3
Tx -> pin4
*/


#include <SoftwareSerial.h>

SoftwareSerial bt(4,3);//tx rx

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  bt.begin(9600); 
  delay(1000);
  
  bt.print("AT");
  delay(1000);
  //Serial.print(bt.read());
  bt.print("AT+NAMEOverWatch");
  delay(1000);
  bt.print("AT+PIN9999");
  Serial.println("Done");

  inputString.reserve(200);
bt.flush();


}

void loop() {

serialEvent();
if(stringComplete){
//Serial.print(inputString);

if(inputString == "~forward\n")
  Serial.print("I am going forward\n");
else if(inputString == "~backward\n")
  Serial.print("I am going backward\n"); 
else if(inputString == "~left\n")
  Serial.print("I am going left\n"); 
else if(inputString == "~right\n")
  Serial.print("I am going right\n"); 
else if(inputString == "~start\n")
  Serial.print("I am starting\n"); 
else if(inputString == "~select\n")
  Serial.print("I am selecting\n"); 
else if(inputString == "~a\n")
  Serial.print("I am a ing\n"); 
else if(inputString == "~b\n")
  Serial.print("I am b ing\n");  
inputString ="";
stringComplete = false;
  
}
if(Serial.available())
  bt.write(Serial.read());
}


void serialEvent() {
  while (bt.available()) {
    // get the new byte:
    char inChar = (char)bt.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


