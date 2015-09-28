#include <SoftwareSerial.h>

SoftwareSerial bt(4,3);

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
  Serial.print("I am going forward");
else if(inputString == "~reverse\n")
  Serial.print("I am going in reverse"); 
else if(inputString == "~left\n")
  Serial.print("I am going in left"); 
else if(inputString == "~right\n")
  Serial.print("I am going in right"); 
else if(inputString == "~start\n")
  Serial.print("I am going in start"); 
else if(inputString == "~select\n")
  Serial.print("I am going in select"); 
else if(inputString == "~a\n")
  Serial.print("I am going in a"); 
else if(inputString == "~b\n")
  Serial.print("I am going in b");  
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


