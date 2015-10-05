
#include <SoftwareSerial.h>

SoftwareSerial bt(4,3);//tx rx

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete



void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 bt.begin(9600);
 Serial.println("Done");
 inputString.reserve(200);
 
}

void loop() {
  // put your main code here, to run repeatedly:
 serialEvent();


  if(stringComplete){
    //Serial.print(inputString); 
    if(inputString == "~forward\n"){
        Serial.print("Forward");
    }else if(inputString == "~notforward\n"){
        Serial.print("NOT Forward");
    }else if(inputString == "~backward\n"){
        Serial.print("backward");
    }else if(inputString == "~notbackward\n"){
        Serial.print("NOT backward");
    }else
    {
      Serial.print("The string is "); 
      Serial.println(inputString); 
      String longitude = inputString;
      inputString.remove(0, 11);
      inputString.remove(13, 200);
      
      Serial.print("The formatted string is "); 
      Serial.println(inputString);
       
      double latitude = atof(inputString.c_str());
      
      Serial.print("The float value is "); 
      Serial.println(latitude, 6);
      
     

      
    }
    
    inputString = "";
    stringComplete = false; 
}
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
