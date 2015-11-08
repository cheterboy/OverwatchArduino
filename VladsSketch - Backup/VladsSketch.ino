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

SoftwareSerial bt(7,8);//tx rx

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  bt.begin(9600); 
  
  /*delay(3000);
  
  bt.print("AT");
 // bt.flush();
  delay(1000);
  //Serial.print(bt.read());
  bt.print("AT+NAMEOverWatchVlad1");
 // bt.flush();
  delay(1000);
  bt.print("AT+PIN9999");
 // bt.flush();
  Serial.println("Done");
*/
  inputString.reserve(200);
  //bt.flush();


}

void loop() {

serialEvent();
if(stringComplete){
Serial.println(inputString);

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
<<<<<<< HEAD
else if(inputString.startsWith("[lat/lng: ("))
  {
    
    //inputString.toInt();//replace("[lat/lng: ()" , "L");
    //char floatbuf[sizeof("[lat/lng: (37.668156139698645,-97.39461153745651)]")]; 
    //inputString.toCharArray(floatbuf, sizeof("[lat/lng: (37.668156139698645,-97.39461153745651)]")); 
    
    inputString.remove(0 , 11);
    inputString.remove(17 , 30 );

    Serial.print("The string is ");
    Serial.println(inputString);

    
    //double test = atoi(inputString.c_str());
    //test = atol(floatbuf);

     
    double x = 0.000000;
    x= ato(inputString.c_str());

  //char * cstr = new char[sizeof(inputString)+1];
  

    Serial.print("The formatted string is "); 
    //Serial.print(inputString.toFloat()); 
    Serial.print(x); 
        
  }
  
=======
else 
  Serial.print(inputString);
>>>>>>> c2b21ea15ca7d3a73c0bf8b575b41c80c6152e6f
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


