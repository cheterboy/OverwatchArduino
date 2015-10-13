
#include <SoftwareSerial.h>
#include <EEPROM.h>
/*
struct Coordinate{
  float latitude;
  float longitude;
  //int number;
};
*/

String test[50]; 
int otherTest = 0; 

SoftwareSerial bt(6,7);//tx rx

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
//int eeAddress = 0;


void setup() {
  // put your setup code here, to run once:
 Serial.begin(250000);
 bt.begin(1200);
/*
 delay(1000);
  
 bt.print("AT");
 delay(1000);
 bt.print("AT+BAUD1");
 delay(1000);*/
 
 inputString.reserve(200);
 //Serial.println("Done");

 
}

void loop() {
  // put your main code here, to run repeatedly:
 serialEvent();

  if(stringComplete){
    //Serial.println(inputString);
    //String final = inputString;
    //Serial.print(inputString); 
    if(inputString == "~a"){
     int counter = otherTest;
     while(counter > 0){
      counter--; 
      //inputString.trim();
      Serial.println(test[counter]);
      //int waypointNumber = atol(test[counter].c_str()); 
       delay(10);
      String waypointNumber = test[counter]; 
      String latitude = test[counter];
      String longitude = test[counter];
 delay(10);
      int one = test[counter].indexOf(',');
      //int two = inputString.indexOf(',', one + 1 );
      int two = test[counter].lastIndexOf(',');


      waypointNumber.remove(one); 
      latitude.remove(0,one+1);
      latitude.remove(two -2);
      
      longitude.remove(0,two+1); 

      char *ptr;
      delay(10);
      double numberOne = strtod(waypointNumber.c_str(),&ptr);
      double numberTwo = strtod(latitude.c_str(),&ptr);
      double numberThree = strtod(longitude.c_str(),&ptr);
      delay(10);

      Serial.print("waypointNumber = "); 
      Serial.println(counter); 
      Serial.print("latitude = "); 
      Serial.println(numberTwo,6); 
      Serial.print("longitude = ");
      Serial.println(numberThree,6); 
      
      Serial.println(); 
      Serial.println(); 
     } 

     otherTest = 0; 
    }else if (isdigit(inputString[0])){

      //store the coordinate in an array 
      test[otherTest] = inputString; 
      otherTest++; 
      //Serial.println("Stored"); 
    }

    
    inputString = "";
    stringComplete = false; 
 
}




}
  






void serialEvent() {
  bt.listen();
  while (bt.available()) {
    // get the new byte:
    char inChar = (char)bt.read();
    // add it to the inputString:
    if (inChar == '\n') {
      //inputString.trim(); 
      stringComplete = true;
    }else
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    
    }

    
  }

