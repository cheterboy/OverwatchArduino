#include <SoftwareSerial.h>
//ble 8,9
//
int dataIn = 6; //DATA PIN FOR ARDUINO MICRO
int clockIn = 3;//CLOCK PIN FOR ARDUINO MICRO (this is default)
SoftwareSerial ble(11,12);

const int buttonPin = 5; //pushButton pin
int buttonState = 0;
int buttonCheck = 0;

int isin=0; //inch = 1 & mm = 0
int isfs=0; //- = 1
int index;

unsigned long xData,oData;
String outputString;
String outputString2;
int readingCounter = 0;

int ledPin = 13; // the number of the LED pin
int ledState = LOW; // ledState used to set the LED
long previousMillis = 0; // will store last time LED was updated
long interval = 500; // interval at which to blink (milliseconds)

long previousGetMillis = 0;
long Timeout = 8; //8ms

void setup(){
  
  pinMode(buttonPin, INPUT);
  ble.begin(57600);
  digitalWrite(dataIn, 1);
  digitalWrite( clockIn, 1);
  pinMode(dataIn, INPUT);
  pinMode(clockIn, INPUT);
  Serial.begin(9600);
  delay(500);
  attachInterrupt(0,getBit,RISING);
  index =0;
  xData=0;
  oData=999;
}

void loop(){

  //Serial.println("Here");
  
  //buttonState = digitalRead(buttonPin);
  
  if ((index !=0) && (millis() - previousGetMillis > Timeout) ) {
  //Serial.println("Here");
  
  index=0;
  xData=0;
  outputString = "";
  };
  if (index >23) {
    Serial.println("Here");

      //output for simple chat iPhone app (one reading on screen at a time
      //outputString2 = "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
      outputString2 += "Count ";
      outputString2 += readingCounter;
      outputString2 += " Begin ";
      outputString = "";
    
    if (oData !=xData) {
      if (isfs==1) {
        
        //Serial.print('-');
        //ble.print('-');
        outputString += "-";

      }
      if (isin==1){ //reading in inches
        xData *=5;
                
        //Serial.print(xData/10000);
        //Serial.print('.');
        //ble.print(xData/10000);
        //ble.print('.');
        outputString += (xData/10000);
        outputString += ".";
          
          if ((xData % 10000)<1000){
            if ((xData % 10000)<100){
              if ((xData % 10000)<10){
                
                
                //Serial.print('0');
                //ble.print('0');
                outputString += "0";              
                
              };
              
              
              //Serial.print('0');
              //ble.print('0');
              outputString += "0";
              
            };

            
            //Serial.print('0');
            //ble.print('0');
            outputString += "0";
            
          };
        
        
        //Serial.println(xData % 10000);
        //ble.println(xData % 10000);
        outputString += (xData % 10000);
        outputString2 += outputString;
        //Serial.println("");
        Serial.println(outputString2);
        Serial.println("Here2");
        ble.println(outputString2);
        readingCounter++; //Only increment when button pushed
        
        
      } else { //reading in mm
        
        
        //Serial.print(xData/100);
        //ble.print(xData/100);
        outputString += (xData/100);

        
        //Serial.print('.');
        //ble.print('.');
        outputString += ".";
        
        if ((xData % 100)<10) { 
          
          
          //Serial.print('0');
          //ble.print('0');
          outputString += "0";
          
        }

        
        //Serial.println(xData % 100);
        //ble.println(xData % 100);
        outputString +=(xData % 100); 
        outputString2 += outputString;
        //Serial.println("");
        Serial.println(outputString2);
        ble.println(outputString2);
        readingCounter++; //Only increment when button pushed
 
      };
    };
    
    oData =xData;
    index=0;
    xData=0;
    //readingCounter
  };
  if (millis() - previousMillis > interval) {
    previousMillis = millis();
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(ledPin, ledState);
  }

}

void getBit(){
  previousGetMillis=millis();
  if(index < 20){
    if(digitalRead(dataIn)==1){
      xData|= 1<<index;
    };
  } else {
  if (index==20) {
    isfs=digitalRead(dataIn);
  }
  if (index==23) {
    isin=digitalRead(dataIn);
  }
  };
  index++;
}

