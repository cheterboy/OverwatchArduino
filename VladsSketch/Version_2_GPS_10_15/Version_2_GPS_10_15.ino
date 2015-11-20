// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include <SoftwareSerial.h>
//#include <TinyGPS++.h>
SoftwareSerial mySerial(6, 7); // RX, TX



void setup()
{
  Serial.begin(9600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  mySerial.begin(9600);
  delay(1000); 
  Serial.println("GPS initializaton done"); 
}

//String inputString; 

void loop()
{
//while(mySerial.available()) 
//if(mySerial.available())
//  Serial.print((char)mySerial.read());
  //delay(10); 
//if(Serial.available())
//  mySerial.print(Serial.read()); 
//    delay(1);   
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  //Serial.print("Here"); 
  if(mySerial.available())
    {
     // char x = (char)mySerial.read(); 
     // Serial.print(x); 
      Wire.write((char)mySerial.read()); 
    }

}
