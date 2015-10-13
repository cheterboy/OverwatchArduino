// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include <Wire.h>


String inputString; 


void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  inputString.reserve(200);
}


void loop()
{

  uint8_t  x = 0; 
  Wire.requestFrom(8, 1);    // request 6 bytes from slave device #8

  while (Wire.available())   // slave may send less than requested
  {
   
  x = (uint8_t)Wire.read();

  }
   //Serial.println(x);
  Serial.print("Forward = "); 
  Serial.println(1000+(x/10*100));
   
  
   Serial.print("Reverse = "); 
  Serial.println(1000+((x%10)*100));
  
   
   //x = 0;
   //inputString = "";
 
  delay(500);
}
