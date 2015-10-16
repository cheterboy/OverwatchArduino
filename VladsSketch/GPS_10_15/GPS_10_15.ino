// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 4); // RX, TX



void setup()
{
  Serial.begin(9600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  mySerial.begin(9600);
}

void loop()
{
   //if (mySerial.available())
   // Serial.write(mySerial.read());
  //if (Serial.available())
  //  mySerial.write(Serial.read());
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  if (mySerial.available())
  Wire.write(mySerial.read()); // respond with message of 6 bytes
  // as expected by master
}
