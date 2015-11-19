// nrf24_reliable_datagram_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
// It is designed to work with the other example nrf24_reliable_datagram_server
// Tested on Uno with Sparkfun WRL-00691 NRF24L01 module
// Tested on Teensy with Sparkfun WRL-00691 NRF24L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module
#include <SoftwareSerial.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_NRF24 driver;
// RH_NRF24 driver(8, 7);   // For RFM73 on Anarduino Mini

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

SoftwareSerial bt(6,7);//tx rx
String inputString;         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

#define buttonPin 3


void setup() 
{
  Serial.begin(9600);
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm

  pinMode(buttonPin, INPUT_PULLUP);
  bt.begin(9600);
  inputString.reserve(200);
}

uint8_t data[RH_NRF24_MAX_MESSAGE_LEN];
// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];








void loop()
{

  if(digitalRead(buttonPin) == 0) 
      {   
          inputString = "kill\n"; 
          stringComplete = true; 
      }
  
   if (!stringComplete)
    serialEvent();
  
  //Serial.println("Sending to nrf24_reliable_datagram_server");
   while(stringComplete){
       Serial.println(inputString); 
  
      inputString.getBytes(data,RH_NRF24_MAX_MESSAGE_LEN);
      inputString.remove(0, RH_NRF24_MAX_MESSAGE_LEN); 
  // Send a message to manager_server
  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      if(inputString.length() == 0)
      { 
      stringComplete = false; 
      inputString = ""; 
      } 
    }
    else
    {
      Serial.println("No reply, is nrf24_reliable_datagram_server running?");
    }
  }
  else
    Serial.println("sendtoWait failed");


  
   }
  
  delay(500);
}




void serialEvent() {
  

  //if(bt.isListening())
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


