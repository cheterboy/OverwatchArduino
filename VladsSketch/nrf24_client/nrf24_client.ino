// nrf24_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_NRF24 class. RH_NRF24 class does not provide for addressing or
// reliability, so you should only use RH_NRF24 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example nrf24_server.
// Tested on Uno with Sparkfun NRF25L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module

#include <SoftwareSerial.h>
#include <SPI.h>
#include <RH_NRF24.h>

// Singleton instance of the radio driver
RH_NRF24 nrf24;
// RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini


SoftwareSerial bt(6,7);//tx rx
String inputString;         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

#define buttonPin 3



void setup() 
{
  Serial.begin(250000);
  while (!Serial) 
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  else
    Serial.println("init OK");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  pinMode(buttonPin, INPUT_PULLUP);
  bt.begin(9600);
  /*delay(1000);
  bt.print("AT");
  delay(1000);
  bt.print("AT+BAUD4");
  delay(1000);*/

  
  inputString.reserve(200);
}

uint8_t data[RH_NRF24_MAX_MESSAGE_LEN];

void loop()
{
 //Serial.println(digitalRead(buttonPin));
 if(digitalRead(buttonPin) == 0) 
    {   
        inputString = "kill\n"; 
        stringComplete = true; 
    }

 if (!stringComplete)
  serialEvent();
 
 while(stringComplete){
       //Serial.println(inputString); 
  
      inputString.getBytes(data,RH_NRF24_MAX_MESSAGE_LEN);
      inputString.remove(0, RH_NRF24_MAX_MESSAGE_LEN); 
//Serial.println("Packet sent");

/*
       Serial.print("Sending ");
       Serial.print((char*)data); 
       Serial.print("  ");
       Serial.println(nrf24.send(data, sizeof(data)));
       Serial.print("waiting to transmitt ");
       Serial.println(nrf24.waitPacketSent());
      */

     if(inputString.length() == 0)
     { 
      stringComplete = false; 
      inputString = ""; 
     } 
     
  
  

 }
  
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

