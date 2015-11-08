// nrf24_reliable_datagram_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
// It is designed to work with the other example nrf24_reliable_datagram_server
// Tested on Uno with Sparkfun WRL-00691 NRF24L01 module
// Tested on Teensy with Sparkfun WRL-00691 NRF24L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module

#include <RH_TCP.h>
#include <RH_Serial.h>
#include <RH_RF95.h>
#include <RH_RF69.h>
#include <RH_RF24.h>
#include <RH_RF22.h>
#include <RH_NRF905.h>
#include <RH_NRF51.h>
#include <RH_ASK.h>
#include <RHTcpProtocol.h>
#include <RHSPIDriver.h>
#include <RHSoftwareSPI.h>
#include <RHRouter.h>
#include <RHNRFSPIDriver.h>
#include <RHMesh.h>
#include <RHHardwareSPI.h>
#include <RHGenericSPI.h>
#include <RHGenericDriver.h>
#include <RHDatagram.h>
#include <RHCRC.h>
#include <radio_config_Si4460.h>
#include <RadioHead.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <String.h> 



#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_NRF24 driver;
//RH_NRF24 driver(8, 10);   // For RFM73 on Anarduino Mini

SoftwareSerial bt(6, 7);//tx rx
String inputString;         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

								 // Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

#define buttonPin 3

void setup()
{

	pinMode(buttonPin, INPUT);

	Serial.begin(9600);
	if (!manager.init())
		Serial.println("init failed");

	bt.begin(9600);
	/* delay(1000);
	bt.print("AT");
	delay(1000);
	bt.print("AT+NAMEHC-06");
	delay(1000);
	bt.print("AT+PIN1234");
	delay(1000); */




	inputString.reserve(200);
	// Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
}

uint8_t data[RH_NRF24_MAX_MESSAGE_LEN];
// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

void loop()
{

	//Serial.println(analogRead(A0)); 

	//Serial.println("Sending to nrf24_reliable_datagram_server");
	if (digitalRead(buttonPin) == LOW)
	{
		inputString = "kill";
		stringComplete = true;
		Serial.print("Kill");

	}

	inputString = Serial.readString(); 
	stringComplete = true; 


	if (stringComplete) {
		Serial.println(inputString);
		uint8_t test[200];

		inputString.getBytes(test, 200);
		//inputString.getBytes(data,200);
		/*
		Serial.print("sizeof(inputString) = ");
		Serial.println(inputString.length());
		Serial.print("RH_NRF24_MAX_MESSAGE_LEN = ");
		Serial.println(RH_NRF24_MAX_MESSAGE_LEN);
		Serial.print("sizeof(data) = ");
		Serial.println(sizeof(data));
		Serial.print("sizeof(test) = ");
		Serial.println(sizeof(test));
		*/



		if (inputString.length() > RH_NRF24_MAX_MESSAGE_LEN)
			Serial.println("The string is longer");

		if (manager.sendtoWait(test, inputString.length(), SERVER_ADDRESS))
		{
			// Now wait for a reply from the server
			uint8_t len = sizeof(inputString);
			uint8_t from;


			if (manager.recvfromAckTimeout(data, &len, 2000, &from))
			{
				Serial.print("got reply from : 0x");
				Serial.print(from, HEX);
				Serial.print(": ");
				Serial.println((char*)data);



				//this might have to get moved
				inputString = "";
				stringComplete = false;
			}
			else
			{
				Serial.println("No reply, is nrf24_reliable_datagram_server running?");
			}
		}
		else
			Serial.println("sendtoWait failed Resending");



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




