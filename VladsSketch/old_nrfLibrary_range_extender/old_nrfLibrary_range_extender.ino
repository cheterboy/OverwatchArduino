/*
Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation.
*/

/**
* Example for Getting Started with nRF24L01+ radios.
*
* This is an example of how to use the RF24 class.  Write this sketch to two
* different nodes.  Put one of the nodes into 'transmit' mode by connecting
* with the serial monitor and sending a 'T'.  The ping node sends the current
* time to the pong node, which responds by sending the value back.  The ping
* node can then see how long the whole cycle took.
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 

RF24 radio(8, 10);

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[1] = {0xF0F0F0F0D2LL};

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  
//

// The various roles supported by this sketch

// The debug-friendly names of those roles



void setup(void)
{
	//
	// Print preamble
	//

	Serial.begin(57600);
	

	//
	// Setup and configure rf radio
	//

	radio.begin();
  //radio.disableCRC();
  radio.setChannel(120);
  //radio.enableDynamicPayloads();
  
	// optionally, increase the delay between retries & # of retries
	//radio.setRetries(15, 15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	//radio.setPayloadSize(8);

	//
	// Open pipes to other nodes for communication
	//

	// This simple sketch opens two pipes for these two nodes to communicate
	// back and forth.
	// Open 'our' pipe for writing
	// Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

	//if ( role == role_ping_out )
	{
		//radio.openWritingPipe(pipes[0]);
		//radio.openReadingPipe(1, pipes[1]);
	}
	//else
	{
		//radio.openWritingPipe(pipes[0]);

	}

	//
	// Start listening
	//

//	radio.startListening();

	//
	// Dump the configuration of the rf unit for debugging
	//

	//radio.printDetails();
   radio.openReadingPipe(0,pipes[0]);
   radio.openReadingPipe(1,pipes[0]);

 
}

void loop(void)
{
	radio.startListening();

  delay(20);
 
 // Serial.println(radio.available());
  //Serial.println((char)pipes[0]);
		if (radio.available())
		{
      char data[32] ;
      String test; 
	// Fetch the payload, and see if this was the last one.
			radio.read(&data, sizeof(data ));

      test = data; 
      
				// Spew it
				Serial.print("Got payload ");
				Serial.println(test);

       if (test == "test")
          Serial.println("That message was a test"); 
          
		
			}

    
     

			
  

	
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
