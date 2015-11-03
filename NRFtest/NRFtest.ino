#include <RF24.h>
#include <RF24_config.h>
#include <printf.h>
#include <nRF24L01.h>
#include<RF24-master\RF24.h>

RF24 test; 


void setup()
{
	test.begin();
	Serial.begin(9600); 

  /* add setup code here */

}

void loop()
{
	Serial.parseFloat();


  /* add main program code here */

}
