/*
  Need to figure out how to disconnect
  need to figure out how to enter AT mode

*/
#include <SoftwareSerial.h>
#include <SimpleTimer.h>

SimpleTimer timer;


#define connectedStatePin 12

int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3

String inputString;
String uadCommands[100] = {"One", "Two", "Three"};
int commandCounter = 0;



bool stringComplete = false;

void serialEvent();
bool atMode();
void storeBTAddres();
bool connectBTNode();
bool terminateBTConnection();
void displayBluetoothSettings();

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

bool atState = false;


bool atMode() {
//  if (!atState)
//  {

    bluetooth.print('$');
    bluetooth.print('$');
    bluetooth.print('$');
    bluetooth.flush();
    delay(100);

    String test = bluetooth.readString();
    if (test.startsWith("CMD") )
    {
      atState = true;
      return true;
    } else
      return false;
//  } else
//    return true;


  //Serial.print("Inside the at mode function - ");

}

void displayBluetoothSettings()
{
  //if (!atState)
 // {
    atMode();
 // }

  String command = "D";

  bluetooth.println(command);
  delay(50);

  while (bluetooth.available())
    Serial.print((char)bluetooth.read());

}


//Add a string as input that holds a bluetooth address
void storeBTAddres() {
  String command = "SR,201502041295";
  delay(10);
  bluetooth.println(command);
  delay(100);

  String test = bluetooth.readString();
  Serial.print(test.startsWith("AOK") );
  // Serial.print("Storing  ");
  //Serial.print("201502041295 -");
}

bool connectBTNode()
{
  if (digitalRead(connectedStatePin) == 0)
  {
    if (!atState)
      if (!atMode())
        return false;


    String command = "C,201502041295";
    delay(10);
    bluetooth.println(command);

    unsigned long start = millis();
    do
    {
      if (digitalRead(connectedStatePin) == 1)
      {
        atState = false;
        digitalWrite(13, HIGH);
        return true;
        delay(1000);
      }

    } while (millis() - start < 10000);

    return false;
  }
  return false;
}


bool terminateBTConnection() {

  if (digitalRead(connectedStatePin) == 1)
  {
    Serial.println("TerminalBTConnection inside if"); 
    //if (!atState)
    //  if (!atMode())
    //   return false;

    if (!atMode())
    {
     Serial.println("TerminalBTConnection inside  second if"); 
      String command = "K,1";

      bluetooth.println(command);
      delay(100);
      String test = bluetooth.readString();
      //Serial.print(test);
      Serial.print(test.startsWith("KILL") );

      unsigned long start = millis();
      do
      {
        if (digitalRead(connectedStatePin) == 0)
        {

          digitalWrite(13, LOW);
          atState = false;
          return true;

        }

      } while (millis() - start < 10000);


      return false;


    }
  }
  else
    return true;


}


static void readBluetooth(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (bluetooth.available())
      Serial.print((char)bluetooth.read());
  } while (millis() - start < ms);
}



void setup()
{
  pinMode(connectedStatePin, INPUT);
  pinMode(13, OUTPUT);
  Serial.begin(38400);  // Begin the serial monitor at 9600bps

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps

  //  Serial.println(atMode());
  bluetooth.print("$");
  bluetooth.print("$");
  bluetooth.print("$");
  //delay(100);
  readBluetooth(300);


  bluetooth.println("SU,38.4");  // Temporarily Change the baudrate to 9600, no parity
  //delay(100);
  readBluetooth(300);

  bluetooth.println("R,1");
  readBluetooth(300);
  bluetooth.begin(38400);  // Start bluetooth serial at 9600
  //delay(100);*/

  bluetooth.println("SF,1");
  //delay(100);
  readBluetooth(300);

  bluetooth.println("ST,255");
  //delay(100);
  readBluetooth(300);

  bluetooth.println("SO,%");
  //delay(100);
  readBluetooth(300);

  bluetooth.println("R,1");
  //delay(100);
  readBluetooth(300);

  bluetooth.setTimeout(300);


  Serial.println("SetupDone");
}

void loop()
{
  //timer.run();
  /* while (bluetooth.available()) // If the bluetooth sent any characters
    {
     // Send any characters the bluetooth prints to the serial monitor
     Serial.print((char)bluetooth.read());
    }*/
  //int counter = 0;
  if (Serial.available()) // If stuff was typed in the serial monitor
  {
    char inChar = (char)Serial.read();
    switch (inChar) {
      case '1':
        Serial.println(atMode());
        break;
      case '2':
        storeBTAddres();
        break;
      case '3':
        Serial.println(connectBTNode());
        break;
      case '4':
        Serial.println(terminateBTConnection());
        break;
      case '5':
        displayBluetoothSettings();
        break;
      default:

        break;
    }




    //  bluetooth.print((char)inChar);
    // Send any characters the Serial monitor prints to the bluetooth


  }




  serialEvent();


  if (stringComplete) {
    //Serial.println(inputString);



    if (inputString.startsWith("~") ) {
      Serial.println("UAD command stored");
      uadCommands[commandCounter] = inputString;
      commandCounter++;
    } else if (isdigit(inputString[0]) )
    {
      Serial.print("Killing Connection");

      Serial.println(terminateBTConnection());

      Serial.print("Connecting to bluetooth");
      Serial.println(connectBTNode());

      Serial.print("Printing to bluetooth");
      for (int counter = 0; counter < commandCounter; counter++)
      {
        bluetooth.print(uadCommands[counter]);
        bluetooth.flush();
        //delay(1000);
      }

      Serial.print("\nterminating bluetooth");
      while (!Serial.println(terminateBTConnection()));
    }


    inputString = "";
    stringComplete = false;

  }
}









void serialEvent() {
  while (bluetooth.available()) {
    // get the new byte:
    char inChar = (char)bluetooth.read();
    Serial.print(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


