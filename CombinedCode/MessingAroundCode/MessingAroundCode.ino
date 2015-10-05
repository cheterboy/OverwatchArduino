//this programm will put out a PPM signal

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 6  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino
#define FowardReverse 0 //set foward and reverse channel
#define LeftRight 1 //set left and right channel
//////////////////////////////////////////////////////////////////


#include <SoftwareSerial.h>

SoftwareSerial bt(4,3);//tx rx

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

int FRValue = 1500;
int LRValue = 1500;



/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

void setup(){  
  //initiallize default ppm values
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;

    // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  bt.begin(9600); 
  //delay(1000);
 /* bt.print("AT");
  delay(1000);
  //Serial.print(bt.read());
  bt.print("AT+NAMEOverWatch");
  delay(1000);
  bt.print("AT+PIN9999");
  delay(1000); 
  bt.print("AT+BAUD1");
  */
  Serial.println("Done");

  inputString.reserve(200);
  //bt.flush();


    
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void loop(){

//int maxSpeed = 1500;
//bool goingForward = false; 



if(stringComplete){
Serial.print(inputString); 
delay(100);
  
if(inputString == "~notforward\n"){
     FRValue = increase(FRValue, FowardReverse, 1600); 
}else if(inputString == "~forward\n"){
     FRValue = decrease(FRValue, FowardReverse, 1500);
}else if(inputString == "~notbackward\n"){
     FRValue = decrease(FRValue, FowardReverse, 1400); 
}else if(inputString == "~backward\n"){
     FRValue = increase(FRValue, FowardReverse, 1500); 
}else if(inputString == "~notleft\n"){
     LRValue = decrease(LRValue, LeftRight, 1250);
}else if(inputString == "~left\n"){
     LRValue = increase(LRValue, LeftRight, 1500);
}else if(inputString == "~notright\n"){
     LRValue = increase(LRValue, LeftRight, 1750);
}else if(inputString == "~right\n"){
     LRValue = decrease(LRValue, LeftRight, 1500);
}else if(inputString == "~start\n"){
     //Serial.print("Starting");
     if (LRValue < 1500)
     LRValue = increase(LRValue, LeftRight, 1500); 
     else if (LRValue > 1500)
     LRValue = decrease(LRValue, LeftRight, 1500); 

     if (FRValue < 1500)
      FRValue = increase(FRValue, FowardReverse, 1500); 
     else if (FRValue > 1500)
      FRValue = decrease(FRValue, FowardReverse, 1500); 
      
}else if(inputString == "~select\n"){
    Serial.print("Selecting");
}else if(inputString == "~a\n"){
     Serial.print("Selecting");
}else if(inputString == "~b\n"){
  Serial.print("B-ing");
}


inputString ="";
stringComplete = false;
}
 

 ppm[FowardReverse] = FRValue; //map(test0, 0, 1024, 1000, 2000); //value should be between 1000= full reverse, 1500=off, and 2000= full foward
 ppm[LeftRight] = LRValue; //map(test1, 0, 1024, 1000, 2000); //value should be between 1000= full left, 1500= centered, and 2000= full right(could be inversed will know for sure once installed)
 
 serialEvent();
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}



void serialEvent() {
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


int increase(int currentSetting,int channel, int targetSetting ){ 
  while (currentSetting < targetSetting){
      ppm[channel] = currentSetting; //map(test0, 0, 1024, 1000, 2000); //value should be between 1000= full reverse, 1500=off, and 2000= full foward
      currentSetting += 1;
      Serial.println(currentSetting); 
      delay(1);
       }
      return currentSetting;
     }

int decrease(int currentSetting, int channel , int targetSetting){ 
  while (currentSetting > targetSetting){
      ppm[channel] = currentSetting; 
      currentSetting -= 1;
      Serial.println(currentSetting); 
      delay(1);
       }
      return currentSetting;
     }
          




