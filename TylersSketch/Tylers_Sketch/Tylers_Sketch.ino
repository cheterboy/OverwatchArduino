//this programm will put out a PPM signal

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 6  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 4  //set PPM signal output pin on the arduino
#define FowardReverse 0 //set foward and reverse channel
#define LeftRight 1 //set left and right channel
//////////////////////////////////////////////////////////////////

 unsigned long start; 
 unsigned long finish; 

int FRValue = 1500; 
int LRValue = 1500; 

 
/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

void setup(){  
  //initiallize default ppm values

  Serial.begin(250000); 
  
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
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
int x; 
void loop(){

 ppm[FowardReverse] = FRValue; //map(analogRead(A0), 0, 1024, 1000, 2000); //value should be between 1000= full reverse, 1500=off, and 2000= full foward
 ppm[LeftRight] = LRValue;//map(analogRead(A1), 0, 1024, 1000, 2000); //value should be between 1000= full left, 1500= centered, and 2000= full right(could be inversed will know for sure once installed)


  //Serial.println(millis());
  /*
  if( millis() > 5000)
    {
      FRValue = 1600;
      LRValue = 1600;
      //Serial.println("Here");
    }*/
    


  //delay(10);
  ////x = finish - start; 
  delay(10000); 
  Serial.println( x);
}

ISR(TIMER1_COMPA_vect){  //leave this alone
 // finish = micros(); 
 
  x = micros() - start ; 
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



  start = micros(); 

  
}
