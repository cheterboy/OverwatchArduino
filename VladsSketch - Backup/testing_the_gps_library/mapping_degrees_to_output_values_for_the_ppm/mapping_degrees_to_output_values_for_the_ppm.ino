#include <TinyGPS++.h>
#include <SoftwareSerial.h>

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


int ppm[chanel_number];

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
SoftwareSerial bluetooth(5,4); 


void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 //while (!Serial){;}
 ss.begin(GPSBaud);
 //while (!ss){;}
 bluetooth.begin(9600);
// while (!ss){;}
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

void loop() {

  double north_lat = 37.655659, north_lon = -97.247204;

  vladCourseTo(north_lat, north_lon);
  Serial.println();
  
  smartDelay(500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));
  

}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  ss.listen();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


// tells how much the truck should turn to get to a point 
void vladCourseTo(double latitude, double longitude)
{
  if (gps.location.isValid()){
    //gets the degree of where the next waypoint is 
    double courseDegree =TinyGPSPlus::courseTo(gps.location.lat(),
        gps.location.lng(),latitude,longitude);
  
  // checks to see if the current gps coordinate is valid
  
    double x , y;
    Serial.print("This is the course degree   "); 
    Serial.print(courseDegree);
    Serial.print("   ");

  
if (courseDegree >= 180)//if the waypoint is to the left 
{

  ppm[FowardReverse] = 1500;
  y =  map(courseDegree, 180 , 359, 1500 , 2000);// go left (will be trouble because the car does not turn 90 
  ppm[LeftRight] = map(courseDegree, 180 , 359, 1500 , 2000);// go left (will be trouble because the car does not turn 90 
 // x =  1500;// always forward
 
}
else
{

    ppm[FowardReverse] = 1500;
    y = map(courseDegree, 0 , 179, 1000 , 1500);//go right (will be trouble because the car does not turn 90 
    ppm[LeftRight] = map(courseDegree, 0 , 179, 1000 , 1500);//go right (will be trouble because the car does not turn 90 
  //x =  1500;// always forward
 
}
    Serial.print(x);
    Serial.print("   ");
    Serial.print(y);
    bluetooth.listen();
    bluetooth.print(x);
    bluetooth.print("   ");
    bluetooth.print(y);

    printFloat(courseDegree, gps.location.isValid(), 7, 2);
}

  
  

}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
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



