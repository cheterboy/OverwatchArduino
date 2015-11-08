#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 2, TXPin = 4;
static const uint32_t GPSBaud = 9600;


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


// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


SoftwareSerial bluetooth(4,5);

void setup()
{
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
  
  bluetooth.begin(9600);
  ss.begin(GPSBaud);

  bluetooth.println(F("FullExample.ino"));
  bluetooth.println(F("An extensive example of many interesting TinyGPS++ features"));
  bluetooth.print(F("Testing TinyGPS++ library v. ")); bluetooth.println(TinyGPSPlus::libraryVersion());
  bluetooth.println(F("by Mikal Hart"));
  bluetooth.println();
  bluetooth.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  bluetooth.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  bluetooth.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop()
{
  static const double LONDON_LAT = 37.718328, LONDON_LON = -97.292439;

  //printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  //printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  //printInt(gps.location.age(), gps.location.isValid(), 5);
  //printDateTime(gps.date, gps.time);
  //printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  //printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);
      /*
  printInt(distanceKmToLondon, gps.location.isValid(), 9);
*/
  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  // ppm[FowardReverse] = 1500;//map(, , 1024, 1000, 2000); //value should be between 1000= full reverse, 1500=off, and 2000= full foward
   
  // if(courseToLondon > 180.0){
  // ppm[LeftRight] = map(courseToLondon, 181, 359, 1000, 1500); //value should be between 1000= full left, 1500= centered, and 2000= full right(could be inversed will know for sure once installed)
  // bluetooth.print("Go left ");
  // bluetooth.print(360 - courseToLondon); 
   
  // }else {
  // ppm[LeftRight] = map(courseToLondon, 0, 180 , 1500, 2000); //value should be between 1000= full left, 1500= centered, and 2000= full right(could be inversed will know for sure once installed)
  // bluetooth.print("Going right ");
  // bluetooth.print(180 - courseToLondon); 
  // }//
   
   const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);
   String test = cardinalToLondon; 

  if (test.endsWith("W"))
  bluetooth.print("Go left"); 
  else if (test.endsWith("E"))
  bluetooth.print("Go right"); 
  

  //printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  //printInt(gps.charsProcessed(), true, 6);
  //printInt(gps.sentencesWithFix(), true, 10);
  //printInt(gps.failedChecksum(), true, 9);
  bluetooth.println();
  
  smartDelay(100);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    bluetooth.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    ss.listen();
    while (ss.available()){
     gps.encode(ss.read());       
    }
  } while (millis() - start < ms);
  bluetooth.listen();
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      bluetooth.print('*');
    bluetooth.print(' ');
  }
  else
  {
    bluetooth.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      bluetooth.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  bluetooth.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    bluetooth.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    bluetooth.print(sz);
  }
  
  if (!t.isValid())
  {
    bluetooth.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    bluetooth.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    bluetooth.print(i<slen ? str[i] : ' ');
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
