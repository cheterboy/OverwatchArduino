/*
   need to fix the bluetooth coordinate converter
   Proper inout, proper string input

   When i try to convert more than one coordinate it freaks out
*/





#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
//#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>



#define chanel_number 6  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 4  //set PPM signal output pin on the arduino
#define FowardReverse 0 //set foward and reverse channel
#define LeftRight 1 //set left and right channel


int straight  = 1500;

int slightLeft = 1000;
int hardLeft = 1000;

int hardRight = 2000;
int slightRight  = 2000;


#define increment 10

#define neutral 1500

#define COMPASSTOLERANCE 20
#define maxWaypoints 10


struct WAYPOINT {
  float latitude = 0.0;
  float longitude  = 0.0;
  float courseToWaypoint = 0.0;
  float distance = 0.0;
  String string = "0,000.00000,000.00000  ";
};



Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
//  Adafruit_BMP085_Unified       bmp(18001);
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

TinyGPSPlus gps;
SoftwareSerial bt(5, 6); //bt tx rx
SoftwareSerial bt2(2, 3); //tx rx

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int slowForward = 1550;
int fastForward = 1550;
int slowBackward = 1300;
int fastBackward = 1300;
int ppm[chanel_number];
int FRValue = 1500;
int LRValue = 1500;
bool followingGPS = false;
WAYPOINT waypoint[maxWaypoints];
int waypointCounter = 0;
int routeCounter = 0;
float angle;
//float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
void   getGPSChar(void);
void   btEvent();
String holder , holder2;
int index;
char *ptr, *ptr2;



void getGPSData();
void wichWay(); 



void setup()
{
  //Compass Modules Initialization
  accel.begin();
  mag.begin();
  //  bmp.begin();

  //GPS Serial
  Serial.begin(9600);
  //Bluetooth Serial


  bt.begin(9600);//////////////////////////////ending








  
  bt2.begin(38400);
  //Reserve space for incoming string
  inputString.reserve(50);


  waypoint[0].string.reserve(50);
  waypoint[1].string.reserve(50);
  waypoint[2].string.reserve(50);
  waypoint[3].string.reserve(50);
  waypoint[4].string.reserve(50);



  //initialize each ppm chanell to default
  for (int i = 0; i < chanel_number; i++) {
    ppm[i] = default_servo_value;
  }

  //initialize ppm pin
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

  //set the timer for the ppm
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

 
  bt.listen();

}



sensors_vec_t   orientation;
void get_heading()
{
  if (ahrs.getOrientation(&orientation))
  {
    orientation.roll;
    orientation.pitch;
    angle = orientation.heading;

    if (angle < 0)
      angle += 360;

    angle -= 360;
    angle = abs(angle);

  }

}

unsigned long start = millis();
void loop()
{
  bluetooth();
  getGPSData();
  // get_heading();

  if (followingGPS)
  {
    if (millis() - start > 500)
    {
      bt2.print("WP ");
      bt2.print(routeCounter);
      bt2.print(" out of(");
      bt2.print(waypointCounter);
      bt2.print(") D ");
      bt2.println(waypoint[routeCounter].distance);


      bt2.print("C ");
      bt2.print(angle);
      bt2.print("H ");
      bt2.println(waypoint[routeCounter].courseToWaypoint);
      start = millis();
    }
    wichWay();

  }

  ppm[FowardReverse] = FRValue;
  ppm[LeftRight] = LRValue;

}

ISR(TIMER1_COMPA_vect) { //leave this alone
  static boolean state = true;

  TCNT1 = 0;

  if (state) { //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else { //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    digitalWrite(sigPin, !onState);
    state = true;

    if (cur_chan_numb >= chanel_number) {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;//
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}

/*
   Calls the Serial even function
   processes the data from the serial even
   Needs Global String Complete and inputString
*/
void bluetooth()
{
  btEvent();
  if (stringComplete) {
    bt2.print(inputString);
    
    
    if (inputString == "~forward\n") {
      FRValue = slowForward;
    } else if (inputString == "~notforward\n") {
      FRValue = neutral;
    } else if (inputString == "~backward\n") {
      FRValue = slowBackward;
    } else if (inputString == "~notbackward\n") {
      FRValue = neutral;
    } else if (inputString == "~left\n") {
      LRValue = hardLeft;
    } else if (inputString == "~notleft\n") {
      LRValue = straight;
    } else if (inputString == "~right\n") {
      LRValue = hardRight;
    } else if (inputString == "~notright\n") {
      LRValue = straight;
    } else if (inputString == "~stoproute\n") {
      followingGPS = false;
      LRValue = straight;
      FRValue = neutral;
      routeCounter = 0;
      waypointCounter = 0;
    } else if (inputString == "~startroute\n") {

      if (waypointCounter > 0)
      {
        for (int counter = 0; counter < waypointCounter; counter++)
        {
          bt2.print("String ");
          bt2.print(counter);
          bt2.print(": ");
          //bt2.print(waypoint[counter].string);

          bt2.println(waypoint[counter].string);

        }
        for (int counter = 0; counter < waypointCounter; counter++)
        {

          //*ptr = NULL;
          //*ptr2 = NULL;
          bt2.print(counter);
          bt2.print(" ");
          bt2.print(waypoint[counter].string);


          holder = waypoint[counter].string;
          delay(10);
          holder2 = waypoint[counter].string;

          delay(10);
          index = holder.indexOf(',');
          delay(10);

          holder.remove(0, index + 1);
          delay(10);
          index = holder.lastIndexOf(',');
          delay(10);
          holder.remove(index );
          delay(10);

          bt2.println(holder);
          delay(10);
          waypoint[counter].latitude = (double)strtod(holder.c_str(), &ptr2);
          delay(10);



          index = holder2.lastIndexOf(',');
          delay(10);
          holder2.remove(0, index + 1);
          delay(10);
          bt2.println(holder2);
          delay(10);
          waypoint[counter].longitude = (double)strtod(holder2.c_str(), &ptr);
          delay(10);

        }

        for (int counter = 0; counter < waypointCounter; counter++)
        {
          bt2.print(counter);
          bt2.print(" ");
          //bt2.print(waypoint[counter].string);

          bt2.print(waypoint[counter].latitude, 5);
          bt2.print(" ");
          bt2.println(waypoint[counter].longitude, 5);
        }

        routeCounter = 0;
        followingGPS = true;
      } else
        followingGPS = false;

    } else if (inputString == "~a\n") {
      //Increment the current value
      slowForward += increment;
      fastForward += increment;
      slowBackward  -= increment;
      fastBackward  -= increment;
      //Limit for the increment
      //Resets the values if gets too high
      if (slowForward > 2000)
      {
        slowForward = 1600;
        fastForward = 1600;
        slowBackward = 1400;
        fastBackward = 1400;
      }
    } else if (inputString == "~uadlocation\n") {
      getGPSData();
      if (gps.location.lat() != 0.0)
      {
        bt.print(gps.location.lat(), 6);
        bt.print(",");
        bt.print(gps.location.lng(), 6);
        bt.print("\n");
      }
    } else if (inputString == "~b\n") {

      if (slightLeft == 2000)
      {
        slightLeft = 1000;
        hardLeft  = 1000;
        slightRight  = 2000;
        hardRight  = 2000;
      } else
      {
        slightLeft = 2000;
        hardLeft  = 2000;
        slightRight  = 1000;
        hardRight  = 1000;
      }

    } else if (inputString == "~clearroute\n")
    {
      for (int counter = 0; counter < maxWaypoints ; counter ++)
      {
        waypoint[counter].latitude = 0;
        waypoint[counter].longitude = 0;
        waypoint[counter].courseToWaypoint = 0;
        waypoint[counter].distance = 0;
        waypoint[counter].string = "";
      }
      waypointCounter = 0;
      routeCounter = 0;
    } 


    inputString = "";
    stringComplete = false;
  }

}

char inChar;

void btEvent() {
  while (bt.available()) {
    // get the new byte:
    inChar = (char)bt.read();

    // bt2.print(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;

      
      if (isdigit(inputString[0])) {

        waypoint[waypointCounter].string = inputString;
        //delay(10);
        waypointCounter++;
        Serial.println(waypointCounter);
        //}
        inputString = "";
        stringComplete = false;
    }
  }
}}

void getGPSData()
{

  getGPSChar();

  waypoint[routeCounter].distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      waypoint[routeCounter].latitude,
      waypoint[routeCounter].longitude);

  waypoint[routeCounter].courseToWaypoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      waypoint[routeCounter].latitude,
      waypoint[routeCounter].longitude);
}

float newCourse;
void wichWay(){



  //if the course is withing the tollerance
  if ((waypoint[routeCounter].courseToWaypoint <= angle + COMPASSTOLERANCE) &&
      (waypoint[routeCounter].courseToWaypoint >= angle - COMPASSTOLERANCE)) {
    LRValue = straight;//go straight
  }
  else if (waypoint[routeCounter].courseToWaypoint <= 180) { //---------------- waypoint is in 2/3 qudrant
    newCourse = angle - waypoint[routeCounter].courseToWaypoint ;
    if (newCourse < 0)
      newCourse += 360;
    if (newCourse == 180) {
      LRValue = hardLeft;
    } else if (newCourse < 180)
    {
      //LRValue = slightLeft;
      LRValue = map(newCourse, 0 , 179 , straight , hardLeft);
    }
    else if (newCourse > 180)
    {
      //LRValue = slightRight;
      LRValue = map(newCourse, 181 , 360 , straight , hardRight);
    }
  }
  else if (waypoint[routeCounter].courseToWaypoint > 180) { //-----------------------waypoint is in 1/4 qudrant
    newCourse = (angle + (360 - waypoint[routeCounter].courseToWaypoint));
    if (newCourse > 360)
      newCourse -= 360;
    if (newCourse == 180) {
      LRValue = hardLeft;
    } else if (newCourse < 180)
    {
      // LRValue = slightLeft;
      LRValue = map(newCourse, 0 , 179 , straight , hardLeft);
    }
    else if (newCourse > 180)
    {
      //LRValue = slightRight;
      LRValue = map(newCourse, 181 , 360 , straight , hardRight);
    }
  }

  if (waypoint[routeCounter].distance >= 5) {
    FRValue = slowForward;
  } else if (waypoint[routeCounter].distance < 5) {
    routeCounter++;
    if (routeCounter >= waypointCounter) {
      FRValue = neutral;
      LRValue = straight;
      followingGPS = false;
      routeCounter = 0;
      waypointCounter = 0;
    }

  }
}


//Feeds gps data into the TinyGPS++ library
void getGPSChar()
{
  while (Serial.available())
  {
    gps.encode(Serial.read());
  }
}
