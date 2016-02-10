//Worked on 2/10/16
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>//LCD

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>

#ifndef Definitions

#define chanel_number 6  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 4  //set PPM signal output pin on the arduino
#define FowardReverse 0 //set foward and reverse channel
#define LeftRight 1 //set left and right channel

#define straight 1500
#define slightLeft 1600
#define slightRight 1400
#define hardLeft 1800
#define hardRight 1200

#define increment 10

#define neutral 1500  

#define COMPASSTOLERANCE 20

#endif // ! Definitions

struct WAYPOINT {
  double latitude;
  double longitude;
  float courseToWaypoint;
  float distance;
  int estimatedArrivalTime;
};







LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
LiquidCrystal lcd2(8, 7, 10, 11, 12, 13);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_Simple_AHRS          ahrs(&accel, &mag);
TinyGPSPlus gps;
SoftwareSerial bt(5, 6); //tx rx
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
WAYPOINT waypoint[15];
int waypointCounter = 0;
int routeCounter = 0;
float angle;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
static void getGPS(unsigned long ms);
void lcd1_print_data(String label, float data_value, int precision, int line_number , int column);
void lcd2_print_data(String label, float data_value, int precision, int line_number , int column);

void setup()
{
  //Compass Modules Initialization
  accel.begin();
  mag.begin();
  bmp.begin();

  //GPS Serial
  Serial.begin(9600);
  //Bluetooth Serial
  bt.begin(1200);
  //Reserve space for incoming string 
  inputString.reserve(200);

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
  

  //LCD Screens for testing 
  lcd.begin(16, 2);
  String x = "LCD 1 Start"; 
  lcd1_print_data(x, -999, 0, 0 ,0);
  lcd2.begin(16, 2);
  lcd2_print_data("LCD 2 Start", -999, 0, 0 ,0);

}


//Calculate the heading from the compass 
//Set the global variable angle to the propper heading 
void get_heading()
{
  sensors_vec_t   orientation;
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
/*
void print_GPS()
{

  if (gps.location.isValid())
  {
    lcd2_print_data("Lat", gps.location.lat(), 6, 0);
    lcd2_print_data("Lon", gps.location.lng(), 6, 1);
  }
  else
  {
    lcd2_print_data("Lat:Invalid", -999, 0, 0);
    lcd2_print_data("Lon:Invalid", -999, 0, 1);
  }
}*/


void loop()
{
 
  /*For testting to make sure the code is running 
   * 
   counter++; 
  lcd1_print_data("C", counter, 0, 0);
  */ 
  
  //sets the global variable angle to the proper heading 
  get_heading();
  //prints Heading using the global variable angle 
  lcd1_print_data("H", angle, 0, 0 , 0);
  //for testing lets the user see the heading 
  
  //Get the gps data from the module and update the GPS object with that data 
  getGPSData(); 
  /*
   * 
   *prints GPS LAT LON on screen two 
   *print_GPS(); 
 */ 
   
  
//   * Waypoint data print to lcd 1
   
   lcd1_print_data("CurWP", routeCounter, 0, 2 , 0);
   lcd1_print_data("TotWP", waypointCounter, 0, 2 , 7);

 
   
  if (followingGPS)
  {
    wichWay();
    lcd1_print_data("C", angle, 0, 0 , 7);
    lcd2_print_data("D", waypoint[routeCounter].distance, 0, 1 , 9);
    bluetooth();
  } else
    bluetooth();

  //Set the values for the ppm 
   ppm[FowardReverse] = FRValue;
   ppm[LeftRight] = LRValue;


    //Prints the current wheel position 
    if(LRValue < 1500)
      lcd2_print_data("Right", -999, 0, 1 , 0);
    else if (LRValue > 1500) 
      lcd2_print_data("Left", -999, 0, 1 , 0);
    else
      lcd2_print_data("Straight", -999, 0, 1 , 0);

   
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
 * Calls the Serial even function 
 * processes the data from the serial even 
 * Needs Global String Complete and inputString 
 */
void bluetooth()
{
  serialEvent();
  if (stringComplete) {
   // lcd1_print_data(inputString, 0, 0, 1);
    lcd2_print_data(inputString, -999, 0, 0 ,0);
    //delay(10);
    //lcd2_print_data(inputString, -999, 0,1);
    //Serial.println("Here"); 
    //delay(1000); 
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
      getGPS(1000);
      if(gps.location.lat() != 0.0)
      {
         bt.print(gps.location.lat(), 6);
         bt.print(",");
         bt.print(gps.location.lng(), 6);
         bt.print("\n");
      }
    } else if (inputString == "~b\n") {
      //decrease the current values 
      slowForward -= increment;
      fastForward -= increment;
      slowBackward  += increment;
      fastBackward  += increment;
      //limit on decreasing 
      //Resets when values get tooo high 
      if (slowForward < 1510)
      {
        slowForward = 1600;
        fastForward = 1600;
        slowBackward = 1400;
        fastBackward = 1400;
      }
    } else if (isdigit(inputString[0])) {
       String waypointNumber = inputString;
      //delay(10);
      String latitude = inputString;
      //delay(10);
      String longitude = inputString;
      //delay(10);
      //find where the first comma is at
      int one = inputString.indexOf(',');
      //find where the second comma is at
      int two = inputString.lastIndexOf(',');

      //trim the string to just the number
      waypointNumber.remove(one);
      latitude.remove(0, one + 1);
      latitude.remove(two - 2);
      longitude.remove(0, two + 1);

      char *ptr;
      waypoint[waypointCounter].latitude = (double)strtod(latitude.c_str(), &ptr);
      delay(10);
      waypoint[waypointCounter].longitude = (double)strtod(longitude.c_str(), &ptr);



      waypointCounter++;
      if (waypointCounter > 10)
        waypointCounter = 0;
      
      //void lcd1_print_data(String label, float data_value , int precision , int line_number , int column)
     // lcd1_print_data("Lat",  waypoint[waypointCounter - 1].latitude , 6 , 0 , 0); 
     // lcd1_print_data("Lon",  waypoint[waypointCounter - 1].longitude , 6 , 1 , 0); 
      
    } else {
      /*followingGPS = false;
      LRValue = straight;
      FRValue = neutral;
    */
    }


    inputString = "";
    stringComplete = false;
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

void getGPSData()
{


  getGPS(100);

  waypoint[routeCounter].distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      waypoint[routeCounter].latitude,
      waypoint[routeCounter].longitude);
  //Serial.print(waypoint[routeCounter].distance);
  //printInt(distance, gps.location.isValid(), 9);
//void lcd1_print_data(String label, float data_value, int precision, int line_number , int column);
  
  //Prints the distance to the 
  //lcd1_print_data("D " , waypoint[routeCounter].distance , 0 , 0 ,0); 

  waypoint[routeCounter].courseToWaypoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      waypoint[routeCounter].latitude,
      waypoint[routeCounter].longitude);
}

void lcd1_print_data(String label, float data_value , int precision , int line_number , int column)
{
  if ((line_number == 0) && (column == 0))
  {
    lcd.clear();
  }
    lcd.setCursor(column, line_number);
    lcd.print(label);

    if (data_value != -999)
    {
      lcd.print(" ");
      lcd.print(data_value, precision);
    }else 
    lcd.print("                  "); 
     delay(100); 
}

void lcd2_print_data(String label, float data_value, int precision, int line_number ,int column)
{
  if ((line_number == 0) && (column == 0))
  {
    lcd2.clear();
  }
  lcd2.setCursor(column, line_number);
  lcd2.print(label);
  if (data_value != -999)
  {
    lcd2.print(" ");
    lcd2.print(data_value, precision);
  }else
  lcd2.print("       "); 
  delay(100); 
}

void wichWay()
{
/*
   lcd.setCursor(0,0);
   lcd.print("H"); 
   lcd.print(angle);
*/
  /*
  Serial.print("  Compas = ");
  Serial.print(angle);
  Serial.print("   going to  = ");
  Serial.print(waypoint[routeCounter].courseToWaypoint);


  
  Serial.print("   d = ");
  Serial.print(waypoint[routeCounter].distance);
  Serial.print("    wC  ");
  Serial.print(waypointCounter); 
  Serial.print("   rC   ");
  Serial.print(routeCounter); 
  Serial.print("   goint to lat   ");
  Serial.print(waypoint[routeCounter].latitude , 6); 
  Serial.print("  lon   ");
  Serial.print(waypoint[routeCounter].longitude , 6);
  Serial.print("   current lat   ");
  Serial.print(gps.location.lat(), 6); 
  Serial.print("  lon   ");
  Serial.println(gps.location.lng(), 6);
  */
/*
 * --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

  
  float newCourse;
  if ((waypoint[routeCounter].courseToWaypoint <= angle + COMPASSTOLERANCE) &&
      (waypoint[routeCounter].courseToWaypoint >= angle - COMPASSTOLERANCE)) {
    LRValue = straight;//go straight
  }
  else if (waypoint[routeCounter].courseToWaypoint <= 180) { //---------------- waypoint is in 2/3 qudrant
    newCourse = angle - waypoint[routeCounter].courseToWaypoint ;
    if (newCourse < 0)
      newCourse += 360;
    if (newCourse == 180) {
      //Serial.print("Turn around");
      LRValue = hardLeft;
    } else if (newCourse < 180)
    {
      LRValue = slightLeft;
    }
    else if (newCourse > 180)
    {
      LRValue = slightRight;
    }
  }
  else if (waypoint[routeCounter].courseToWaypoint > 180) { //-----------------------waypoint is in 1/4 qudrant
    newCourse = (angle + (360 - waypoint[routeCounter].courseToWaypoint));
    if (newCourse > 360)
      newCourse -= 360;
    if (newCourse == 180) {
      //Serial.print("Turn around");
      LRValue = hardLeft;
    } else if (newCourse < 180)
    {
      //Serial.print("Go left  ");
      //Serial.print(newCourse);
      //Serial.println();
      LRValue = slightLeft;
    }
    else if (newCourse > 180)
    {
      //Serial.print("Go right   ");
      //Serial.print(360 - newCourse);
      //Serial.println();
      LRValue = slightRight;
    }
  }
  
  if (waypoint[routeCounter].distance >= 7) {
    FRValue = slowForward;
    //else if(waypoint[routeCounter].distance < 20)
    //FRValue = slowForward;
  } else if (waypoint[routeCounter].distance < 7) {
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

static void getGPS(unsigned long ms)
{ 
  if (ms > 0)
  {
    if (Serial.available())
    {
      while ((Serial.available()))
      {
        gps.encode(Serial.read());
      }
      return;
    }
    else
      getGPS(ms - 1);
  }
  else
    return; 


}

