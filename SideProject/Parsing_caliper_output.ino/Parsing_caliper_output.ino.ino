
//Simple Digital Calliper Reader
//See http://j44industries.blogspot.com/
#include<stdio.h>
//#include<math.h>
#define ARRAYLENGTH 24;
// Pin Declarations
int dataIn = 11;
int clockIn = 12;

// Variables
int clock = 1;
int lastClock = 1;
unsigned long time = 0;
unsigned long timeStart = 0;
int out = 0;
int x[24] = {0};
int arrayCounter = 0;

void printTheNumber();

void setup() {
  // Pin Set Up
  pinMode(dataIn, INPUT);     
  pinMode(clockIn, INPUT);  

  
  Serial.begin(9600);
  Serial.println("Ready: ");
}

int array[25] = {1,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0}; 







void loop(){

  lastClock = clock;
  clock = digitalRead(clockIn);
  
  
  

  if (lastClock == 1 && clock == 0){
    out = digitalRead(dataIn)+digitalRead(dataIn)+digitalRead(dataIn); // Tripple sampling to remove glitches
    if((micros() - time) > 800){
      Serial.println(" ");///------------------------------New Line
      //printTheNumber();
      arrayCounter = 0;
    }
    else if((micros() - time) > 400){
      Serial.print("  ");
    }

    if (out > 1){
      Serial.print("1");
      x[arrayCounter] = 1;
      arrayCounter++;
      
    }
    else{
      Serial.print("0");
      x[arrayCounter] = 0;
      arrayCounter++;
    }
    Serial.print(",");
    time = micros();
  }

  /*for(int counter = 0; counter <= 24; counter++)
    Serial.print(array[counter]); 
    

  int x[6] = {1,1,1,1,1,1}; 


  int n =6;
  int dec=0; 
  int j=0;


  for(int i=(n-1);i>=0;i--) 
    {
        dec += (x[i]*power(2,j));
        j++;
    }

  Serial.println(dec); 

  */
}


int power(int c, int d)
{
    int pow=1;
    int i=1;
    while(i<=d) 
    {
        pow=pow*c;
        i++;
    }
    return pow;
}



void printTheNumber() {
int i;
  for (i = 0; i < 24; i++ ) {
    Serial.print(x[i]);
  }
  Serial.println("");
  /*
  int n = ARRAYLENGTH;
  int dec=0; 
  int j=0;


  for(int i=(n-1);i>=0;i--) 
    {
        dec += (x[i]*power(2,j));
        j++;
    }
  Serial.println(dec);
  */ 
}

void convertToChar(int binaryChar[],int LENGHT)
{
  int a[100]; 
  int n,dec=0; 
  int j=0,f;
  for(int i=(n-1);i>=0;i--) 
    {
        dec=(a[i]*pow(2,j))+dec;
        j++;
    }
}


