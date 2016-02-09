
volatile int count = 0;
int total = 0;
char data[14];
int dataPin = 4;
int gauge = 0; // display of calipers reading
int factor = 0;
unsigned long sec;


void setup(){
pinMode(dataPin, INPUT); //DATA line goes to Arduino digital pin 4
Serial.begin(9600);
delay(500);
//attachInterrupt(0,getBit,FALLING); //CLOCK line goes to Arduino digital pin 2

}

void loop(){
int i,time;
for(i=2;i<14;i++){ //initialize array of bits to 0
data[i]=0;
}
do{time = pulseIn(2, LOW);} //repeatedly get the length of LOW pulses until one is longer than 130uS
while(time < 130);

attachInterrupt(0,getBit,FALLING); //start watching data line

while(count < 14){} //wait till all bits are filled in

detachInterrupt(0);
count = 0;
for(i=2;i<14;i++){


}

gauge = 0; //the following is code for converts least significant bit first binary to decminal, units are thousandths of an inch
factor = 1;
for(i=2; i<14; i++){
gauge += data[i]*factor;
factor = factor*2;
}

Serial.print("Time (sec):");
sec = millis();
Serial.print(sec/1000);
Serial.print(" Inches:");
Serial.print(gauge);

delay(1000);
Serial.println();
}


void getBit(){
char sample = 0; //variable used for "triple sampling"
if(digitalRead(dataPin) == HIGH) //here the dataPin is checked three times for a HIGH value.
sample++;
if(digitalRead(dataPin) == HIGH)
sample++;
if(digitalRead(dataPin) == HIGH)
sample++;
if(sample > 1) //if the pin was HIGH at least twice, a 1 is recorded
data[count] = 1;
count++; //increment count so main() knows when the entire string of bits is ready
}
