//电子尺读数
// gnd data clk vcc1.5
// from china

int dataIn = 3;
int clockIn = 2;

int isin=0; //英寸inch=1，mm=0
int isfs=0; //是负数
int index;

unsigned long xData,oData;//最终要输出的数据

int ledPin = 13; // the number of the LED pin
int ledState = LOW; // ledState used to set the LED
long previousMillis = 0; // will store last time LED was updated
long interval = 500; // interval at which to blink (milliseconds)

long previousGetMillis = 0; //上次中断发生时间
long Timeout = 8; //超时时间 8ms



void setup(){
digitalWrite(dataIn, 1);
digitalWrite( clockIn, 1);
pinMode(dataIn, INPUT); //DATA line goes to Arduino digital pin 4
pinMode(clockIn, INPUT);
Serial.begin(9600);
delay(500);
attachInterrupt(0,getBit,RISING); //CLOCK line goes to Arduino digital pin 2 上升沿触发
index =0;
xData=0;
oData=999;
}

void loop(){

//超时处理
if ((index !=0) && (millis() - previousGetMillis > Timeout) ) {
index=0;
xData=0;
};

//输出
if (index >23) {
if (oData !=xData) {
if (isfs==1)
Serial.print('-');

if (isin==1){ //英寸 inch
xData *=5;
Serial.print(xData/10000);
Serial.print('.');
if ((xData % 10000)<1000){
if ((xData % 10000)<100){
if ((xData % 10000)<10){
Serial.print('0');
};
Serial.print('0');
};
Serial.print('0');
};
Serial.println(xData % 10000);
}else { //公制 mm

Serial.print(xData/100);
Serial.print('.');
if ((xData % 100)<10) //补0
Serial.print('0');
Serial.println(xData % 100);
};
}; //if 公英制
oData =xData;
index=0;
xData=0;
};


if (millis() - previousMillis > interval) {
previousMillis = millis();
if (ledState == LOW)
ledState = HIGH;
else
ledState = LOW;
digitalWrite(ledPin, ledState);
}

}



void getBit(){
previousGetMillis=millis();
if(index < 20){
if(digitalRead(dataIn)==1){
xData|= 1<<index;

};
} else {

if (index==20) //第21位为符号位 -
isfs=digitalRead(dataIn);


if (index==23) //第24位为公英制 1=英制inch
isin=digitalRead(dataIn);

};

index++;
}

