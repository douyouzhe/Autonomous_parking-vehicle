/*
Author: YOUZHE DOU
Project: Self-parking car
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#define trigPinR 22 //ultrasonic sensor pin
#define echoPinR 23 //ultrasonic sensor pin
#define trigPinRB 26 //ultrasonic sensor pin
#define echoPinRB 27 //ultrasonic sensor pin
#define trigPinLB 28 //ultrasonic sensor pin
#define echoPinLB 29 //ultrasonic sensor pin
#define trigPinL 52 //ultrasonic sensor pin
#define echoPinL 53 //ultrasonic sensor pin
#define trigPinF 43 //ultrasonic sensor pin
#define echoPinF 42 //ultrasonic sensor pin
#define trigPinB 50 //ultrasonic sensor pin
#define echoPinB 51 //ultrasonic sensor pin
int enA = 8;/////For enabling motor and giving rotation speed hence pwm pins are best for this purpose.
int enB = 13;
int in1 = 10;////Digital Pins are used because we only need to
int in2 = 9;////give signal to motor whether to rotate or 
int in3 = 12;
int in4 = 11;
//Board                        Digital Pins Usable For Interrupts
//Mega, Mega2560, MegaADK     2, 3, 18, 19, 20, 21
int encoderPinLeft=2;
int encoderPinRight=3;
unsigned int rpmLeft;     // rpm reading
unsigned int rpmRight;     // rpm reading
volatile byte pulsesLeft;  // number of pulses
volatile byte pulsesRight;  // number of pulses
unsigned long timeold; 
// The number of pulses per revolution
// depends on your index disc!!
unsigned int pulsesperturn = 20;
int baseSpeedLeft=70;
int baseSpeedRight=70;
//unsigned int codeWheelCounter=0;
int mapSize=50;
int mapCellCount=5;
long thresHold=15;
int binaryMapLeft[50];
int binaryMapRight[50];
int binaryMapRightPara[50];
boolean findSpotPer=false;
boolean findSpotPara=false;
long rightDis,leftDis, frontDis, backDis;
int runTimes=0;
boolean perpedicular = true;
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);



void setup() {
  Serial.begin (9600);
  pinMode(trigPinR, OUTPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(trigPinRB, OUTPUT);
  pinMode(echoPinRB, INPUT); 
  pinMode(echoPinR, INPUT); 
  pinMode(echoPinL, INPUT);
  pinMode(echoPinF, INPUT);
  pinMode(echoPinB, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(encoderPinLeft, INPUT);
  pinMode(encoderPinRight, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinLeft), counterLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinRight), counterRight, RISING);
  pulsesLeft=0; 
  pulsesRight=0;
  rpmLeft=0;
  rpmRight=0;
  timeold=0;

  for(int i =0; i<50;i++){
    binaryMapRight[i]=1;
    binaryMapRightPara[i]=1;
  }
   if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  /* Display some basic information on this sensor */
  //displaySensorDetails();
//  sensors_event_t event; 
//  mag.getEvent(&event);
}

void loop() {

  while (runTimes<1){
    buildMap();
    if (perpedicular==true){
      doPerpedicularParking();
    }
    else{
      doParallelParking();
    }
   runTimes++;
  }
//   backDis = backDistance();
//    Serial.println(backDis);

}

/*
----------------------------------------------------------------Sensors start----------------------------------------------------------------
*/
/*
HC-SR04 Ping distance sensor]
Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
*/

long rightDistance(){
  long duration, distance;
  digitalWrite(trigPinR, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinR, LOW);
  duration = pulseIn(echoPinR, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}
long rightBackDistance(){
  long duration, distance;
  digitalWrite(trigPinRB, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinRB, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinRB, LOW);
  duration = pulseIn(echoPinRB, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}
long leftDistance(){
  long duration, distance;
  digitalWrite(trigPinL, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinL, LOW);
  duration = pulseIn(echoPinL, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}
long leftBackDistance(){
  long duration, distance;
  digitalWrite(trigPinLB, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinLB, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinLB, LOW);
  duration = pulseIn(echoPinLB, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

long frontDistance(){
  long duration, distance;
  digitalWrite(trigPinF, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinF, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinF, LOW);
  duration = pulseIn(echoPinF, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

long backDistance(){
  long duration, distance;
  digitalWrite(trigPinB, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinB, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinB, LOW);
  duration = pulseIn(echoPinB, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

void buildMap(){
  while((mapCellCount<mapSize)&&(findSpotPer==false)&&(findSpotPara==false)){
    long tmp=rightDistance();
    Serial.println(tmp);
    if (tmp>thresHold){
      binaryMapRightPara[mapCellCount]=0;
      if(tmp>thresHold+15){
        binaryMapRight[mapCellCount]=0;
      }
      findSpotPer=isSpotValidPer(mapCellCount);
      findSpotPara=isSpotValidPara(mapCellCount);
    }else{
    }
    moveABit();
    mapCellCount++;
  }
  moveABit();
  moveABit();
  moveABit();
  moveABit();
  mapCellCount=10;
}

boolean isSpotValidPer(int curPos){
  if( (binaryMapRight[curPos-1]==0)){
    return true;
  }
  else return false;
}

boolean isSpotValidPara(int curPos){
  if( (binaryMapRightPara[curPos-1]==0) &&(binaryMapRightPara[curPos-2]==0) &&(binaryMapRightPara[curPos-3]==0)){
    perpedicular=false;
    return true;
  }
  else return false;
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

float getHeading(){
  /* Get a new sensor event */ 
  //Serial.println("dqdew");
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
//  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.19;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
  //delay(500);
}

/*
----------------------------------------------------------------Sensors end----------------------------------------------------------------
*/
/*
----------------------------------------------------------------Motor start----------------------------------------------------------------
*/
void leftTurn90UsingHeading(){
  
  float curHeading=getHeading();
  float targetHeading = curHeading -40;// tune this value
  
  while(abs(curHeading-targetHeading)>3){//tune this value
    curHeading=getHeading();
    backDis = backDistance();
    //ghSerial.println(backDis);
    if(backDis<=6){
      break;
    }
    goSimpleBackward(0,200);
  }
  turnOff();
}
void leftTurn45UsingHeading(){
  float curHeading=getHeading();
  float targetHeading = curHeading -37;// tune this value
  while(abs(curHeading-targetHeading)>3){//tune this value
    curHeading=getHeading();
    goSimpleBackward(0,200);
  }
  turnOff();
}
void rightTurn45UsingHeading(){
  float curHeading=getHeading();
  float targetHeading = curHeading +16;// tune this value
  while(abs(curHeading-targetHeading)>3){//tune this value
    backDis = backDistance();
    Serial.println(backDis);
    if(backDis<=4){
      break;
    }
    curHeading=getHeading();
    goSimpleBackward(200,0);
  }
  turnOff();
}




//void leftTurn90(int cnt){
//  pulsesLeft=0;
//  while(pulsesLeft<cnt){
//    goSimpleBackward(0,160);
//  }
//  turnOff();
//  pulsesLeft=0;
//}
//
//void rightTurn90(int cnt){
//  while(pulsesRight<cnt){
//    goSimpleBackward(200,0);
//  }
//  turnOff();
//  pulsesRight=0;
//}
void gogogoinline(int cnt){
   pulsesRight=0;
   pulsesLeft=0;
   long tmp = rightBackDistance();
   while(pulsesRight<cnt or pulsesLeft<cnt){
    tmp = rightBackDistance();
    if(tmp<=12){
      break;
    }
    int ls=0;
    int rs=0;
    int a = 140;
    if(pulsesRight<cnt){
      rs = a;
    }
    if(pulsesLeft<cnt){
      ls = a;
    }
    goSimpleBackward(rs,ls);
   }
   turnOff();
   pulsesRight=0;
   pulsesLeft=0;
}

void doPerpedicularParking(){
    Serial.println("per");
    moveABit();
    leftTurn90UsingHeading();
    backDis = backDistance();
    while(backDis>10){
      backDis = backDistance();
      goSimpleBackward(90,90);
    }
    turnOff();
}

void doParallelParking(){
    Serial.println("para");
    leftTurn45UsingHeading();
    turnOff();
    delay(500);
    gogogoinline(27);
    turnOff();
    delay(500);
    rightTurn45UsingHeading(); 
    delay(500);  
    moveABit();
    //[moveABit();
    //moveABit();
    turnOff();
}

void moveABit(){
  int delay1=500;
  int speed=100;
  goSimpleFoward(70,100);
  delay(delay1);
  turnOff();
  delay(delay1-200);
}
void counterLeft(){pulsesLeft++;}
void counterRight(){pulsesRight++;}
void turnOff(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  baseSpeedLeft=70;
  baseSpeedRight=70;
}
unsigned int getRpmLeft(){
  if (millis() - timeold >= 1000){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
    detachInterrupt(0);
    rpmLeft = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulsesLeft;
    timeold = millis();
    pulsesLeft = 0;
    Serial.print("Left RPM = ");
    Serial.println(rpmLeft,DEC);
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), counterLeft, RISING);
    return rpmLeft;
  }
}

unsigned int countLeftWheel(){
  if (millis() - timeold >= 1000){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
    detachInterrupt(0);
    rpmLeft = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulsesLeft;
    timeold = millis();
    pulsesLeft = 0;
    Serial.print("Left RPM = ");
    Serial.println(rpmLeft,DEC);
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), counterLeft, RISING);
    return rpmLeft;
  }
}

unsigned int getRpmRight(){
  
}

void setRpmLeft(unsigned int targetRpmLeft){
  unsigned int rpmNow = getRpmLeft();
  if (rpmNow<targetRpmLeft){
    baseSpeedLeft=baseSpeedLeft+5;
    setSpeedLeft(baseSpeedLeft);
  }else if(rpmNow>targetRpmLeft){
    baseSpeedLeft=baseSpeedLeft-5;
    setSpeedLeft(baseSpeedLeft);
  }
  
}
void setRpmRight(unsigned int targetRpmRight){
  
}

void setSpeedLeft(int speed){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
}

void setSpeedRight(int speed){
  
}

void goSimpleFoward(int speedLeft, int speedRight){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, speedLeft);
  analogWrite(enB, speedRight);
}
void goSimpleBackward(int speedLeft, int speedRight){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, speedLeft);
  analogWrite(enB, speedRight);
}
/*
----------------------------------------------------------------Motor end----------------------------------------------------------------
*/



 