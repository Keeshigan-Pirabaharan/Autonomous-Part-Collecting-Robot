/*

 MSE 2202 MSEBot base code for Labs 3 and 4
 Language: Arduino
 Authors: Michael Naish and Eugen Porter
 Date: 16/01/17
 
 Rev 1 - Initial version
 Rev 2 - Update for MSEduino v. 2
 
 */

#include <Servo.h>
#include <EEPROM.h>
//#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <SoftwareSerial.h>
#include "NewPing.h"

Servo RMotor;
Servo LMotor;
Servo BArm;
Servo SArm;

SoftwareSerial mySerial(2, 2); // RX, TX

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION

#define TRIGGER_PIN_1  A0
#define ECHO_PIN_1     A0
#define TRIGGER_PIN_2  A1
#define ECHO_PIN_2     A1
#define TRIGGER_PIN_3  A2
#define ECHO_PIN_3     A2
#define MAX_DISTANCE 400

#define addr 0x1E //I2C Address for The HMC5883

boolean bt_Motors_Enabled = true;

//port pin constants

const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;

const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Motor_Enable_Switch = 12;

const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments

const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;


//constants

// EEPROM addresses
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Display_Time = 500;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;

unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;

bool Direction = false; //Direction is false for left and true for right

// Beacon Servo BA = BeaconServo
const int BA = 11;
const int RightSide = 0;
const int LeftSide = 180;

// Arm Servo SA = ServoArm
const int SA = 10;
unsigned int SArmtimerS = 10;
unsigned int SArmtimerE = 0;
int i = 0;
bool ArmBool = true;

//Ultrasonic Sensors
unsigned int LeftST = A0;
unsigned int LeftSE = A0;
unsigned int MidST = A1;
unsigned int MidSE = A1;
unsigned int RightST = A2;
unsigned int RightSE = A2;

//UltraS Additional
long duration1, distance1;
long duration2, distance2;
long duration3, distance3;
long actionDs = 15;
long extDs = 30;
float soundsp = 331.4;;  // Stores calculated speed of sound in M/S
float soundcm = soundsp / 10000;;  // Stores calculated speed of sound in cm/ms
int iterations = 2;

//Compass Directions
int x,y,z;

//BEACON
bool T = true;
bool S = true;
int startB = 0;
int endB = 0;
int startB1 = 0;
int endB1 = 0;
int startB2 = 0;
int endB2 = 0;
const int RotL = 1600; 
const int RotR = 1400;
const int RotL2 = 1400; //Opposite direction of rotation
const int RotR2 = 1600;
unsigned int LSF = 1660;
unsigned int RSF = 1650;

//BUMBERS
int bprL = 3;
int bprR = 11;
int countB = 1;
int countB1 = 1;
int startBPR = 0;
int endBPR = 0;
int startBPR1 = 0;
int endBPR1 = 0;
bool BPRON = true;

//bool right = true;
//bool nxtR = false;
//bool nxtL = false;
//bool ckg = true;
bool fwd = true;
int count1 = 1;
//int count2 = 1;
int count3 = 1;
int count4 = 1;
int count5 = 1;
int startN = 0;
int endN = 0;
//int tElap;
bool NDDir = true;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,65536};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

 
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(ci_Charlieplex_LED1,ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3,ci_Charlieplex_LED4,ci_Mode_Button);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

   mySerial.begin(2400); // IR decoder
  
  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  RMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  LMotor.attach(ci_Left_Motor);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  
  // Set up Shovel Servo
  pinMode(SA, OUTPUT);
  SArm.attach(SA);

  // Set up Beacon Servo
  pinMode(BA, OUTPUT);
  BArm.attach(BA);

  //Set up Compass
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();
  
  //setup bumper switches
  pinMode(bpr1, INPUT);
  pinMode(bpr2, INPUT);
}

void loop()
{
  if((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if(CharliePlexM::ui_Btn)
  {
    if(bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);
  Serial.println("Waiting   "); 
  Serial.println(ui_Robot_State_Index);
  
  // modes 
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch(ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
    {
      /*
      if (mySerial.available())
  {
    Serial.write(mySerial.read());
      }*/
      SArm.write(180);
    UltraSensors();
       // Send results to Serial Monitor
  
    Serial.print("Distance 1: ");
 
    if (distance1 >= 400 || distance1 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance1);
    Serial.print(" cm ");
    }
    
    Serial.print("Distance 2: ");
 
    if (distance2 >= 400 || distance2 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance2);
    Serial.print(" cm");
    }

    Serial.print("Distance 3: ");
 
    if (distance3 >= 400 || distance3 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance3);
    Serial.print(" cm ");
    }
  
      Serial.println(" ");
      
     
      
      //ArmSwing();
      SArm.write(0);
      //Compass
      ReadCompass();
      Serial.print("X Value: ");
      Serial.println(x);
      Serial.print("Y Value: ");
      Serial.println(y);
      Serial.print("Z Value: ");
      Serial.println(z);
      Serial.println();
      
      delay(2000);

      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
      ui_Mode_Indicator_Index = 0;
      break;
    } 
  
    case 1:    //Robot Run after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
/*
#ifdef DEBUG_ENCODERS           
        l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
        l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

        Serial.print("Encoders L: ");
        Serial.print(l_Left_Motor_Position);
        Serial.print(", R: ");
        Serial.println(l_Right_Motor_Position);
#endif*/

       // set motor speeds
        ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
        ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);

//ENTER YOUR CODE HERE STEVEN

       ObstacleAvoidance(); 
       //BeaconSensor();
     
 
      
      
      
    

#ifdef DEBUG_MOTORS
        Serial.print("Motors enabled: ");
        Serial.print(bt_Motors_Enabled);
        Serial.print(", Default: ");
        Serial.print(ui_Motors_Speed);
        Serial.print(", Left = ");
        Serial.print(ui_Left_Motor_Speed);
        Serial.print(", Right = ");
        Serial.println(ui_Right_Motor_Speed);
#endif    
        ui_Mode_Indicator_Index = 1;
      }
      break;
    } 


    case 4:    //Calibrate motor straightness after 3 seconds.
    {

      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          encoder_LeftMotor.zero();
          encoder_RightMotor.zero();
          ul_Calibration_Time = millis();
          LMotor.writeMicroseconds(ui_Motors_Speed);
          RMotor.writeMicroseconds(ui_Motors_Speed);
        }
        else if((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time) 
        {
          LMotor.writeMicroseconds(ci_Left_Motor_Stop); 
          RMotor.writeMicroseconds(ci_Right_Motor_Stop); 
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
          if(l_Left_Motor_Position > l_Right_Motor_Position)
          {
           // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = 0;
            ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;  
          }
          else
          {
           // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
            ui_Left_Motor_Offset = 0;
          }
          
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Motor Offsets: Left = ");
          Serial.print(ui_Left_Motor_Offset);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Offset);
#endif              
          EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));
          
          ui_Robot_State_Index = 0;    // go back to Mode 0 
        }
#ifdef DEBUG_MOTOR_CALIBRATION           
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif        
        ui_Mode_Indicator_Index = 4;
      } 
      break;
    }  
  }

  if((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY  
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    //CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    //Indicator();
  }
} 

// set mode indicator LED state

void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED,!(ui_Mode_Indicator[ui_Mode_Indicator_Index] & 
                      (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// Reads if IR Sensors sees an obstacle
/*
void readIRSensors()
{ 
  if(analogRead(LeftF) > 100)
    LF = LOW;
  else
    LF = HIGH; 
  if(analogRead(MidF) > 100)
    MF = LOW;
  else
    MF = HIGH;
  if(analogRead(RightF) > 100)
    RF = LOW;
  else
    RF = HIGH;
}
*/

// Measure distance to target using ultrasonic sensor (x3) 
void UltraSensors()
{
  // Measure duration for first sensor
    
  duration1 = sonar1.ping_median(iterations);
  
  // Add a delay between sensor readings
  
  // Measure duration for second sensor
  
  duration2 = sonar2.ping_median(iterations);

  // Measure duration for third sensor
  
  duration3 = sonar3.ping_median(iterations);
  
  // Calculate the distances for three sensors
  
  distance1 = (duration1 / 2) * soundcm;
  distance2 = (duration2 / 2) * soundcm;
  distance3 = (duration3 / 2) * soundcm;
}

// Emptys the shovel by bringing rotating the shovel arm to the desired position
void ArmSwing()
{
  if(ArmBool == true)
  {
  ArmBool = false;
  for(i = 0; i < 130;)
  {
    SArm.write(i++);
    delay(20);
    if(i == 129)
      SArmtimerS = millis();
  }
  }
  SArmtimerE = millis();
  if((SArmtimerE - SArmtimerS) > 1000)
  {
    SArmtimerS = 10;
    SArmtimerE = 0;
    ArmBool = true;
    SArm.write(0);
  } 
}

void ReadCompass()
{
  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  
 
 //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(addr, 6);
  if(6<=Wire.available())
  {
    x = Wire.read()<<8; //MSB  x 
    x |= Wire.read(); //LSB  x
    z = Wire.read()<<8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read()<<8; //MSB y
    y |= Wire.read(); //LSB y
  }
}

void TestUltra()
{
  UltraSensors();
       // Send results to Serial Monitor
  
    Serial.print("Distance 1: ");
 
    if (distance1 >= 400 || distance1 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance1);
    Serial.print(" cm ");
    }
    
    Serial.print("Distance 2: ");
 
    if (distance2 >= 400 || distance2 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance2);
    Serial.print(" cm");
    }

    Serial.print("Distance 3: ");
 
    if (distance3 >= 400 || distance3 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance3);
    Serial.print(" cm ");
    }
  
    Serial.println(" ");
  
}

void ObstacleAvoidance(){
  UltraSensors();
  if(count1%2!=0){ //set start time once
        startN = millis();
        count1++;
      }
      endN = millis(); 
      if((endN - startN)>3000 && count1%2==0 && fwd == true){ // every time 2 seconds passes turn in alternating direction
        if(NDDir){ //turn right for a bit every other time
          LMotor.writeMicroseconds(RotL); 
          RMotor.writeMicroseconds(RotR);
          delay(1000);
          NDDir=false;
        }else if(NDDir!=true){ //turn left a bit every other time
          LMotor.writeMicroseconds(RotL2);
          RMotor.writeMicroseconds(RotR2);
          delay(1000);
          NDDir=true;
        }
        count1++; //increment count to allow start to be reset
      }
      
     
      if(distance1<actionDs && distance2>=actionDs && distance3>=actionDs || distance1<extDs && distance2<actionDs && distance3>=actionDs || distance1>=actionDs && distance2<actionDs && distance3>=actionDs){ //only left sensor sees something OR middle and left w/ extended, turn right until all clear then prompt next stage
        //fwd = false; //only left sensor sees something OR middle and left w/ extended, OR just middle sensor, turn right until all clear then prompt next stage
        Serial.println("case2");
        LMotor.writeMicroseconds(RotL); 
        RMotor.writeMicroseconds(RotR);
        fwd = false;
      }
      else if(distance1>=actionDs && distance2>=actionDs && distance3<actionDs || distance1>=actionDs && distance2<actionDs && distance3<extDs || distance1<actionDs && distance3<actionDs){ 
       //fwd = false; //only right sensor sees something OR middle and right w/ extended, OR left and right (same as all 3) see smthng, turn left until all clear then prompt next stage
        Serial.println("case3");
        LMotor.writeMicroseconds(RotL2); 
        RMotor.writeMicroseconds(RotR2);
        fwd = false;
      }
      else if(distance1>=actionDs && distance2>=actionDs && distance3>=actionDs){ //all clear, move straight
        Serial.println("case1");
        LMotor.writeMicroseconds(LSF); 
        RMotor.writeMicroseconds(RSF);
        fwd = true;
      }
}

void BeaconSensor(){
    if(T == true){
      if(count3%2!=0){
        startB = millis();
        count3++;
      }
      endB = millis();
      endB1 = millis();
      //S = true;
      if((endB-startB)<1000){
        LMotor.writeMicroseconds(RotL); 
        RMotor.writeMicroseconds(RotR);
        Serial.println("rotating right");
      }
      if((endB-startB)>=1000){
        LMotor.writeMicroseconds(RotL2); 
        RMotor.writeMicroseconds(RotR2);
        Serial.println("rotating left");
        if(count4%2!=0){
          startB1 = millis();
          count4++;
        }
        if((endB1-startB1)>=500){
          if(count3%2==0){
            count3++;
            count4++;
          }
        }
      }  
    }
    /*
    if(mySerial.available()){
      if(mySerial.read() != NULL){
         LMotor.writeMicroseconds(1500); 
         RMotor.writeMicroseconds(1500);
         T = false;
      }
    }
    */
  
  if(mySerial.available())
  {
   if(mySerial.read() != NULL)
   {
      T = false;
      if(count5%2!=0){
        startB2 = millis();
        count5++;
      }
      LMotor.writeMicroseconds(LSF);
      RMotor.writeMicroseconds(RSF);
      //SArm.write(0);
   }
  }
  endB2 = millis();
  if((endB2 - startB2) >= 3000)
  {
      T = true;
  }
}

void Bumpers(){
  if(BPRON==true){
      if(digitalRead(bprL)==HIGH){
      if(countB%2 != 0){
        startBPR = millis();
        countB++;
      }
      LMotor.writeMicroseconds(RotL);
      RMotor.writeMicroseconds(RotR);
      endBPR = millis();
      if(digitalRead(bprR)==HIGH){
        LMotor.writeMicroseconds(LSF);
        RMotor.writeMicroseconds(RSF);
        delay(500);
        LMotor.writeMicroseconds(1500);
        RMotor.writeMicroseconds(1500);
        BPRON = false;
      }
      if((endBPR-startBPR)>1000){
        UltraSensors();
        if(distance1 < 50 && distance2 < 50 && distance3 < 50){
          LMotor.writeMicroseconds(1400);
          RMotor.writeMicroseconds(1200);
          delay(500);
  
          LMotor.writeMicroseconds(1500);
          RMotor.writeMicroseconds(1500);
          BPRON = false;
        }else{
          LMotor.writeMicroseconds(1600);
          RMotor.writeMicroseconds(1800);
          delay(500);
  
          LMotor.writeMicroseconds(1500);
          RMotor.writeMicroseconds(1500);
          BPRON = false;
        }
        
      }
    }
    else if(digitalRead(bprR)==HIGH){
      if(countB1%2 != 0){
        startBPR1 = millis();
        countB1++;
      }
      LMotor.writeMicroseconds(RotL2);
      RMotor.writeMicroseconds(RotR2);
      endBPR1 = millis();
      if(digitalRead(bprL)==HIGH){
        LMotor.writeMicroseconds(LSF);
        RMotor.writeMicroseconds(RSF);
        delay(500);
        LMotor.writeMicroseconds(1500);
        RMotor.writeMicroseconds(1500);
        BPRON = false;
      }
      if((endBPR1-startBPR1)>1000){
        UltraSensors();
        if(distance1 < 50 && distance2 < 50 && distance3 < 50){
          LMotor.writeMicroseconds(1200);
          RMotor.writeMicroseconds(1400);
          delay(500);
          
          LMotor.writeMicroseconds(1500);
          RMotor.writeMicroseconds(1500);
          BPRON = false;
        }else{
          LMotor.writeMicroseconds(1800);
          RMotor.writeMicroseconds(1600);
          delay(500);
          
          LMotor.writeMicroseconds(1500);
          RMotor.writeMicroseconds(1500);
          BPRON = false;
        }
      }
    }
  }
}


/*    maybe will be useful
      if(distance1>=actionDs && distance2>=actionDs && distance3>=actionDs && ckg == true){ //all clear, move straight
        LMotor.writeMicroseconds(1700); 
        RMotor.writeMicroseconds(1700);
      }
      if((distance1<actionDs && distance2>=actionDs && distance3>=actionDs || distance1>=actionDs && distance2<actionDs && distance3>=actionDs && right == false || distance1<extDs && distance2<actionDs && distance3>=actionDs) && ckg == true){
        LMotor.writeMicroseconds(1600);  //only left sensor sees something OR only middle and moving left relative to beacon OR middle and just extended left, turn right until all clear then prompt next stage
        RMotor.writeMicroseconds(1400);
        if(count1%2 != 0){
          startN = millis();
          count1++;
        }
        if(distance1>=actionDs && distance2>=actionDs && distance3>=actionDs && ckg == true){ //if all clear
          if(count1%2 == 0){
            endN = millis();
            tElap = endN-startN;
            count1++;
          }
          nxtL = true; //FLAG TO NEXT LEFT STAGE
          ckg = false;
        }
      }
      if((distance1>=actionDs && distance2>=actionDs && distance3<actionDs || distance1>=actionDs && distance2<actionDs && distance3>=actionDs && right == true || distance1>=actionDs && distance2<actionDs && distance3<extDs) && ckg == true){ 
        LMotor.writeMicroseconds(1400); //only right sensor sees something OR only middle and moving right relative to beacon OR middle and just extended right, turn left until all clear then prompt next stage
        RMotor.writeMicroseconds(1600);
        if(count1%2 != 0){
          startN = millis();
          count1++;
        }
        if(distance1>=actionDs && distance2>=actionDs && distance3>=actionDs && ckg == true){ //if all clear
          if(count1%2 == 0){
            endN = millis();
            tElap = endN-startN;
            count1++;
          }
          nxtR = true; //FLAG TO NEXT RIGHT STAGE
          ckg = false;
        }
      }
      if((distance1<actionDs && distance3<actionDs) && ckg == true){ //if both left and right treated same as all 3 in action distance do turn around
        //DO TURN AROUND
        ckg = false;
        if(right==true){
          LMotor.writeMicroseconds(1400); //rotate left
          RMotor.writeMicroseconds(1600);
          delay(500);
          LMotor.writeMicroseconds(1600); //fwd a bit
          RMotor.writeMicroseconds(1600);
          delay(500);
          LMotor.writeMicroseconds(1400); //rotate left
          RMotor.writeMicroseconds(1600);
          delay(500);
        }else if(right==false){
          LMotor.writeMicroseconds(1600); //rotate right
          RMotor.writeMicroseconds(1400);
          delay(500);
          LMotor.writeMicroseconds(1600); //fwd a bit
          RMotor.writeMicroseconds(1600);
          delay(500);
          LMotor.writeMicroseconds(1600); //rotate right
          RMotor.writeMicroseconds(1400);
          delay(500);
        }
        ckg = true;
      }
      
      //-------------------------------------------------------------------------------------------------------
      
      if(nxtL==true){ //GO LEFT MODE
        if(count2%2 != 0){
          startN = millis();
          count2++;
        }
        endN = millis();
        LMotor.writeMicroseconds(1700); 
        RMotor.writeMicroseconds(1600);
        if(distance1<actionDs || distance2<actionDs || distance3<actionDs){
          nxtL = false;
          ckg = true;
          count2++;
        }
        else if((endN-startN)>2000 && count2%2 == 0){ //IF NOTHING IN WAY, DO FOR 2 SECONDS
          LMotor.writeMicroseconds(1600);
          RMotor.writeMicroseconds(1400);
          delay(tElap);
          ckg = true;
          count2++;
        }
      }else if(nxtR==true){ //GO RIGHT MODE
        if(count2%2 != 0){
          startN = millis();
          count2++;
        }
        endN = millis();
        LMotor.writeMicroseconds(1600); 
        RMotor.writeMicroseconds(1700);
        if(distance1<actionDs || distance2<actionDs || distance3<actionDs){
          nxtR = false;
          ckg = true;
          count2++;
        }
        else if((endN-startN)>2000 && count2%2 == 0){
          LMotor.writeMicroseconds(1400); 
          RMotor.writeMicroseconds(1600);
          delay(tElap);
          ckg = true;
          count2++;
        }
      }


      
*/

