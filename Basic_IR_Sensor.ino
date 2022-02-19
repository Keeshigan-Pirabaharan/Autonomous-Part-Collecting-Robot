#include <Servo.h>
#include <EEPROM.h>
//#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <SoftwareSerial.h>


const int LeftS = A0;
const int LeftF = A1;
const int MidF = A2;
const int RightF = A3;
const int RightS = A4;

unsigned int LS;
unsigned int LF;
unsigned int MF;
unsigned int RF;
unsigned int RS;

void setup() 
{
  pinMode(LeftS, INPUT);
  pinMode(LeftF, INPUT);
  pinMode(MidF, INPUT);
  pinMode(RightF, INPUT);
  pinMode(RightS, INPUT);
}

void loop() 
{
  readIRSensors();
  Serial.print("Left Side Sensor: ");
  Serial.println(LS);
  Serial.print("Left Front Sensor: ");
  Serial.println(LF);
  Serial.print("Middle Front Sensor: ");
  Serial.println(MF);
  Serial.print("Right Front Sensor: ");
  Serial.println(RF);
  Serial.print("Right Side Sensor: ");
  Serial.println(RS);
  delay(1000);
}

void readIRSensors()
{
  if(analogRead(LeftS) > 100)
    LS = LOW;
  else
    LS = HIGH; 
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
  if(analogRead(RightS) > 100)
    RS = LOW;
  else  
    RS = HIGH;
}
