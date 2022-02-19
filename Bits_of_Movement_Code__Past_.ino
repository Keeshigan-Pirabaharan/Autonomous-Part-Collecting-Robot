
 * void Fan() // USING
{
  //Fan
  if(V == true)
  {
    V = false;
    LMotor.writeMicroseconds(LSF); 
    RMotor.writeMicroseconds(RSF);
    delay(1500);
    LMotor.writeMicroseconds(1700); 
    RMotor.writeMicroseconds(1300);
    delay(900);
  }
  UltraSensors();
  if(U == true && n < ntimes)
  {
    U = false;
    starttimer = millis();
    endtimer = 0;
    LMotor.writeMicroseconds(LSF); 
    RMotor.writeMicroseconds(RSF);
  }
  endtimer = millis();
  UltraSensors();
  if(((endtimer - starttimer) >= 4000 || distance2 <= 12) && U == false)
  {
    endtimer2 = millis();
    del = (endtimer2 - starttimer);
    LMotor.writeMicroseconds(LSB); 
    RMotor.writeMicroseconds(RSB);
    delay(del/1.1);
    U = true;
    LMotor.writeMicroseconds(1300); 
    RMotor.writeMicroseconds(1700);
    delay(80);
    endtimer = 0;
    endtimer2 = 0;
    starttimer = 10;
    n++;
  }
  UltraSensors();
  if(n == ntimes)
  {
    LMotor.writeMicroseconds(1700); 
    RMotor.writeMicroseconds(1300);
    delay(300);
    LMotor.writeMicroseconds(LSB); 
    RMotor.writeMicroseconds(RSB);
    delay(800);
    LMotor.writeMicroseconds(1500); 
    RMotor.writeMicroseconds(1500);
    n++;
  }
}
*/
 */


/*void ObstacleAvoidanceTest() //Keesh's Basic Ass Obstacle Avoidance // NOT USING
{
  if(distance1 > obs2 && distance2 > obs && distance3 > obs2)
  {
    LMotor.writeMicroseconds(LS); 
    RMotor.writeMicroseconds(RS);
  }
  if(distance1 > obs2 && distance2 <= obs && distance3 > obs2)
  {
    LMotor.writeMicroseconds(1500); 
    RMotor.writeMicroseconds(RS);
  }
  if(distance1 <= obs2 && distance2 > obs && distance3 > obs2)
  {
    LMotor.writeMicroseconds(LS); 
    RMotor.writeMicroseconds(1500);
  }
  if(distance1 > obs2 && distance2 > obs && distance3 <= obs2)
  {
    LMotor.writeMicroseconds(1500); 
    RMotor.writeMicroseconds(RS);
  }
  if(distance1 <= obs2 && distance2 <= obs && distance3 > obs2)
  {
    LMotor.writeMicroseconds(LS); 
    RMotor.writeMicroseconds(1500);
  }
  if(distance1 > obs2 && distance2 <= obs && distance3 <= obs2)
  {
    LMotor.writeMicroseconds(1500); 
    RMotor.writeMicroseconds(RS);
  }
  if(distance1 <= obs2 && distance2 <= obs && distance3 <= obs2)
  {
    LMotor.writeMicroseconds(1700); 
    RMotor.writeMicroseconds(1300);
    delay(2000);
  }
  if(distance1 <= obs2 && distance2 > obs && distance3 <= obs2)
  {
    LMotor.writeMicroseconds(LS); 
    RMotor.writeMicroseconds(RS);
  }
}*/

//Callibrate Compass
/*
if(T == true)
{
  T = false;
  Serial.println("???");
  CalibrateCompass();
}
*/

// Compass Turns FOR TEST
/*
 * if(CA == true)
{
  CA = false;
  LMotor.writeMicroseconds(1700); 
  RMotor.writeMicroseconds(1700);
  delay(2000);
  LMotor.writeMicroseconds(1500); 
  RMotor.writeMicroseconds(1700);
}
if(headingDegrees > (West-Offset) && headingDegrees < (West+Offset))
{
  LMotor.writeMicroseconds(1700); 
  RMotor.writeMicroseconds(1700);
}
if(headingDegrees > (East-Offset) && headingDegrees < (East+Offset))
{
  LMotor.writeMicroseconds(1700); 
  RMotor.writeMicroseconds(1700);
}
*/


//Compass calibration
/*
if(S == true)
{
  LMotor.writeMicroseconds(1700); 
  RMotor.writeMicroseconds(1700);
  delay(2000);
  CalibrateCompass();
  S = false;
}
if(T == true)
{
  T = false;
  LMotor.writeMicroseconds(1700); 
  RMotor.writeMicroseconds(1500);
}
Compass();
HeadingCompass();
if(headingDegrees >= (North - Offset) && headingDegrees <= (North + Offset))
{
  LMotor.writeMicroseconds(1500); 
  RMotor.writeMicroseconds(1500);
}
*/
//Straight Line
/*
LMotor.writeMicroseconds(1700); 
RMotor.writeMicroseconds(1700); 
BArm.write(0
*/

//To Beacon
/*
UltraSensors();
//Obstacle Avoidance
if(distance1 < 8 && distance2 > 8 && distance3 > 8)
{
  LMotor.writeMicroseconds(1700); 
  RMotor.writeMicroseconds(1500); 
}

if(distance1 > 8 && distance2 > 8 && distance3 < 8)
{
  LMotor.writeMicroseconds(1500); 
  RMotor.writeMicroseconds(1700); 
}
*/

//Circle
   /* 
     LMotor.writeMicroseconds(j++); 
     RMotor.writeMicroseconds(i++);
     if(i>=1500 && i<=1700)
     delay(10);
     else
     delay(20);
     if(i > 1700)
     {
      i = 1500;
      j = 1900;
     }
   */

//Lines (Kinda Straight)
/*
     LMotor.writeMicroseconds(1700); 
     RMotor.writeMicroseconds(1690); 
     if(A == false && B == true)
     {
      A = true;
      B = false;
     }
     else if(A == true && B == true)
     {
      A = false;
      B = false;
     }
     delay(2000);
     if(A == true)
     {
       LMotor.writeMicroseconds(1500); 
       RMotor.writeMicroseconds(1690);
       delay(2000);
       B = true;
     }
     else if(A == false)
     {
       LMotor.writeMicroseconds(1700); 
       RMotor.writeMicroseconds(1500);
       delay(2000);
       B = true;
     }
*/
//Ultrasonic Turn
/*
      LMotor.writeMicroseconds(1700); 
      RMotor.writeMicroseconds(1690);  
      Ping();
      if((ul_Echo_Time/58) <= 5 && (ul_Echo_Time/58) >= 1  && T == true)
      {
        T = false;
        LMotor.writeMicroseconds(1500); 
        RMotor.writeMicroseconds(1690);
        delay(1100);
      }
      T = true;
*/


//IR Sensor

LMotor.writeMicroseconds(1700); 
RMotor.writeMicroseconds(1690); 
int detect = analogRead(A4);
Serial.println(detect);
if(detect <= 100 && T == true)
{
  T = false;
  LMotor.writeMicroseconds(1500); 
  RMotor.writeMicroseconds(1690);
  delay(1100);
}
T = true;
