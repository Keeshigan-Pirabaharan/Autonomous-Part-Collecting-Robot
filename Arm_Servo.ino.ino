#include <Servo.h>

Servo SArm;
const int SA = 10;
unsigned int SArmtimerS = 10;
unsigned int SArmtimerE = 0;
int i = 0;
bool ArmBool = true;

void setup() 
{
  pinMode(SA, OUTPUT);
  SArm.attach(SA);
}

void loop() 
{
  ArmSwing();
}

void ArmSwing()
{
  if(ArmBool == true)
  {
  ArmBool = false;
  for(i = 0; i < 160;)
  {
    SArm.write(i++);
    delay(20);
    if(i == 159)
      SArmtimerS = millis();
  }
  }
  SArmtimerE = millis();
  if((SArmtimerE - SArmtimerS) > 1000)
  {
    SArmtimerS = 10;
    SArmtimerE = 0;
    ArmBool = true;
  }
}
