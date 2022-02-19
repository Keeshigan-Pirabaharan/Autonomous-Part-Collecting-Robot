#include <Servo.h>

Servo BArm; // Beacon Arm
const int BA = 11;

void setup() 
{
  pinMode(BA, OUTPUT);
  BArm.attach(BA);
}

void loop() 
{
  if(HE == true) // When heading East, servo points left
    BArm.write(0);
  if(HW == true) // When heading West, servo point right
    BArm.write(180);
}
