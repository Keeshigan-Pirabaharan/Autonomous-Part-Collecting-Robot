/*
  Software serial MSE 2202 IR tester

 The circuit:

* RX is digital pin 7 (connect to TX of other device)
* TX is digital pin 11 (connect to RX of other device)

*/

#include <SoftwareSerial.h>

SoftwareSerial mySerial(A13, A13); // RX, TX
unsigned long J = 0;

void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("MSE 2202 IR tester");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);
}

 

void loop() 
{ // run over and over
  if(mySerial.available())
  {
   Serial.println(mySerial.read());
    /*
    if(mySerial.read() == 66 || mySerial.read() == 57)
    {
      Serial.println ("Good"); 
    }
    else 
    {
      Serial.println ("Bad"); 
    }
    */
  }
}
