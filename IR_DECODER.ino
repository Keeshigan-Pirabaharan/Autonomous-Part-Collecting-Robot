#include <IRremote.h>

const int RECV_PIN = A3;
IRrecv irrecv(RECV_PIN);
decode_results results;
int H = 0;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  irrecv.blink13(true);
}

void loop()
{
  if(irrecv.decode(&results))
  {
    Serial.println(results.value, HEX); //INT?
    irrecv.resume();
  }
}
