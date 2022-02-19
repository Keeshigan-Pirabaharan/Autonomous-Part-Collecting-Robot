void setup() 
{
  // put your setup code here, to run once:
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if(digitalRead(3))
  Serial.println("High");
  else
  Serial.println("Low");
}
