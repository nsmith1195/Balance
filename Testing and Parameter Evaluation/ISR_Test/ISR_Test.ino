/** Program to familiarize with how intterupts work in the arduino. Various tests
 *  will be performed and this code will change greatly over time as this is
 *  basically a scratch pad.
 */

#include "QuadEncoder.cpp"

volatile boolean called;
QuadEncoder enc1 (2,3);   //initialize encoder 1 object

void setup ()
{
  called = false;

  Serial.begin (9600);  //start serial communication
  Serial.println ("Initialized");
  Serial.print ("PCICR: ");
  Serial.println (PCICR,BIN);
  Serial.print ("PCMSK2: ");
  Serial.println (PCMSK2,BIN);
}

void loop ()
{
  if (called)
  {
    Serial.println (enc1.getPosition());
    called = false;
  }
}

ISR (PCINT2_vect)
{
  enc1.readEncoder();
  called = true;
}
