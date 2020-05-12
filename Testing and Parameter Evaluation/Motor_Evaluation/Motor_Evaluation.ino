#include "PID.cpp"
#include "QuadEncoder.cpp"
#include "L298.cpp"

const int NUMREADINGS = 10; //number of readings of speed and voltage

boolean running = false;    //wait for user input to make sure power supply is ready
volatile boolean changed = false;
boolean readingVoltage = false; //true if taking voltage and speed readings.

unsigned long lastControllerTime;

int currentPower = 0;  //TEST VARIABLE DELETE WHEN DONE

PID pid (2,0.0,0.1,0.1);  //define pid object                          PID LOOP STILL DOESN'T WORK
QuadEncoder enc1 (2,3);   //initialize encoder 1 object
L298 Driver (6,5);       //setup motor driver object

void setup() {  
  lastControllerTime = 0;
 
  //Setup timer interrupt to update velocity
  noInterrupts ();  //turn off interrupts

  TCCR1A = 0;   //Set all bits to 0 to be changed later
  TCCR1B = 0;

  TCCR1B |= (1 << WGM12); //CTC Mode on timer 1 comparing to OCR1A
  TCCR1B |= (1 << CS12);  //256 prescaler

  TIMSK1 |= (1 << OCIE1A);  //Enable compare match interrupt for timer 1
  
  OCR1A = 6249; //10 Hz
  TCNT1 = 0;   //initialize counter to 0

  interrupts ();  //make sure interrupts are turned back on

  Serial.begin(9600); //Start serial communication
  Serial.println("Initialized");

  pid.setReference (50); //Change the reference value
}

void loop() {
  if (running)
  {

  }

  if (changed)
  {
    Serial.println (enc1.getPosition ());
    changed = false;
  }

  if (Serial.available() > 0)
  {
    handleSerialInput ();
  }
}

void handleSerialInput ()
{
  char received = Serial.read();

  switch (received)
  {
    case 'q':
      Driver.stop();
      running = false;
      break;
    case 'e':
      pid.printErr ();
      break;
    case 'r':
      pid.printReference ();
      break;
    case 'w':
      pid.setReference(pid.getReference() + 50);
      break;
    case 'a':
      currentPower += 5;
      Driver.commandMotor1 (currentPower);
      break;
    case 's':
      currentPower -= 5;
      Driver.commandMotor1 (currentPower);
      break;
    case 'x':
      enc1.displayState ();
      break;
    case 'g':
      running = true;
      break;
  }
}

ISR (PCINT2_vect)
{
  enc1.readEncoder ();
  changed = true;
}

ISR (TIMER1_COMPA_vect)
{
  enc1.estimateVelocity ();
  if (running)
  {
    Driver.commandMotor1(pid.generateInput(enc1.getPosition()));
  }
}
