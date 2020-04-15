#include "PID.cpp"
#include "QuadEncoder.cpp"
#include "L298.cpp"

                            //GLOBAL VARIABLE DEFINITION
boolean running = false;    //wait for user input to make sure power supply is ready
volatile boolean changed = false;

unsigned long lastControllerTime;

int currentPower = 0;  //TEST VARIABLE DELETE WHEN DONE

PID pid (1.5,1,0,0.01);  //define pid object
QuadEncoder enc1 (2,3);   //initialize encoder 1 object
L298 Driver (6,5);       //setup motor driver object

void setup() {  
  lastControllerTime = 0;
  
  pinMode(2, INPUT);  //Pin 2 is the interrupt pin
  pinMode(3, INPUT);
  pinMode(13, OUTPUT);
  
  digitalWrite(13, HIGH); //Turn on the encoder. Will remain on for entirety of sketch

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

  //Trigger interrupt to read encoder on rising edge of channel A. Checking only A halves resolution
  attachInterrupt(INT0, enc1Read, RISING);   //Hacky solution to avoid problem of calling instance method from interrupt.

  Serial.begin(9600); //Start serial communication
  Serial.println("Initialized");

  pid.setReference (200); //Change the reference value
}

void loop() {
  if (running)
  {
//    if (millis() - lastControllerTime > 10)
//    {
//      Driver.commandMotor1(pid.generateInput(enc1.getPosition())); //update motor command
//      lastControllerTime = millis();
//    }
    Serial.println(enc1.getPosition());
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
    case 'i':
      pid.printAccErr ();
      break;
    case 'r':
      pid.printReference ();
      break;
    case 'u':
      pid.printLastU ();
      break;
    case 'a':
      Driver.commandMotor1 (-255);
      break;
    case 's':
      Driver.commandMotor1 (255);
      break;
    case 'x':
      enc1.displayState ();
      break;
    case 't':                   //TEST CODE
      currentPower += 25;
      Driver.commandMotor1 (currentPower);
      break;
    case 'y':
      currentPower -= 25;
      Driver.commandMotor1 (currentPower);
      break;
    case 'g':
      running = true;
      break;
  }
}

void enc1Read ()
{
  enc1.readEncoder ();
  changed = true;
}

//Function to return the 4 point moving average of velocity for the most accurate estimate since measurements fluctuate
float updateAverageVelocity (float vCurrent)
{
  static float velocity [3] = {0, 0, 0};  //4th point is vCurrent

  float average = (vCurrent + velocity[0] + velocity[1] + velocity[2])/4.0;

  velocity[2] = velocity[1];  //Move each stored velocity back a time step
  velocity[1] = velocity[0];
  velocity[0] = vCurrent;

  return average;
}

ISR (TIMER1_COMPA_vect)
{
  enc1.updateVelocity ();
}
