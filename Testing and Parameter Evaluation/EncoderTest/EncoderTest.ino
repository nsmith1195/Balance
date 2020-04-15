class PID
{
  private:
  float kp;  //Controller gains
  float ki;
  float kd;

  int32_t err [2];     //Tracked variables
  int32_t accErr;
  float diff;       //float because it is read from encoder
  
  int32_t lastCount;  //previous position for speed calculations
  int32_t reference;  //reference position specified in counts

  int32_t lastU;

  float t; //time step for the controller (seconds)

  public:
  PID (float p, float i, float d, float dt)
  {
    kp = p;
    ki = i;
    kd = d;

    err [0] = reference;  //initial errors are the entire distance from reference to 0
    err [1] = reference;

    reference = 0;

    t = dt;
  }

  /**main function to generate input signal. Assumes the time step dt is respected by the main
   * program. A timer interrupt is to be used to ensure it is. The controller assumes continuous
   * time so dt must be small to prevent errors.
    */
  int32_t generateInput (int32_t count)
  {
    float a = (float)(kp + ki*t/2.0 + kd/t);
    float b = (float)(-kp + ki*t/2.0 - 2*kd/t);
    float c = (float)kd/t;

    lastU = (int32_t)(lastU + a*(reference - count) + b*err[1] + c*err[2]);   //Discrete time pid loop

    err[1] = err[0];
    err[0] = reference - count;

    return lastU;
  }

  void getSpeed ()
  {
    return diff;
  }

  void printAccErr ()
  {
    Serial.print ("accErr: ");
    Serial.println(accErr);
  }

  void printErr ()
  {
    Serial.print ("Err: ");
    Serial.println (err[0]);
  }

  void printLastU ()
  {
    Serial.print ("U: ");
    Serial.println (lastU);
  }

  void printReference ()
  {
    Serial.print ("Reference: ");
    Serial.println(reference);
  }

  void setReference (int32_t ref)
  {
    reference = ref;
  }
};

/** Code to run the AEDS-96xx optical encoder. Due to the speed of the arduino only a quarter of possible states
 *  are used. This class assumes that the main program has already set up the appropriate pins as inputs. The 
 *  constructor of this class however attaches the appropriate interrupts.
 *    It is possible that in the future the micros function may be replaced with direct register reads on an 
 *  internal timer to increase accuracy and speed up the functions.
 */
class QuadEncoder
{
  private:
  const uint16_t COUNTS = 700; //Encoder wheel believed to be 700 count/rev
  
  volatile int32_t positionCounts [2];   //last and current positions. Measured in counts
  volatile float velocity;        //velocity measure in counts per second

  uint8_t channelAPin;  //Channel A triggers interrupt
  uint8_t channelBPin;  //Channel B checks direction

  public:
  QuadEncoder (uint8_t pin1, uint8_t pin2)  //Constructor method
  {
    positionCounts[0] = 0; //Relative encoder always starts at position 0
    positionCounts[1] = 0;
    velocity = 0; //Velocity is completely dependent on position in this application

    channelAPin = pin1;   //channelAPin has interrupt attached and triggers updates
    channelBPin = pin2;   //channelBPin is checked in the interrupt to determine direction 
  }

  void displayState ()
  {
    if (Serial) //will only work if serial communication is running
    {
      Serial.print ("Position: ");
      Serial.println (positionCounts[0]);
      Serial.print ("Velocity: ");
      Serial.println (velocity);
    }
  }

  int32_t getPosition ()
  {
    return positionCounts[0];
  }

  float getVelocity ()
  {
    return velocity;
  }

  //Read encoder and update position and velocity
  void readEncoder ()
  {    
    if (digitalRead(channelBPin))
    {
      positionCounts[0]++;
    }
    else
    {
      positionCounts[0]--;
    }
  }

  void updateVelocity ()
  {    
    //dt is constant (0.1s)
    velocity = (float)((positionCounts[0] - positionCounts[1])/(0.1)); //dt is constant since the motor is moving slow

    positionCounts [1] = positionCounts[0]; //update the value of the previous position
  }
};

                            //GLOBAL VARIABLE DEFINITION
boolean running = true;
volatile boolean changed = false;

unsigned long lastControllerTime;

PID pid (1.5,1,0,0.01);  //define pid object
QuadEncoder enc1 (2,3);   //initialize encoder 1 object

void setup() {  
  lastControllerTime = 0;
  
  pinMode(2, INPUT);  //Pin 2 is the interrupt pin
  pinMode(3, INPUT);
  pinMode(5, OUTPUT); //pwm output for forward motor command
  pinMode(6, OUTPUT); //pwm output for reverse motor command  
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
    if (millis() - lastControllerTime > 10)
    {
      commandMotors(pid.generateInput(enc1.getPosition())); //update motor command
      lastControllerTime = millis();
    }

    Serial.println(enc1.getPosition());
  }

  if (Serial.available() > 0)
  {
    handleSerialInput ();
  }
}

/**Function to process the input u generated by the controller and convert it into motor
 * voltages. The motor output is insignificant at values less than ~125 due to the friction
 * in the encoder so care must be taken when tuning a linear controller.
 */
void commandMotors (int32_t u)
{
  //Serial.println (u);
    
  if (u > 255) //constrain u to the limits of analogWrite function
  {
    u = 255;
  }
  if (u < -255)
  {
    u = -255;
  }

  if (u >= 0)
  {
    analogWrite (5, u);
    analogWrite (6, 0);    //Motor will lock up if the other pin isn't written to 0
  }
  else
  {
    analogWrite (5, 0);
    analogWrite (6, abs(u));
  }
}

void handleSerialInput ()
{
  char received = Serial.read();

  switch (received)
  {
    case 'q':
      analogWrite (5,0);
      analogWrite (6,0);
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
      commandMotors (-2550);
      break;
    case 's':
      commandMotors (2550);
      break;
    case 'x':
      enc1.displayState ();
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
