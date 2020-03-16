class PID
{
  private:
  uint16_t kp;  //Controller gains
  uint16_t ki;
  uint16_t kd;

  int32_t err;     //Tracked variables
  int32_t accErr;
  int32_t diff;
  
  int32_t lastCount;  //previous position for speed calculations
  int32_t reference;  //reference position specified in counts

  int32_t lastU;

  unsigned long dt; //time step for the controller

  public:
  PID (uint16_t p, uint16_t i, uint16_t d, unsigned long t)
  {
    kp = p;
    ki = i;
    kd = d;

    err = 0;
    accErr = 0;
    diff = 0;

    lastCount = 0;  //initialize both to zero
    reference = 0;

    t = dt;
  }

  /**main function to generate input signal. Assumes the time step dt is respected by the main
   * program. A timer interrupt is to be used to ensure it is. The controller assumes continuous
   * time so dt must be small to prevent errors.
    */
  int32_t generateInput (int32_t count)
  {
    err = reference - count;
    accErr += err;
    diff = (int32_t)(count - lastCount)/dt;
    lastCount = count;  //current count becomes last count
    
    return lastU = (int32_t)(kp*err + ki*accErr + kd*diff);
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
    Serial.println (err);
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
  
  volatile int32_t positionCounts;   //variables measured in counts and counts per sec respectively
  volatile float velocity;

  uint8_t channelAPin;  //Channel A triggers interrupt
  uint8_t channelBPin;  //Channel B checks direction

  volatile unsigned long lastRun;  //Timestamp in microseconds when last run

  public:
  QuadEncoder (uint8_t pin1, uint8_t pin2)  //Constructor method
  {
    positionCounts = 0; //Relative encoder always starts at position 0
    velocity = 0; //Velocity is completely dependent on position in this application

    channelAPin = pin1;   //channelAPin has interrupt attached and triggers updates
    channelBPin = pin2;   //channelBPin is checked in the interrupt to determine direction 
  }

  void displayState ()
  {
    if (Serial) //will only work if serial communication is running
    {
      Serial.print ("Position: ");
      Serial.println (positionCounts);
      Serial.print ("Velocity: ");
      Serial.println (velocity);
    }
  }

  void readEncoder () //Read encoder and update position and velocity       TODO: Velocity calc gives weird results
  {
    unsigned long currentTime = micros();
    
    if (digitalRead(channelBPin))
    {
      positionCounts++;

      velocity = (float)(1000000/(currentTime - lastRun)); //calculate the velocity (in count/sec) and convert to float
    }
    else
    {
      positionCounts--;

      velocity = (float)(-1000000/(currentTime - lastRun)); 
    }
    
    lastRun = currentTime; //update lastRun  
  }
};

                            //GLOBAL VARIABLE DEFINITION
boolean running = true;
volatile boolean changed = false;

unsigned long lastControllerTime;

PID pid (100,10,0,100);  //define pid object
QuadEncoder enc1 (2,3);   //initialize encoder 1 object

void setup() {
  interrupts(); //make sure interrupts are enabled
  
  lastControllerTime = 0;
  
  pinMode(2, INPUT);  //Pin 2 is the interrupt pin
  pinMode(3, INPUT);
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT); //pwm output for forward motor command
  pinMode(11, OUTPUT); //pwm output for reverse motor command

  digitalWrite(13, HIGH); //Turn on the encoder. Will remain on for entirety of sketch

  //Trigger interrupt on rising edge of channel A. Checking only A halves resolution
  attachInterrupt(INT0, enc1Read, RISING);   //Hacky solution to avoid problem of calling instance method from interrupt.

  Serial.begin(9600); //Start serial communication
  Serial.println("Initialized");

  pid.setReference (100); //Change the reference value
}

void loop() {
  if (running)
  {
    /**if (millis() - lastControllerTime > 100)
    {
      commandMotors(pid.generateInput(encCount1)); //update motor command
      lastControllerTime = millis();
    }**/
        
    if (changed)
    {
      enc1.displayState();
      changed = false;
    }
  }

  if (Serial.available() > 0)
  {
    handleSerialInput ();
  }
}

void commandMotors (int32_t u)
{
  u = map (u, -2550, 2550, -255, 255);  //map down the values of u
  
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
    analogWrite (10, u);
  }
  else
  {
    analogWrite (11, abs(u));
  }
}

void handleSerialInput ()
{
  char received = Serial.read();

  if (received == 'q')
  {
    analogWrite (10,0);
    analogWrite (11,0);
    running = false;
  }
  if (received == 'e')
  {
    pid.printErr();
  }
  if (received == 'i')
  {
    pid.printAccErr();
  }
  if (received == 'r')
  {
    pid.printReference ();
  }
  if (received == 'u')
  {
    pid.printLastU ();
  }
}

void enc1Read ()
{
  enc1.readEncoder ();
  changed = true;
}
