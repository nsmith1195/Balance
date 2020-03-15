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

                            //GLOBAL VARIABLE DEFINITION
volatile int32_t encCount1;
volatile boolean changed;

boolean running = true;

unsigned long lastControllerTime;

PID pid (100,10,0,100);  //define pid object

void setup() {
  interrupts(); //make sure interrupts are enabled
  
  encCount1 = 0;  //Initialize encoder 1 count
  lastControllerTime = 0;
  
  pinMode(2, INPUT);  //Pin 2 is the interrupt pin
  pinMode(3, INPUT);
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT); //pwm output for forward motor command
  pinMode(11, OUTPUT); //pwm output for reverse motor command

  attachInterrupt(INT0, enc1, RISING); //Trigger interrupt on rising edge of channel A. Checking only A halves resolution

  digitalWrite(13, HIGH); //Turn on the encoder. Will remain on for entirety of sketch

  Serial.begin(9600); //Start serial communication
  Serial.println("Initialized");

  pid.setReference (100); //Change the reference value
}

void loop() {
  if (running)
  {
    if (changed)      //Check if the encoder has changed value. If it has print the new value
    {
      Serial.print("Count: ");
      Serial.println(encCount1);
      changed = false;
    }

    if (millis() - lastControllerTime > 100)
    {
      commandMotors(pid.generateInput(encCount1)); //update motor command
      lastControllerTime = millis();
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

  //Serial.print ("u: ");
  //Serial.println(u);
}

void enc1()
{
  if (digitalRead(3)) //Read channel A. If high moving forward, else backwards
  {
    encCount1++;
  }
  else
  {
    encCount1--;
  }  

  changed = true; //Tell program count has changed
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
