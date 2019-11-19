#include <Wire.h>

//Interrupt and timer variables
const uint16_t T1_COMP = 500;   //=(16,000,000/prescale*freq) - 1. Sample time of 0.5s or 2Hz
const float controllerSampleTime = 1024*(T1_COMP + 1)/16000000.0;   //calculated based on T1_COMP

volatile boolean interrupted; //flag to check if interrupt has run

//MPU 6050 variables
uint8_t gyroSampleRate = 1;

const uint8_t SMPLRT_DIV = (uint8_t)((8/gyroSampleRate) - 1); //Calculate the sample rate division and fit it into an 8bit int
const uint8_t IMUADDRESS = byte(0x68); //I2C address of the MPU6050 (b1101000)

float ax, ay, az;
float gx, gy, gz;

const float gxBias = -2.75;   //Biases found for each gyro measurement. Found by averaging readings while on a flat surface
const float gyBias = 1.48;
const float gzBias = -0.54;

unsigned long time;   //timestamp of the last time the gyro was measured. used to calculate dt

//Application variables
boolean running;

float theta [3];              //angle of the robot
float thetaRef = -98.5;  //IMU is not mounted along flat axis so reference is not 0
volatile float u;         //controller input to actuators. Must be volatile since it's accessed from the interrupt service routine

float a = 0.98;   //Weighting variable for complimentary filter in state estimation algorithm

uint8_t count;  //counter to prevent pid loop from running before enough samples are taken

void setup() 
{
  Serial.begin(9600); //begin serial communication

  //MPU 6050 SETUP
  //Wake up the MPU 6050
  Wire.beginTransmission(IMUADDRESS);
  Wire.write(107);
  Wire.write(0);
  Wire.endTransmission(true);

  //Set the gyro full scale rate
  Wire.beginTransmission(IMUADDRESS);
  Wire.write(27); //register 27 sets the gyro rate
  Wire.write(0); //Set the gyro rate to +-250 deg/s
  Wire.endTransmission(true);
  
  //Set the accelerometer's full scale range
  Wire.beginTransmission(IMUADDRESS);  
  Wire.write(28); //Full scale value is stored in register 28
  Wire.write(0);  //0 corresponds to +- 2g
  Wire.endTransmission(true);

  //Set the gyroscope sample rate (Accelerometer sample rate is set at 1kHz)
  Wire.beginTransmission(IMUADDRESS); 
  Wire.write(25); //SMPLRT_DIV is stored in register 25. Sample rate is: gyroSampleRate = 8kHz/(1 + SMPLRT_DIV)
  Wire.write(SMPLRT_DIV);
  Wire.endTransmission(true);

  //L298N Setup
  //Pin 6 is the OC0A output so pin 12 is used instead
  pinMode(7, OUTPUT);   //2,7 backwards/forward motor 1
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);   //3,4 backwards/forward motor 2
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);   //5,6 pwm speed control motor 1 & 2
  pinMode(6, OUTPUT);

  cli();  //interrupts are allowed by default. stop them.
  
  //TIMER INTERRUPT SETUP
  TCCR1A = 0;
  TCCR1B = 0;   //make sure register is zeroed out before setting bits
  TCCR1C = 0;
  
  TCCR1B |= (1 << 3);    //Set to prescaler of 1024 and CTC mode. Other bits to select CTC are set by default in TCCR1A
  TCCR1B |= (1 << 2);
  TCCR1B |= (1 << 0);

  TCNT1H = 0;    //Reset timer0 and set the compare value
  TCNT1L = 0;
  
  OCR1AH = T1_COMP >> 8;  //set high byte
  OCR1AL = T1_COMP & 0xFF;  

  TIMSK1 |= (1 << 1);   //Compare TCNT1 to OCR1A

  //Application variables setup
  running = true;
  count = 0;
  
  Serial.println("Setup finished");

  sei();  //allow interrupts again
}

void loop() 
{
  if (running)
  {
    if (interrupted)  //do anything associated with the interrupt first to prevent too much time passing
    {
      interrupted = false;
      Serial.println("Interrupt");
    }

    count++;  //increment loop counter

    //update theta
    theta[2] = theta[1];
    theta[1] = theta[0];
    theta[0] = estimateState();
    Serial.print("Theta: ");
    Serial.print(theta[0]);
    Serial.print("    u: ");
    Serial.println(u);

    //control law is undefined if any theta terms are undefined so dont send motor commands if this is true
    if (count > 5)
    {
      //interpret the controller command from the ISR
      interpretMotorCommand (u);
    }
  }
}

/**State estimation function will consist of a complimentary filter to
 * combine the accelerometer and gyroscope measurements into a single
 * value that will be relied on by the system.
 */

float estimateState ()
{
  readIMU ();   //update the values in the global readings
  
  /**Estimate the time delay from when the state was last estimated. State estimation
   * runs as fast as possible and is assumed to be a continuous process.
   */
  unsigned long dt = micros() - time;
  time += dt;             //increment time by dt
  
  float gtheta, atheta; //estimates of theta derived from either gyroscope or accelerometer measurements

  gtheta = theta[0] + gx * dt/1000000;   //need to define dt. Divide by 1000000 to convert to seconds
  atheta = atan2(ay,az)*180/PI; //Use atan2 function and convert to degrees.

  // A complimentary filter is defined as z = ax + (1-a)y. Calculate and return this value
  return a*gtheta + (1-a)*atheta;
}

  /**Interpret the control input u as a motor command. This will
  * be done linearly though the scaling may have to be adjusted depending on
  * the control law implemented do to the limits imposed on pwm variables 
  * (0 - 255).
  */
void interpretMotorCommand (float u)
{
  if (u > 0)
  {
    digitalWrite (7, LOW); //Forward
    digitalWrite (2, HIGH);
    digitalWrite (3, LOW);
    digitalWrite (4, HIGH);
  }
  else
  {
    digitalWrite (7, HIGH);  //Reverse
    digitalWrite (2, LOW);
    digitalWrite (3, HIGH);
    digitalWrite (4, LOW);
  }

  analogWrite (5, abs(u)); //set the pwm speed proportional to u. Sign is taken care of above
  analogWrite (6, abs(u));
  
  return;
}

/**Read the sensor into global variables for each relevant measure. Global variables
 * for gyro readings are gx,gy,gz and for accel are ax,ay,az.
 */
void readIMU ()
{  
  //The IMU automatically updates each sensor's reading at the rate set in the registers.
  Wire.beginTransmission(IMUADDRESS);
  Wire.write(59); //Read addresses 59-72 are sensor data
  Wire.endTransmission(false);  //For burst read don't send a stop signal

  Wire.requestFrom(IMUADDRESS,14); //Request 14 bytes of data from IMU (Register already set to start at 59)

  if (6 <= Wire.available())  //if all of the data is available
  {   
    ax = (Wire.read() << 8 | Wire.read()) / 16384.0;  //Read Accelerometer
    ay = (Wire.read() << 8 | Wire.read()) / 16384.0;
    az = (Wire.read() << 8 | Wire.read()) / 16384.0;

    Wire.read();  //Ignore the temperature sensor in the middle
    Wire.read();

    gx = ((Wire.read() << 8 | Wire.read()) / 131.0) - gxBias;  //read gyro
    gy = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyBias;
    gz = ((Wire.read() << 8 | Wire.read()) / 131.0) - gzBias;
  }

  Wire.endTransmission(true); //send stop signal to terminate transmission

  return;
}

/**Interrupt service routine will be used to calculate motor command u. The actual writing
 * of u to the motors will be done in the main loop function outside of interrupts.
 */
ISR (TIMER1_COMPA_vect)
{
  static float kp = 300;
  static float ki = 0;
  static float kd = 50;

  float a = (kp + ki*controllerSampleTime/2 + kd/controllerSampleTime);
  float b = (-kp + ki*controllerSampleTime/2 - 2*kd/controllerSampleTime);
  float c = kd/controllerSampleTime;

  u = float(a*(thetaRef - theta[0]) + b*(thetaRef - theta[1]) + c*(thetaRef - theta[2]));  //apply proportional feedback
  
  interrupted = true;
}
