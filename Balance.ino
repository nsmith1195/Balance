/**This program controls the balancing robot. This Arduino based robot uses an L298N motor driver
 * and MPU 6050 IMU to measure the robot state. Currently no odometry sensors are connected so 
 * ground position cannot be directly controlled.
 */

#include <Wire.h>

const uint16_t t0_load = 0;     //Set variables for the initial value of timer0 (t0_load) and value to throw interrupt at (t0_comp)
const uint16_t t0_comp = 12500;

boolean running;  //variable to automatically kill program if necessary

unsigned long runTime;  //time in milliseconds for the program to run before stopping
unsigned long time; //Time used to calculate dt in the stateEstimation function

float ax, ay, az;
float gx, gy, gz;

const float thetaRef = -100.3;  //Experimentally determined for upright position

float theta;  //System state is considered to be just theta measured about the IMU x axis. Measured in degrees.
float u;  //Control input function. Robot is constrained to straight line motion to 1 input for 2 motors

float a = 0.98; //bias for the complimentary filter in estimateState function. Range 0-1 with 1 favoring gyro measurements

uint8_t gyroSampleRate = 1;

const uint8_t SMPLRT_DIV = (uint8_t)((8/gyroSampleRate) - 1); //Calculate the sample rate division and fit it into an 8bit int
const uint8_t IMUADDRESS = byte(0x68); //I2C address of the MPU6050 (b1101000)

const float gxBias = -2.75;   //Biases found for each gyro measurement. Found by averaging readings while on a flat surface
const float gyBias = 1.48;
const float gzBias = -0.54;

void setup() {
  Wire.begin();   //Join (create) the I2C bus
  
  Serial.begin (9600);  //Start serial communication with the host PC

  delay (5000);   //Wait 5 seconds before doing anything to allow both batteries to be installed. TODO: install switch on robot...
  running = true;   //allow main loop to run
  runTime = 15000;  //run program for 10 seconds

  //TODO: update the MPU setup to be a burst write. Need to look into register 26 first

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

  //L298 SETUP
  //Pin 6 is the OC0A output so pin 12 is used instead
  pinMode(12, OUTPUT);   //12,7 backwards/forward motor 1
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);   //8,9 backwards/forward motor 2
  pinMode(9, OUTPUT);
  pinMode(10,OUTPUT);   //10,11 pwm speed control motor 1 & 2
  pinMode(11,OUTPUT);

  //TIMER INTERRUPT SETUP
  TCCR0A = byte(0x42); //reset Timer0 control reg A to use OC0A register for compare match and use CTC mode

  TCCR0B |= (1 << CS12);    //Set to prescaler of 256
  TCCR0B &= ~(1 << CS11);
  TCCR0B &= ~(1 << CS10);

  TCNT0 = t0_load;    //Reset timer0 and set the compare value
  OCR0A = t0_comp;

  TIMSK0 &= ~(1 << 2);   //Enable timer0 compare intterupt A
  TIMSK0 |= (1 << 1);
  TIMSK0 &= ~(1 << 0);

  sei();  //Enable global interrupts
}

void loop() {
  if (running)
  {
    readIMU();  //Read the accelerometer and gyro

    theta = estimateState ();   //State only consists of theta for now

    Serial.print ("Theta: ");
    Serial.println(theta);

    if (millis() > runTime) //stop the program after 10 seconds
    {
      noInterrupts();   //turn off interrupts
      calculateMotorSpeed(0); //send the motors a stop signal
      running = false;  //stop the loop from running
    }
  }
}

/**Function to interpret the control input u as a motor command. This will
 * be done linearly though the scaling may have to be adjusted depending on
 * the control law implemented do to the limits imposed on pwm variables 
 * (0 - 255).
 */
void calculateMotorSpeed(int u)
{
  u = 0;
  
  if (u > 0)
  {
    digitalWrite (12, LOW); //Forward
    digitalWrite (7, HIGH);
    digitalWrite (8, LOW);
    digitalWrite (9, HIGH);
  }
  else
  {
    digitalWrite (12, HIGH);  //Reverse
    digitalWrite (7, LOW);
    digitalWrite (8, HIGH);
    digitalWrite (9, LOW);
  }

  analogWrite (10, abs(100)); //set the pwm speed proportional to u. Sign is taken care of above
  analogWrite (11, abs(100));
  
  return;
}


/**Theta will be estimated using a complimentary filter combining the accelerometer 
 * and gyro readings. Care must be taken to avoid the integration error impacting this
 * value since a complimentary filter will not fix that, gtheta is calculated by adding
 * the estimated gyro reading to the previous filtered theta to prevent this build up.
 * The parameter a is defined above as a global variable.
 */
float estimateState ()
{
  /**Estimate the time delay from when the state was last estimated. State estimation
   * runs as fast as possible and is assumed to be a continuous process.
   */
  unsigned long dt = micros() - time;
  time += dt;             //increment time by dt
  
  float gtheta, atheta; //estimates of theta derived from either gyroscope or accelerometer measurements

  gtheta = theta + gx * dt/1000000;   //need to define dt. Divide by 1000000 to convert to seconds
  atheta = atan2(ay,az)*180/PI; //Use atan2 function and convert to degrees.

  // A complimentary filter is defined as z = ax + (1-a)y. Calculate and return this value
  return a*gtheta + (1-a)*atheta;
}

/**Function to determine the control input, u, from the system state. Currently this is
 * done as fast as the program can run in a round robin configuration while making the 
 * assumption that the controller is in continuous time. This will be changed in the
 * future to be a discrete time controller running off interrupts to ensure a constant
 * sample time. Currently this is a PID controller with the system modeled as SISO
 * (x degree of freedom is ignored so robot may still wander so long as it remains
 * balanced upright).
 */
float evaluateControlLaw ()
{ 
  static float err, accErr = 0; //error and accumulated error
   
  static float kp = 100;  //Defined as static local to allow gains to persist through
  static float ki = 0;  //function returns.
  static float kd = 0;

  err = thetaRef - theta;   //absolute error  
  accErr += err;   //ignored dt since result can be arbitrarily decided with ki
  
  return kp*err + ki*accErr;  
}

void readIMU ()
{
  //The IMU automatically updates each sensor's reading at the rate set in the registers.
  Wire.beginTransmission(IMUADDRESS);
  Wire.write(59); //Read addresses 59-72 are sensor data
  Wire.endTransmission(false);  //For burst read don't send a stop signal

  Wire.requestFrom(IMUADDRESS,14); //Request 14 bytes of data from IMU (Register already set to start at 59)

  if (6 <= Wire.available())  //if all of the data is available
  {
    noInterrupts(); //don't allow interrupts while reading data
    
    ax = (Wire.read() << 8 | Wire.read()) / 16384.0;  //Read Accelerometer
    ay = (Wire.read() << 8 | Wire.read()) / 16384.0;
    az = (Wire.read() << 8 | Wire.read()) / 16384.0;

    Wire.read();  //Ignore the temperature sensor in the middle
    Wire.read();

    gx = ((Wire.read() << 8 | Wire.read()) / 131.0) - gxBias;  //read gyro
    gy = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyBias;
    gz = ((Wire.read() << 8 | Wire.read()) / 131.0) - gzBias;

    interrupts(); //allow interrupts again
  }

  Wire.endTransmission(true); //send stop signal to terminate transmission

  return;
}

/**Timer interrupt running off timer0. This ISR evaluates the control
 * law and translates it into a motor command at a regular interval
 * defined by t0_comp register.
 */
ISR (TIMER0_COMPA_vect)
{  
  calculateMotorSpeed(evaluateControlLaw());
}
