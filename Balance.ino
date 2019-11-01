/**This program controls the balancing robot. This Arduino based robot uses an L298N motor driver
 * and MPU 6050 IMU to measure the robot state. Currently no odometry sensors are connected so 
 * ground position cannot be directly controlled.
 */

#include <Wire.h>

unsigned long time; //Time used to calculate dt in the stateEstimation function

float ax, ay, az;
float gx, gy, gz;

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
  pinMode(6, OUTPUT);   //6,7 backwards/forward motor 1
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);   //8,9 backwards/forward motor 2
  pinMode(9, OUTPUT);
  pinMode(10,OUTPUT);   //10,11 pwm speed control motor 1 & 2
  pinMode(11,OUTPUT);
}

void loop() {

  readIMU();  //Read the accelerometer and gyro

  theta = estimateState ();

  u = 200;

  calculateMotorSpeed(u);
}

//Function to interpret the control input u as a motor command.
void calculateMotorSpeed(int u)
{
  if (u > 0)
  {
    digitalWrite (6, LOW); //Forward
    digitalWrite (7, HIGH);
    digitalWrite (8, LOW);
    digitalWrite (9, HIGH);
  }
  else
  {
    digitalWrite (6, HIGH);  //Reverse
    digitalWrite (7, LOW);
    digitalWrite (8, HIGH);
    digitalWrite (9, LOW);
  }

  analogWrite (10, abs(u)); //set the pwm speed proportional to u. Sign is taken care of above
  analogWrite (11, abs(u));
  
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

  gtheta = theta + gx * dt;   //need to define dt
  atheta = atan2(ay,az)*180/PI; //Use atan2 function and convert to degrees.

  // A complimentary filter is defined as z = ax + (1-a)y. Calculate and return this value
  return a*gtheta + (1-a)*atheta;
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
