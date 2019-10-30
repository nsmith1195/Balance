#include <Wire.h>

float ax, ay, az;
float gx, gy, gz;

uint8_t gyroSampleRate = 1;

const uint8_t SMPLRT_DIV = (uint8_t)((8/gyroSampleRate) - 1); //Calculate the sample rate division and fit it into an 8bit int
const uint8_t IMUADDRESS = byte(0x68); //I2C address of the MPU6050 (b1101000)

const float gxBias = -2.75;   //Biases found for each gyro measurement. Found by averaging readings while on a flat surface
const float gyBias = 1.48;
const float gzBias = -0.54;

void setup() {
  Wire.begin();   //Join (create) the I2C bus
  Serial.begin (9600);  //Start serial communication with the host PC

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
  pinMode(6, OUTPUT);   //6,7 forward/backwards motor 1
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);   //8,9 forward/backwards motor 2
  pinMode(9, OUTPUT);
  pinMode(10,OUTPUT);   //10,11 pwm speed control motor 1 & 2
  pinMode(11,OUTPUT);
}

void loop() {

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

  Wire.endTransmission(true);

  Serial.println (gz);

  delay (500);  //slow down program to make serial output readable
}
