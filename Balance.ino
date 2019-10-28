#include <Wire.h>

uint16_t ax, ay, az;
uint16_t gx, gy, gz;

uint8_t gyroSampleRate = 1;

uint8_t SMPLRT_DIV = (uint8_t)((8/gyroSampleRate) - 1); //Calculate the sample rate division and fit it into an 8bit int

uint8_t IMUADDRESS = byte(0x68); //I2C address of the MPU6050 (b1101000)

void setup() {
  Wire.begin();   //Join (create) the I2C bus
  Serial.begin (9600);  //Start serial communication with the host PC

  //Set the gyro full scale rate
  Wire.beginTransmission(IMUADDRESS);
  Wire.write(27); //register 27 sets the gyro rate
  Wire.write((uint8_t)0); //Set the gyro rate to +-250 deg/s
  Wire.endTransmission();
  
  //Set the accelerometer's full scale range
  Wire.beginTransmission(IMUADDRESS);  
  Wire.write(28); //Full scale value is stored in register 28
  Wire.write(0);  //0 corresponds to +- 2g
  Wire.endTransmission();

  //Set the gyroscope sample rate (Accelerometer sample rate is set at 1kHz)
  Wire.beginTransmission(IMUADDRESS); 
  Wire.write(25); //SMPLRT_DIV is stored in register 25. Sample rate is: gyroSampleRate = 8kHz/(1 + SMPLRT_DIV)
  Wire.write(SMPLRT_DIV);
  Wire.endTransmission();
  
}

void loop() {

  //The IMU automatically updates each sensor's reading at the rate set in the registers.
  Wire.beginTransmission(IMUADDRESS);
  Wire.write(59); //Read addresses 59-64 are accelerometer data.
  Wire.endTransmission();

  Wire.requestFrom(IMUADDRESS,6); //Request 6 bytes of data from IMU (Register already set to start at 59)

  if (6 <= Wire.available())  //if all of the data is available
  {
    ax = Wire.read(); //receive high bits of ax
    ax = ax << 8; //shift low bytes to be high set
    ax |= Wire.read();  //receive low bits of ax

    ay = Wire.read();
    ay = ay << 8;
    ay |= Wire.read();

    az = Wire.read();
    az = az << 8;
    az |= Wire.read();
  }

  Serial.println (ax);

  delay (500);  //slow down program to make serial output readable
}
