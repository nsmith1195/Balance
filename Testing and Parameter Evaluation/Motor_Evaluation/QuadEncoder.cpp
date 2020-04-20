/** Code to run the AEDS-96xx optical encoder. Due to the speed of the arduino only a quarter of possible states
 *  are used. This class assumes that the main program has already set up the appropriate pins as inputs. The 
 *  constructor of this class however attaches the appropriate interrupts.
 *    It is possible that in the future the micros function may be replaced with direct register reads on an 
 *  internal timer to increase accuracy and speed up the functions.
 */

#include <stdint.h>
#include <Arduino.h>

class QuadEncoder
{
  private:
  const int COUNTS = 720; //Encoder wheel believed to be 700 count/rev
  
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

  //return position measured in counts
  int32_t getPosition ()
  {
    return positionCounts[0];
  }

  //return position in radians
  float getPositionRad ()
  {
    return positionCounts[0] * 2*PI/COUNTS;
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
      estimateVelocity((int32_t)1);
    }
    else
    {
      positionCounts[0]--;
      estimateVelocity((int32_t)-1);
    }
  }

  /**Function implementing the velocity estimator for each motor. This is currently done by measuring the 
   * time between each encoder pulse. This assumes a constant motor speed which is non-zero.
   */
  void estimateVelocity (int32_t dir)
  {     
    static unsigned long lastRun; //time in microseconds of the last time the function ran
    unsigned long dt = micros() - lastRun;   //time since last run in microseconds

    lastRun += dt;  //update last run

    velocity = (float)(dir*1000000.0/dt); //velocity = dx/dt converted to counts/second
  }
};
