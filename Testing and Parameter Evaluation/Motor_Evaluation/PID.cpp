#include <stdint.h>
#include <Arduino.h>

/**The PID class manages all functions related to the control loop and implements a 
 * standard PID controller. The measured value is not sppecific to anything and so 
 * an instance of this class can in theory handle any relevant variable (position,
 * speed, current, etc). The controller operates in discrete time and makes the 
 * assumption that the time step set in the constructor will be respected by the main
 * program, necessitating the use of interrupt service routines.
 */

class PID
{
  private:
  float kp;  //Controller gains
  float ki;
  float kd;

  float N = 5;  //Low pass derivative filter gain

  float b2; //calculated gains for discrete time pid controller
  float b1;
  float b0;
  float a2;
  float a1;
  float a0;

  int32_t err [3];     //Tracked errors. Only errors k-1 and k-2 are stored since the current error can be calculated each loop

  int32_t reference;  //reference position specified in counts

  float u [3];  //holds current and previous two control inputs

  float t; //time step for the controller (seconds)

  public:
  PID (float p, float i, float d, float dt)
  {
    kp = p;
    ki = i;
    kd = d;

    err [0] = reference;  //initial errors are the entire distance from reference to 0
    err [1] = reference;
    err [2] = reference;

    u[0] = 0; //initial input is 0
    u[1] = 0;
    u[2] = 0;

    reference = 0;

    t = dt;

    float a = kp + N*kd;
    float b = N*kp + ki;
    float c = N*ki;

    b2 = a - b + c;
    b1 = 2*(c - a);
    b0 = a + b + c;
    a2 = 1 - N;
    a1 = -2;
    a0 = 1 + N;
  }

  /**main function to generate input signal. Assumes the time step dt is respected by the main
   * program. A timer interrupt is to be used to ensure it is. The controller assumes continuous
   * time so dt must be small to prevent errors.
    */
  float generateInput (int32_t currentVal)        //VERIFY NEW ALGORITHM  
  {
    err[2] = err[1];  //update errors
    err[1] = err[0];
    err[0] = reference - currentVal;
    
    u[2] = u[1];  //update control input history
    u[1] = u[0];
    u[0] = 0.11*(-a1*u[1]/a0 - a2*u[2]/a0 + b0*err[0]/a0 + b1*err[1]/a0 + b2*err[2]/a0); //scale to account for motor constant

    if (u[0] > 255)   //anti windup check
    {
      u[0] = 255;
    }
    if (u[0] < -255)
    {
      u[0] = -255;
    }

    return u[0] + 32*abs(err[0])/err[0]; //add to account for coulomb friction
  }

  void printErr ()
  {
    Serial.print ("Err: ");
    Serial.println (err[0]);
  }

  void printReference ()
  {
    Serial.print ("Reference: ");
    Serial.println(reference);
  }

  int32_t getReference ()
  {
    return reference;
  }

  void setReference (int32_t ref)
  {
    reference = ref;
  }
};
