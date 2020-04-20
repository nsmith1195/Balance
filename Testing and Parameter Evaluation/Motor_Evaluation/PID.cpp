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

  int32_t err [2];     //Tracked variables
  int32_t accErr;
  float diff;       //float because it is read from encoder
  
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
  int32_t generateInput (int32_t currentVal)
  {
    float a = (float)(kp + ki*t/2.0 + kd/t);
    float b = (float)(-kp + ki*t/2.0 - 2*kd/t);
    float c = (float)kd/t;

    lastU = (int32_t)(lastU + a*(reference - currentVal) + b*err[1] + c*err[2]);   //Discrete time pid loop

    err[1] = err[0];
    err[0] = reference - currentVal;

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
