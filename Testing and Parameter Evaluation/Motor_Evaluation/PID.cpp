#include <stdint.h>
#include <Arduino.h>

class PID
{
  private:
  float kp;  //Controller gains
  float ki;
  float kd;

  int32_t err [2];     //Tracked variables
  int32_t accErr;
  float diff;       //float because it is read from encoder
  
  int32_t lastCount;  //previous position for speed calculations
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
  int32_t generateInput (int32_t count)
  {
    float a = (float)(kp + ki*t/2.0 + kd/t);
    float b = (float)(-kp + ki*t/2.0 - 2*kd/t);
    float c = (float)kd/t;

    lastU = (int32_t)(lastU + a*(reference - count) + b*err[1] + c*err[2]);   //Discrete time pid loop

    err[1] = err[0];
    err[0] = reference - count;

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
