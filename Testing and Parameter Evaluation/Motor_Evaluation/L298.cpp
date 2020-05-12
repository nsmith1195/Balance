/** Class to manage all motor commands. The decision to structure this function
 *  in an object representing the driver rather than the motor itself was mostly
 *  based on the fact that the Arduino doesn't directly send commands to the 
 *  motors. This structure also allows for cleaner code with respect to where
 *  leads are connected.
 *  It is assumed that PWM control is intended to be used on brushed DC motors
 *  and so the control functions are named accordingly
 * 
 */

#include <stdint.h>
#include <Arduino.h>

class L298
{
  private:
    uint8_t input1Pin;  //Define the pins on the arduino related to inputs 1-4
    uint8_t input2Pin;  //These pins are used for analog write
    uint8_t input3Pin;
    uint8_t input4Pin;

  public:
    L298 (uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
    {
      input1Pin = in1;
      input2Pin = in2;
      input3Pin = in3;
      input4Pin = in4;

      pinMode (input1Pin, OUTPUT);  //Ensure the arduino has these pins configured as outputs
      pinMode (input2Pin, OUTPUT);
      pinMode (input3Pin, OUTPUT);
      pinMode (input4Pin, OUTPUT);
    }

    //Overloaded constructor to allow for only one motor to be used
    L298 (uint8_t in1, uint8_t in2)
    {
      input1Pin = in1;
      input2Pin = in2;

      pinMode (input1Pin, OUTPUT);  //Ensure the arduino has these pins configured as outputs
      pinMode (input2Pin, OUTPUT);
    }

    //Function to alter pwm signal to motor 1. Takes control signal u 
    //which is currently an arbitrary number -255 - 255
    void commandMotor1 (float u)
    {
      //cap u between the saturation limits of analogwrite
      if (u > 255)
      {
        u = 255;
      }
      if (u < -255)
      {
        u = -255;
      }
            
      //If u is positive write to input 1 to go forward. Otherwise input 2
      if (u > 0)
      {
        analogWrite (input1Pin, u);
        analogWrite (input2Pin, 0);
        return;
      }

      analogWrite (input2Pin, abs(u));
      analogWrite (input1Pin, 0);
    }

    //Function to alter pwm signal to motor 2. Takes control signal u 
    //which is currently an arbitrary number -255 - 255
    void commandMotor2 (float u)
    {      
      //cap u between the saturation limits of analogwrite
      if (u > 255)
      {
        u = 255;
      }
      if (u < -255)
      {
        u = -255;
      }
      
      //If u is positive write to input 3 to go forward. Otherwise input 4
      if (u >= 0)
      {
        analogWrite (input3Pin, u);
        analogWrite (input4Pin, 0);
        return;
      }

      analogWrite (input4Pin, abs(u));
      analogWrite (input3Pin, abs(u));
    }

    //Function to shut off both motors
    void stop ()
    {
      commandMotor1 (0);
      commandMotor2 (0);
    }
};
