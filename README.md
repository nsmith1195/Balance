# Balance
Balancing Robot Repository

Robot acts as an inverted pendulum with the goal of remaining stable while being mobile in at least one direction. Turning may be added as a later feature as wheels are independently actuated.

## Hardware
All of the structural parts and motors are secondhand, and in the case of the motors are likely counterfeits though I can't confirm this. Due to this the construction of the robot is very irregular. This presents several issues and opportunities in the design. For example the IMU had to be attached upside down at an odd angle due to other parts encroaching on its' planned location. Most references and other projects of this nature have their sensor placed flat and aligned such that the system is at a stationary point when the measured variable is at zero. One unexpected challenge was in finding a way to solidly attach the breadboard to the robots frame in a way that minimized out of plane readings. To solve this I had to sandwich the breadboard holding the IMU between two pieces of metal to allow it to be adjusted slightly while remaining secure.

## Connections
Power is provided to the system by a 9V battery connected to the L198N. The internal regulator on this chip provides 5V which is then used to power the power rail of a breadboard mounted to the robot. The power rail then powers the Arduino Uno and MPU 6050. Communication with the MPU 6050 is done via an I2C interface using the Wire library over wires connected to the A4 and A5 pins. The AD0 pin on the MPU 6050 is grounded on the Arduino setting its I2C address to 0x68. A 6 wire ribbon cable connects the Arduino to the L298N to allow for a digital input to power both motors forward and backwards while also providing a PWM input for speed control of each motor. The PWM inputs are connected to digital pins 10 and 11 while the remaining four wires are connected to digital pins 6-9.

## State Estimation
Currently the robot is assumed to have a 1 degree of freedom state (despite having a degree of freedom in the x direction). This has the consequence of allowing the robot to move freely so long as it remains balanced. The angle, theta, from the vertical is the only tracked variable and is measured as positive if the robot is falling forward. A complimentary filter is employed in the estimateState function though it may be swapped later for a Kalman Filter.

# Control Systems

