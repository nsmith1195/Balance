# Balance
Balancing Robot Repository

Robot acts as an inverted pendulum with the goal of remaining stable while being mobile in at least one direction. Turning may be added as a later feature as wheels are independently actuated.

## Hardware
Almost all hardware used in this project is secondhand. The structural components come from an erectors set while the motors and encoders both come from an old printer. The benefit of using the motors and encoders from the printer is that they are already mounted to a backplate which can be cut to the proper size and shape to attach to the robot's body. A breadboard is also attached to the body to mount the IMU and provide a location to make all necessary connections.

## Connections
Power is provided to the system by a 9V battery connected to the L298N. The internal regulator on this chip provides 5V which is then used to power the power rail of a breadboard mounted to the robot. The power rail then powers the Arduino Uno and MPU 6050. Communication with the MPU 6050 is done via an I2C interface using the Wire library over wires connected to the A4 and A5 pins. The AD0 pin on the MPU 6050 is grounded on the Arduino setting its I2C address to 0x68. A 6 wire ribbon cable connects the Arduino to the L298N to allow for a digital input to power both motors forward and backwards while also providing a PWM input for speed control of each motor. The PWM inputs are connected to digital pins 5 and 6 while the remaining four wires are connected to digital pins 2 - 7.

## State Estimation
To measure each of the robot's degrees of freedom there are several sensors onboard. An inertial measurement unit uses measurements from a gyroscope and accelerometer to estimate the angle theta in the plane of motion. To combine these measures and reduce noise a complementary filter is used. An encoder is also used on each of the two wheels to measure the distance the robot has traveled. The angular position is incremented every time an interrupt is triggered by the rising edge of an encoder pulse. This has the consequence of halving the number of detectable encoder states but in this application it should be acceptable. Velocity estimation is done at a constant frequency by dividing the number of encoder counts measured over the time period. This was done instead of dividing each encoder step by the time between each step to avoid problems around 0 velocity where the estimator never finds out it stopped. This will likely be changed in the future to a more sophisticated algorithm due to issues with integration errors in this implementation.

#Dynamic Model
The equations of motion for a balancing robot were derived and will be used in the future to simulate and design a linear control system using some form of full state feedback to ensure both the translational and rotational degrees of freedom are stable. In the initial implementation, however, a simple pid loop will be used to stabilize the rotational axis while allowing translation to be free.
