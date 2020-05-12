# Balance
<p align="center">
  <img src="./Dynamic Models/Images/Unforced Dynamics.gif" width="300" />
  <img src="./Controller Design/SISO/Direct Feedback/DirectFeedback(k = 0,4).gif" width="300" />
</p>

Balancing Robot Repository

Robot acts as an inverted pendulum with the goal of remaining stable while being mobile in at least one direction. Turning may be added as a later feature as wheels are independently actuated. A full report on this project can be found in Final.pdf which includes relevant derivations and explanations for design choices. Below is a short summary of each major section.


## Dynamic Model
The full equations of motion were derived for a Wheeled Inverted Pendulum (WIP) and were used to simulate the system with Scipy. It is assumed that the robot can only move in a straight line which greatly simplifies the calculations. This model was then linearized and used to generate several control laws. 

## Controller Design
To simplify the initial design the linearized model was further reduced to a single input single output transfer function relating the angle of the robot to the torque input. This allows the use of very simple frequency domain design tools such as the root locus, as well as the use of common control architectures like PD and direct feedback.


## Mechanical Design
The body of the robot is mostly composed of secondhand parts from various electronics. Several small parts such as wheel connectors were 3D modeled and printed.

## Electrical Design
The controller for the robot is an Arduino Uno which draws power from the 5 volt regulator on an L298 motor driver. This driver provides power for both motors. To allow for direct control of the torque to each motor, current sensing resistors are placed in series with each motor. To read each voltage a difference amplifier is used to amplify the small voltage drop and allow reading by the Arduinio ADC. Finally, to measure the state of the robot an MPU-6050 inertial measurement unit and quadrature encoders allow direct measurement of the angle and distance travelled. To manage many of these components a breadboard is mounted opposite from the Arduino where each component can easily attach.

## Software Design

