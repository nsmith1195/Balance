# Simulation of the nonlinear dynamics of the Wheeled Inverted Pendulum (WIP). Dynamics are
# integrated with the ODEInt function from scipy to allow testing of control laws applied
# at each time step. The outputs of the program are traces of the state variables
# [x, xdot, theta, thetaDot] as well as an animation of the WIP.

from scipy.integrate import odeint

import matplotlib.pyplot as plt
from matplotlib import style
from matplotlib import animation
from matplotlib import patches

import numpy as np

style.use('fivethirtyeight')

#Define system variables
g = 9.81    #defined as positive (m/s^2)

m1 = 0.058    #Wheel mass (kg)
m2 = 0.55    #body mass (kg)

j1 = 0.0024    #Wheel moment of inertia (kgm^2)
j2 = 0.001    #Body moment of inertia (kgm^2)

l = 0.0825     #Distance from wheel center axis to body center of mass (meters)
r = 0.03    #Radius of wheels (meters)

#Change of variables to simplify both derivation and translation to code
M = m1 + m2 + j1/(r**2) #Effective mass
I = m2*(l**2) + j2      #Effective moment of inertia

#Linear Model
A = np.array([[0, 1, 0, 0],[0, 0, ((m2**2)*(l**2)*g)/((m2**2)*(l**2) - M*I), 0],[0, 0, 0, 1],[0, 0, -(M*m2*g*l)/((m2**2)*(l**2) - M*I), 0]])
B = np.array([[0],[-(I + r*m2*l)/(r*((m2**2)*(l**2) - M*I))],[0],[(r*M - m2*l)/(r*((m2**2)*(l**2) - M*I))]])
C = np.array([0, 0, 1, 0])
D = 0

#State vector X = [x xDot theta thetaDot]

#Define simulation variables
dt = 0.01   #Timestep for the simulation
end = 3    #End time measured in seconds
t = np.arange(0, end, dt)  #Create a timeseries to run ODEInt against
energy_err = np.zeros_like(t)  #Create series to store system energy error (K - U)

u1 = np.empty(0) #Empty array to record torque values (units of Nm)

y0 = [0.0, -0.1, 0.2, 0.0]   #initial conditions for the simulation

# Return the energy in the system at this state. Used to verify energy is conserved
# in this model.
def calculateEnergy (y):
    x, xDot, theta, thetaDot = y

    energy = 0.5 * M * xDot**2 + 0.5 * I * thetaDot**2 + m2*l*np.cos(theta)*xDot*thetaDot + m2*g*l*np.cos(theta)

    return energy

#Callable function to return the nonlinear dynamics of the system to ODEInt.
def pend (y, t):
    x, xDot, theta, thetaDot = y    #enter state into readable variables
    u = controlInput(y)     #calculate input function for given state. U is a torque applied by the motor in Nm.

    #Derivatives of the state will be called: dy/dt = [v, a, omega, alpha]
    v = xDot

    #both linear and angular accelerations are very messy equations
    numa = (m2**2)*(l**2)*g*np.cos(theta)*np.sin(theta) - m2*l*I*np.sin(theta)*(thetaDot**2)
    dena = (m2**2)*(l**2)*np.cos(theta) - M*I

    a = numa/dena - ((I + r*m2*l*np.cos(theta))/(r*(m2**2)*(l**2)*np.cos(theta) - r*M*I))*u

    omega = thetaDot

    numt = -M*m2*l*g*np.sin(theta) + (m2**2)*(l**2)*np.sin(theta)*(thetaDot**2)
    dent = (m2**2)*(l**2)*np.cos(theta) - M*I

    alpha = numt/dent + ((r*M + m2*l)/(r*((m2**2)*(l**2)*np.cos(theta) - M*I)))*u

    dydt = [v, a, omega, alpha]     #Define the state derivative

    return dydt

#Function defining the controller input scheme. Currently PD
def controlInput (y):
    x, xdot, theta, thetaDot = y

    #PD
    kp = 1.2
    kd = 0.1

    u = kp*theta + kd*thetaDot

    #Direct Feedback
    # K = 1
    # u = K*theta

    return u

#Animation initializer function
def anim_init ():
    body.set_data([],[])
    time_text.set_text('hello')
    energy_err_text.set_text('hello')
    return body,wheel

#Animation function
def animate(i):
    wheel.center = (sol[i,0],r)
    body.set_data([sol[i,0],sol[i,0]+l*np.sin(sol[i,2])],[r,r+l*np.cos(sol[i,2])])
    time_text.set_text("Time: "+str(i*dt)+" s")
    energy_err_text.set_text ("Energy Error: "+"{:.3f}".format(energy_err[i]))
    title_text.set_text ('')

    return wheel,

#Run simulation
sol = odeint (pend, y0, t)

initialEnergy = calculateEnergy (y0)    #Initial system energy for energy_err calculations

for i in [0, energy_err.size - 1]:
    energy_err[i] = initialEnergy - calculateEnergy(sol[i,:])

#Plot results
traceFig = plt.figure()
trAx = traceFig.gca()
trAx.plot(t,sol[:,0],'b',label='x (m)')  #Plot each state variable with a lable and unique color
trAx.plot(t,sol[:,1],'g',label='xDot (m/s)')
trAx.plot(t,sol[:,2],'r',label='theta (rad)')
trAx.plot(t,sol[:,3],'k',label='thetaDot (rad/s)')

trAx.legend(loc="upper right")    #Display legend
trAx.set_title("State Variable Traces Direct Feedback (K = 0.4)")
trAx.set_xlabel("Time (s)")
trAx.set(xlim=(0,end), ylim=(-1.5,2.5))

traceFig.subplots_adjust(bottom=0.1)    # Make room for the x label

#Animate the results to give intuitive understanding of results
aniFig = plt.figure()  #Create an empty figure
ax = aniFig.gca()      #Get the current axis from the new figure
ax.axis('equal')    #Make the axes scale equally to preserve wheel shape
ax.set(xlim=(-1.5, 1.5), ylim=(-0.1,0.25))
ax.set_title("Direct Feedback w/ K = 0.4")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
aniFig.subplots_adjust(bottom=0.15, left=0.15)

time_text = ax.text(0.05, 0.1,'',horizontalalignment='left',verticalalignment='bottom', transform=ax.transAxes) #text to show time at top
energy_err_text = ax.text(0.05, 0.1,'',horizontalalignment='left',verticalalignment='top', transform=ax.transAxes)
title_text = ax.text(0.5,0.95,'',horizontalalignment='center',verticalalignment='center',transform=ax.transAxes)

wheel = patches.Circle((sol[0,0],r),r, fc='y')
body, = ax.plot([],[], lw=2)

ax.add_patch(wheel)

anim = animation.FuncAnimation(aniFig, animate, init_func=anim_init,frames=(sol[:,0].size), interval=dt*1000)

#Save animation as gif at 30fps
#anim.save('DirectFeedback.gif', writer='imagemagick', fps=30)

plt.show()
