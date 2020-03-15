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

j1 = 0.005    #Wheel moment of inertia (kgm^2)
j2 = 0.2    #Body moment of inertia (kgm^2)

l = 0.0825     #Distance from wheel center axis to body center of mass (meters)
r = 0.03    #Radius of wheels (meters)

#Change of variables to simplify both derivation and translation to code
M = m1 + m2 + j1/(r**2) #Effective mass
I = m2*(l**2) + j2      #Effective moment of inertia
A = m2*l                #Change to help declutter

#State vector X = [x xDot theta thetaDot]

#Define simulation variables
dt = 0.01   #Timestep for the simulation
end = 3    #End time measured in seconds
t = np.arange(0, end, dt)  #Create a timeseries to run ODEInt against

u1 = np.empty(0) #Empty array to record torque values (units of Nm)

y0 = [0.0, 0.0, 0.2, 0]   #initial conditions for the simulation

#Callable function to return the nonlinear dynamics of the system to ODEInt.
def pend (y, t):
    x, xDot, theta, thetaDot = y    #enter state into readable variables
    u = controlInput(y)     #calculate input function for given state. U is a torque applied by the motor in Nm.

    #Derivatives of the state will be called: dy/dt = [v, a, omega, alpha]
    v = xDot

    #both linear and angular accelerations are very messy equations
    numa = A*I*np.sin(theta)*(thetaDot**2) - g*(A**2)*np.sin(theta)*np.cos(theta)
    dena = M*I - (A**2)*(np.cos(theta)**2)

    a = numa/dena + ((I + r*A*np.cos(theta))/(r*(M*I - (A**2)*(np.cos(theta)**2))))*u   #Added input term (Element of B vector). gives weird results

    omega = thetaDot

    numt = (A**2)*(np.cos(theta)**2)*np.tan(theta)*(thetaDot**2) + g*A*M*np.sin(theta)
    dent = M*I - (A**2)*(np.cos(theta)**2)

    alpha = numt/dent + (((1)/(r*A*np.cos(theta)))*(1 - M*(I + r*A*np.cos(theta))/(M*I - (A**2)*(np.cos(theta)**2))))*u     #TEST THIS

    dydt = [v, a, omega, alpha]     #Define the state derivative

    return dydt

#Function defining the controller input scheme. Currently PD
def controlInput (y):
    global u1   #Reference the global variable u1

    x, xdot, theta, thetaDot = y

    kp = 10.0
    kd = 2.0

    u = kp*theta + kd*thetaDot

    u1 = np.append(u1, u)

    return u

#Animation initializer function
def anim_init ():
    body.set_data([],[])
    time_text.set_text('hello')
    return body,wheel

#Animation function
def animate(i):
    wheel.center = (sol[i,0],r)
    body.set_data([sol[i,0],sol[i,0]+l*np.sin(sol[i,2])],[r,r+l*np.cos(sol[i,2])])
    time_text.set_text(i*dt)

    return wheel,

#Run simulation
sol = odeint (pend, y0, t)

#Plot results
plt.plot(t,sol[:,0],'b',label='x')  #Plot each state variable with a lable and unique color
plt.plot(t,sol[:,1],'g',label='xDot')
plt.plot(t,sol[:,2],'r',label='theta')
plt.plot(t,sol[:,3],'k',label='thetaDot')

plt.legend(loc="lower left")    #Display legend

#Plot Controller torque
fig1,ax1 = plt.subplots() #Create empty figure
ax1.plot(u1)                                            #TODO: u1 and t have different lengths. find out why
plt.legend(loc="lower left")

#Animate the results to give intuitive understanding of results
fig = plt.figure()  #Create an empty figure
ax = fig.gca()      #Get the current axis from the new figure
ax.axis('equal')    #Make the axes scale equally to preserve wheel shape
ax.set(xlim=(-1.5, 1.5), ylim=(-0.1,0.25))

time_text = ax.text(0.05, 0.95,'',horizontalalignment='left',verticalalignment='top', transform=ax.transAxes) #text to show time at top

wheel = patches.Circle((sol[0,0],r),r, fc='y')
body, = ax.plot([],[], lw=2)

ax.add_patch(wheel)

anim = animation.FuncAnimation(fig, animate, init_func=anim_init,frames=(sol[:,0].size), interval=dt*1000)

plt.show()
