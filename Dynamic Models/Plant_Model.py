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
m2 = 0.59    #body mass (kg)

j1 = 0.005    #Wheel moment of inertia (kgm^2)
j2 = 0.09    #Body moment of inertia (kgm^2)

l = 0.1     #Distance from wheel center axis to body center of mass (meters)
r = 0.03    #Radius of wheels (meters)

#Change of variables to simplify both derivation and translation to code
M = m1 + m2 + j1/(r**2) #Effective mass
I = m2*(l**2) + j2      #Effective moment of inertia
A = m2*l                #Change to help declutter

#State vector X = [x xDot theta thetaDot]

#Define simulation variables
dt = 0.01   #Timestep for the simulation
end = 10    #End time measured in seconds
t = np.arange(0, end, dt)  #Create a timeseries to run ODEInt against

y0 = [0.1, 0.0, 1.0, 0.0]   #initial conditions for the simulation

#Callable function to return the nonlinear dynamics of the system to ODEInt. Currently only homogenous solution
def pend (y, t):
    x, xDot, theta, thetaDot = y    #enter state into readable variables

    #Derivatives of the state will be called: dy/dt = [v, a, omega, alpha]
    v = xDot

    #both linear and angular accelerations are very messy equations
    numa = A*I*np.sin(theta)*(thetaDot**2) - g*(A**2)*np.sin(theta)*np.cos(theta)
    dena = M*I - (A**2)*(np.cos(theta)**2)

    a = numa/dena

    omega = thetaDot

    numt = (A**2)*(np.cos(theta)**2)*np.tan(theta)*(thetaDot**2) + g*A*M*np.sin(theta)
    dent = M*I - (A**2)*(np.cos(theta)**2)

    alpha = numt/dent

    dydt = [v, a, omega, alpha]     #Define the state derivative

    return dydt

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

plt.legend(loc="upper left")    #Display legend

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
