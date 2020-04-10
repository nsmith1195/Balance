# Program to walk through the attempt to design a direct feedback controller for
# the linearized WIP using the Python-Controls library. Due to some quirks in the
# rlocus function for state space systems the model was converted to a Transfer
# Function relating theta and the input torque.
# The pole zero plot shows this is an unstable 2nd order system with both poles
# on the real axis symmetric to the origin, while the root locus plot shows that
# adding direct feedback with a gain only moves the poles further from the origin.
# Therefore a more complicated controller is necessary.

from control import *
import numpy as np
import matplotlib.pyplot as plt

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

#Names a23, a43, etc refer to state space model. A(row 2, col 3)
a23 = ((m2**2)*(l**2)*g)/((m2**2)*(l**2) - M*I)
a43 = -(M*m2*g*l)/((m2**2)*(l**2) - M*I)

b2 = -(I + r*m2*l)/(r*((m2**2)*(l**2) - M*I))
b4 = (r*M + m2*l)/(r*((m2**2)*(l**2) - M*I))

plant = tf (-b4, ([1, 0, -a43]))

print (plant)

plt.figure (0)
pzmap (plant)
plt.title("Pole Zero Map of Theta WRT Torque")
plt.grid(True)


plt.figure (1)
plt.xlim(-20,20)
plt.grid(True)
pzmap (plant)

plt.title("Root Locus Plot Theta WRT Torque")

#Get poles and gains to plot root locus
[rlist, klist] = rlocus (plant, Plot=False)

#Each branch is held in a column of rlist. Loop over columns and plot each
for col in rlist.T:
    plt.plot(col.real, col.imag)

#Pick two gains and annotate them on the plot to show the direction the poles move
i = 10
plt.annotate ('K = {:.2f}'.format(klist[i]), (rlist[i,1].real,rlist[i,1].imag), (8,-400), arrowprops=dict(facecolor='black', shrink=0.05, width=2))
i = 70
plt.annotate ('K = {:.2f}'.format(klist[i]), (rlist[i,1].real,rlist[i,1].imag), (5,500), arrowprops=dict(facecolor='black', shrink=0.05, width=2))

plt.show ()
