import math

import matplotlib.pyplot as plt
# Starting angle
theta = -math.pi/12

# Motor mount angle
phi = 0

# Change in theta (delta theta)
dTheta = 0

# Change in time (delta t)
dTime = 0.01

# Motor thrust
F = 15

# Thrust distance from center
L = .3

# Rotational intera
RI = .2

# Matpotlib outputs
x = []
y = []

# Open File
fout = open('output.out', 'w')

# Main loop
for i in range(1000):
    # Change theta by thrust (Fx=ma, Fx = RI/L * dtheta/dt,
    # Fx = Thrust * sin(phi), dtheta = dt * thrust * sin(phi) * L / RI
    dTheta += dTime * F * math.sin(phi) * L / RI
    theta += dTheta * dTime
    #theta += .005 * (i / 7 % 2) - Random wind

    # Plot every 5th
    if i % 5 == 0:
        x.append(i / 100)
        y.append(math.degrees(theta) % 360)

    fout.write(str(theta) + "\n")

    # PID loop
    #                 D           P
    phi = - dTheta * .5 - theta * 5
    # P = -5, D = -0.5

# Plot
plt.scatter(x,y)
plt.show()
