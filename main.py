import math

import matplotlib.pyplot as plt
theta = -math.pi/12
phi = 0
dTheta = 0
dTime = 0.01
F = 15
L = .3
RI = .2
x = []
y = []
fout = open('output.out', 'w')
for i in range(1000):
    dTheta += dTime * F * math.sin(phi) * L / RI
    theta += dTheta * dTime
    theta += .005 * (i / 7 % 2)
    if i % 5 == 0:
        x.append(i / 100)
        y.append(math.degrees(theta) % 360)
    #fout.write(str(theta) + "\n")
    phi = - dTheta * .5 - theta * 5

plt.scatter(x,y)
plt.show()
