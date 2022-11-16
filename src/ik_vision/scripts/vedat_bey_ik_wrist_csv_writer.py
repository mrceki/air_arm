#!/usr/bin/env python

# subscribe encoder data
# subscribe imu data
# output to csv file


import numpy as np
import csv

f = open('ik_wrist_theory_cross.csv', 'w')
writer = csv.writer(f)

X1 = -28.11

Y1 = 28.46

X2 = -28.46

Y2 = -28.11

Z= 22.5

Zz = 184.5

THETA1 = 0

for THETA2 in np.arange(-20.0, 20.1, 0.1):
#    for THETA2 in np.arange(-30.0, 30.1, 0.1):

    C1 =np.cos(np.deg2rad(THETA2))
    C2 =np.cos(np.deg2rad(THETA2))

    S1 =np.sin(np.deg2rad(THETA2))
    S2 =np.sin(np.deg2rad(THETA2))


    L1 = np.sqrt(np.square(X1-X1*C2)+np.square(Y1-(X1*S1*S2+Y1*C1))+np.square(Z-(Zz-X1*C1*S2+Y1*S1)))
    L2 = np.sqrt(np.square(X2-X2*C2)+np.square(Y2-(X2*S1*S2+Y2*C1))+np.square(Z-(Zz-X2*C1*S2+Y2*S1)))

    row = [162.0 - L1, 162.0 - L2, THETA1, THETA2]
    writer.writerow(row)
f.close()
