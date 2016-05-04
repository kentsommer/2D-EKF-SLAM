#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

with open("./data/odom/odomRun.txt") as od:
    odomData = od.read()

with open("./data/features/featuresRun.txt") as fe:
    featuresData = fe.read()

odomData = [x for x in odomData.split('\n') if x != '']
featData = [x for x in featuresData.split('\n') if x != '']


odx = [row.split(' ')[0] for row in odomData]
ody = [row.split(' ')[1] for row in odomData]

fex = [row.split(' ')[0] for row in featData]
fey = [row.split(' ')[1] for row in featData]

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Map of run")    
ax1.set_xlabel('X')
ax1.set_ylabel('Y')

ax1.plot(odx,ody, c='r', label='the odomData')
ax1.plot(fex, fey, c='b', label='the featureData')

leg = ax1.legend()

plt.show()