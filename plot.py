#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

with open("./data/odom/odomRun.txt") as od:
    odomData = od.read()

with open("./data/features/featuresRun.txt") as fe:
    featuresData = fe.read()

with open("./data/scan/scanRun.txt") as sc:
    scannerData = sc.read()

odomData = [x for x in odomData.split('\n') if x != '']
featData = [x for x in featuresData.split('\n') if x != '']
scanData = [x for x in scannerData.split('\n') if x != '']


odx = [row.split(' ')[0] for row in odomData]
ody = [row.split(' ')[1] for row in odomData]

fex = [row.split(' ')[0] for row in featData]
fey = [row.split(' ')[1] for row in featData]

scx = [row.split(' ')[0] for row in scanData]
scy = [row.split(' ')[1] for row in scanData]

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Map of run")    
ax1.set_xlabel('X')
ax1.set_ylabel('Y')

ax1.plot(scx, scy,'r.', label='the scannerData')
ax1.plot(odx,ody, 'bo', label='the odomData')
ax1.plot(fex, fey,'go', label='the featureData')

leg = ax1.legend()

plt.show()