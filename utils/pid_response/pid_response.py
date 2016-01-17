#!/bin/python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import time
import re
from collections import deque

ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=460800);

# number of points to be displayed on the plot
points = 1000

# lists to hold received data
yawTgt   = deque([0] * points)
yawCur   = deque([0] * points)
pitchTgt = deque([0] * points)
pitchCur = deque([0] * points)
rollTgt  = deque([0] * points)
rollCur  = deque([0] * points)
motFL    = deque([0] * points)
motFR    = deque([0] * points)
motBL    = deque([0] * points)
motBR    = deque([0] * points)

x_axis = range(0, points)
recv = ''

# TODO react to ctrl+c, quit gracefully

r = re.compile('[ \t\n\r:]+')

def data_update():
    vals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    while True:

        try:
            recv = ser.readline().decode('utf-8')

            # process a received line from the serial port
            vals = r.split(recv)

        except:
            print('error splitting')
            return

        #print(vals)
        yield vals

fig = plt.figure()

# ax1 = fig.add_subplot(2, 1, 1)
# yawCurPlot, = ax1.plot([], [], linestyle='-', color='r')
# yawTgtPlot, = ax1.plot([], [], linestyle='-', color='b')
# ax1.set_xlim(0, points)
# ax1.set_ylim(-1800, 1800)
# ax1.grid()

# ax2 = fig.add_subplot(2, 1, 2)
ax2 = fig.add_subplot(2, 1, 1)
pitchCurPlot, = ax2.plot([], [], linestyle='-', color='r')
pitchTgtPlot, = ax2.plot([], [], linestyle='-', color='b')
ax2.set_xlim(0, points)
ax2.set_ylim(-5000, 5000)
ax2.grid()

# ax2 = fig.add_subplot(2, 1, 2)
ax3 = fig.add_subplot(2, 1, 2)
rollCurPlot, = ax3.plot([], [], linestyle='-', color='r')
rollTgtPlot, = ax3.plot([], [], linestyle='-', color='b')
ax3.set_xlim(0, points)
ax3.set_ylim(-5000, 5000)
ax3.grid()

def line_update(vals):
    try:
        yawTgt.popleft()
        yawTgt.append(int(vals[0]))
        yawCur.popleft()
        yawCur.append(int(vals[1]))
        pitchTgt.popleft()
        pitchTgt.append(int(vals[2]))
        pitchCur.popleft()
        pitchCur.append(int(vals[3]))
        rollTgt.popleft()
        rollTgt.append(int(vals[4]))
        rollCur.popleft()
        rollCur.append(int(vals[5]))
        motFL.popleft()
        motFL.append(int(vals[6]))
        motFR.popleft()
        motFR.append(int(vals[7]))
        motBL.popleft()
        motBL.append(int(vals[8]))
        motBR.popleft()
        motBR.append(int(vals[9]))
    except:
        # print('data format error')
        print(recv)
        pass

    # yawCurPlot.set_data(x_axis, yawCur)
    # yawTgtPlot.set_data(x_axis, yawTgt)
    pitchCurPlot.set_data(x_axis, pitchCur)
    pitchTgtPlot.set_data(x_axis, pitchTgt)
    rollCurPlot.set_data(x_axis, rollCur)
    rollTgtPlot.set_data(x_axis, rollTgt)
    # return yawCurPlot, yawTgtPlot, pitchCurPlot, pitchTgtPlot
    return pitchCurPlot, pitchTgtPlot, rollCurPlot, rollTgtPlot
aniYawCur = animation.FuncAnimation(fig, line_update, data_update, blit=True, interval=10, repeat=False)
plt.show()

ser.close()
