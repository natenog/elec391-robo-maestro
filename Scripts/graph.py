#!/usr/bin/env python3

import serial
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

PORT = "COM3"
BAUDRATE = 115200
BUF_LEN = 1000

ser = serial.Serial(PORT, BAUDRATE, timeout=0)
app = pg.mkQApp("Encoder Plot")
window = pg.GraphicsLayoutWidget(title="Position")
plot = window.addPlot(title="Motor position")
curve = plot.plot()

window2 = pg.GraphicsLayoutWidget(title="Subtarget")
plot2 = window2.addPlot(title="Subtarget")
curve2 = plot2.plot()

encoderVals = deque([0]*BUF_LEN, maxlen=BUF_LEN)
timestamps = deque([0]*BUF_LEN, maxlen=BUF_LEN)
subTargets = deque([0]*BUF_LEN, maxlen=BUF_LEN)
props = deque([0]*BUF_LEN, maxlen=BUF_LEN)
integrals = deque([0]*BUF_LEN, maxlen=BUF_LEN)
derivs = deque([0]*BUF_LEN, maxlen=BUF_LEN)
outputs = deque([0]*BUF_LEN, maxlen=BUF_LEN)

def update():
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                break
            timestamp_str, encoderVal_str, subTarget_str, prop_str, integral_str, deriv_str, output_str = line.split(',')
            timestamp = float(timestamp_str)
            encoderVal = int(encoderVal_str)
            subTarget = int(subTarget_str)
            prop = float(prop_str)
            integral = float(integral_str)
            deriv = float(deriv_str)
            output = float(output_str)

            timestamps.append(timestamp)
            encoderVals.append(encoderVal)
            subTargets.append(subTarget)
            props.append(prop)
            integrals.append(integral)
            derivs.append(deriv)
            outputs.append(output)

        except Exception as e:
            print("PARSE ERROR:", e)

    if len(timestamps) > 1:
        time0 = timestamps[0]
        timeDelta = [time - time0 for time in timestamps]
        print("Time:", timestamps[-1] / 1000.0, "|", "Position:", encoderVals[-1], "|", "Subtarget:", subTargets[-1], "|", "Prop:", props[-1], "|", "Int:", integrals[-1], "|", "Deriv:", derivs[-1], "|", "Out:", outputs[-1])
        curve.setData(timeDelta, list(encoderVals))
        curve2.setData(timeDelta, list(subTargets))

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

window.show()
window2.show()
app.exec()