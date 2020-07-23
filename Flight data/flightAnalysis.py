import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pandas import DataFrame
import sys

axes = ['X', 'Y', 'Z']
readings = 'acceleration'

df = pd.read_csv(sys.argv[1])

fig, ax = plt.subplots()

apogeeX = -1
maxAltitude = -1
for j in range(len(df["altitude"])):
    if (df["altitude"][j] > maxAltitude):
        maxAltitude = df["altitude"][j]
        apogeeX = j

totalAcceleration = []
for axis in axes:
    column = readings + axis
    for i in range(len(df[column])):
        value = pow(df[column][i],2)
        if (len(totalAcceleration) <= i):
            totalAcceleration.append(value)
        else:
            totalAcceleration[i] += value
        i += 1

minimumAcceleration = 999999999
locationOfMinimumAcceleration = -1

for a in range(len(totalAcceleration)):
    totalAcceleration[a] = pow(totalAcceleration[a], 0.5)
    if (totalAcceleration[a] < minimumAcceleration and ):
        minimumAcceleration = totalAcceleration[a]
        locationOfMinimumAcceleration = a

ax.plot(totalAcceleration, "b", label=readings)
ax.set_ylabel(readings)
ax.text(apogeeX, totalAcceleration[apogeeX], "Maximum height")

# ax2 = ax.twinx()
# ax2.plot(df["brake-percentage"], "r", label="Brake percentage")
# ax2.set_ylabel("Brake percentage")

lines = ax.get_lines()# + ax2.get_lines()
ax.legend(lines, [line.get_label() for line in lines], loc='lower right')

ax.set_title("Simulated air-brake percentage and height over time")
fig.canvas.set_window_title('Air brakes simulations')
plt.show()
