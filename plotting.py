import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("dataset/1-baseline_accel.csv", skiprows=1)

time = np.array(df.iloc[:, 0])
fx = np.array(df.iloc[:, 1])
fy = np.array(df.iloc[:, 2])
fz = np.array(df.iloc[:, 3])

#time = time - time[1]

plt.figure(1)
plt.plot(time, fx)
plt.plot(time, fy)
plt.plot(time, fz)
plt.grid(True)
plt.show()
