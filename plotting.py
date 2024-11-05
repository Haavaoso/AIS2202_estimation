import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("dataset/1-baseline_wrench.csv", skiprows=1)

df1 = pd.read_csv("data.csv")

#9.07633 -1.01814, 9.98482
#torq 0.432449 -0.692162 -0.156746

time = np.arange(len(df))
fx = np.array(df.iloc[:, 1])
fy = np.array(df.iloc[:, 2])
fz = np.array(df.iloc[:, 3]) - 9.98482

ty = np.array(df.iloc[:, 5])-0.692162

kalman0 = np.array(df1.iloc[:, 0])
kalman1 = np.array(df1.iloc[:, 1])
kalman2 = np.array(df1.iloc[:, 2])
kalman3 = np.array(df1.iloc[:, 3])
kalman4 = np.array(df1.iloc[:, 4])
kalman5 = np.array(df1.iloc[:, 5])
kalman6 = np.array(df1.iloc[:, 6])
kalman7 = np.array(df1.iloc[:, 7])
kalman8 = np.array(df1.iloc[:, 8])
time_Kalman = np.linspace(0,len(time),len(kalman0))

#time = time - time[1]

plt.figure(1)
#plt.plot(time, fx)
#plt.plot(time, fy)
plt.plot(time, fz, color='b')
plt.plot(time_Kalman, kalman5, linestyle='-', color='r')

plt.grid(True)
plt.show()
