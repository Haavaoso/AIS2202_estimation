import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#df = pd.read_csv("dataset/1-baseline_wrench.csv", skiprows=1)
#df = pd.read_csv("dataset/3-vibrations-contact_wrench.csv", skiprows=1)
df = pd.read_csv("dataset/2-vibrations_wrench.csv", skiprows=1)


df1 = pd.read_csv("data.csv")

#9.07633 -1.01814, 9.98482
#torq 0.432449 -0.692162 -0.156746

time = np.arange(len(df))
fx = np.array(df.iloc[:, 1]) - 9.07633
fy = np.array(df.iloc[:, 2]) - 1.01814
fz = np.array(df.iloc[:, 3]) - 9.98482

tx = np.array(df.iloc[:, 5])-0.432449
ty = np.array(df.iloc[:, 5])-0.692162
tz = np.array(df.iloc[:, 5])-0.156746

ax_1 = np.array(df1.iloc[:, 0])
ay_1 = np.array(df1.iloc[:, 1])
az_1 = np.array(df1.iloc[:, 2])
fx_1 = np.array(df1.iloc[:, 3])
fy_1 = np.array(df1.iloc[:, 4])
fz_1 = np.array(df1.iloc[:, 5])
tx_1 = np.array(df1.iloc[:, 6])
ty_1 = np.array(df1.iloc[:, 7])
tz_1 = np.array(df1.iloc[:, 8])
time_Kalman = np.linspace(0,len(time),len(ax_1))

#time = time - time[1]

plt.figure(1)
#plt.plot(time, fx)
#plt.plot(time, fy)
plt.plot(time, fz, color='b')
plt.plot(time_Kalman,fz_1, linestyle='-', color='r')

plt.grid(True)
plt.show()
