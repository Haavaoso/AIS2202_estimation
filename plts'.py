import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#pd_baseline_wrench = pd.read_csv("dataset/1-baseline_wrench.csv", skiprows=1)
#pd_baseline_accel = pd.read_csv("dataset/1-baseline_accel.csv", skiprows=1)
pd_baseline_wrench = pd.read_csv("dataset/2-vibrations_wrench.csv", skiprows=1)
pd_baseline_accel = pd.read_csv("dataset/2-vibrations_accel.csv", skiprows=1)
#pd_baseline_wrench = pd.read_csv("dataset/3-vibrations-contact_wrench.csv", skiprows=1)
#pd_baseline_accel = pd.read_csv("dataset/3-vibrations-contact_accel.csv", skiprows=1)

np_baseline_wrench = pd_baseline_wrench.to_numpy().T
np_baseline_accel = pd_baseline_accel.to_numpy().T
forceBias = [9.07633, -1.01814, 9.98482]
torqueBias = [0.432449, -0.692162, -0.156746]

estimated_baseline = pd.read_csv("data.csv")
estimated_baseline_Zc = pd.read_csv("dataEstimate.csv")
np_estimated_baseline = estimated_baseline.to_numpy().T
np_Zc = estimated_baseline_Zc.to_numpy().T

time = np.arange(len(np_baseline_accel[0]))
kalmanTime = np.linspace(0,len(time),len(np_estimated_baseline[0]))

plt.close(1); fig = plt.figure(1,figsize=(10,5))
plt.subplot(3,1,1)
plt.plot(time, np_baseline_accel[1]*9.81)
plt.plot(kalmanTime,np_estimated_baseline[2], linestyle='-', color='r',linewidth=2)

plt.subplot(3,1,2)
plt.plot(time, np_baseline_accel[2]*9.81)
plt.plot(kalmanTime,np_estimated_baseline[0], linestyle='-', color='r',linewidth=2)

plt.subplot(3,1,3)
plt.plot(time, np_baseline_accel[3]*9.81)
plt.plot(kalmanTime,-np_estimated_baseline[1], linestyle='-', color='r',linewidth=2)
plt.grid(True)
plt.show()

time = np.arange(len(np_baseline_wrench[0]))
kalmanTime = np.linspace(0,len(time),len(np_estimated_baseline[0]))

plt.close(2); plt.figure(2,figsize=(10,5))



plt.subplot(3,2,1)
plt.plot(time, np_baseline_wrench[1]-forceBias[0])
plt.plot(kalmanTime,np_Zc[0])
plt.plot(kalmanTime,np_estimated_baseline[3], linestyle='-', color='r',linewidth=2)

plt.subplot(3,2,2)
plt.plot(time, np_baseline_wrench[2]-forceBias[1])
plt.plot(kalmanTime,np_Zc[1])
plt.plot(kalmanTime,np_estimated_baseline[4], linestyle='-', color='r',linewidth=2)

#plt.subplot(2,1,1)
plt.subplot(3,2,3)
plt.plot(time, np_baseline_wrench[3]-forceBias[2])
plt.plot(kalmanTime,np_Zc[2])
plt.plot(kalmanTime,np_estimated_baseline[5], linestyle='-', color='r',linewidth=2)

plt.subplot(3,2,4)
plt.plot(time, np_baseline_wrench[4]-torqueBias[0])
plt.plot(kalmanTime,np_Zc[3])
plt.plot(kalmanTime,np_estimated_baseline[6], linestyle='-', color='r',linewidth=2)

plt.subplot(3,2,5)
#plt.subplot(2,1,2)
plt.plot(time, np_baseline_wrench[5]-torqueBias[1])
plt.plot(kalmanTime,np_Zc[4])
plt.plot(kalmanTime,np_estimated_baseline[7], linestyle='-', color='r',linewidth=2)

plt.subplot(3,2,6)
plt.plot(time, np_baseline_wrench[6]-torqueBias[2])
plt.plot(kalmanTime,np_Zc[5])
plt.plot(kalmanTime,np_estimated_baseline[8], linestyle='-', color='r',linewidth=2)

plt.show()
