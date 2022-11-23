import os
import numpy as np
from matplotlib import pyplot as plt


t = np.arange(0.0, 10.0, 0.05)
v = 0.5 * t + 0.05 * t**2
s = 0.25 * t**2 + (0.05 / 3.0)*t**3
eps = np.random.randn(t.shape[0])

v_noisy = v + eps
s_noisy = s + eps


# cwd = os.getcwd()
# output_name = "noisy_data.csv"
# datapath = cwd + os.sep + os.pardir + os.sep + "data" + os.sep + output_name


# tup = (s_noisy, t)
# velocity_data = np.vstack(tup).T
# print(velocity_data.shape)
# with open(datapath, 'w') as f:
#     np.savetxt(datapath, velocity_data, delimiter=',')

filtered_data = np.loadtxt("/home/pradeep/pradeep/state_estimation/data/filtered_data.csv", delimiter=',', dtype=np.float32)

print(filtered_data.shape)


fig, axs = plt.subplots(2,1)

axs[0].plot(t, s_noisy)
axs[0].plot(t, s)
axs[0].plot(t, filtered_data[:,0])
axs[0].grid()
axs[0].set_title('distance vs time')
axs[0].set_xlabel('s')
axs[0].set_ylabel('m')

axs[1].plot(t, v_noisy)
axs[1].plot(t, v)
axs[1].plot(t, filtered_data[:,1])
axs[1].grid()
axs[1].set_title('velocity vs time')
axs[1].set_xlabel('s')
axs[1].set_ylabel('m/s')


# plt.axis('equal')

plt.show()