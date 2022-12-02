import numpy as np
from numpy.random import default_rng
from matplotlib import pyplot as plt

rng = default_rng()

N_points = 100000
n_bins = 200

# define the function
def f(in_data):
    return np.exp(np.sin(in_data))

# Generate two normal distributions
dist1 = rng.standard_normal(N_points)
dist2 = f(dist1)

# Generate the plot for the function
min_x = np.min(dist1)
max_x = np.max(dist1)
step = (max_x - min_x) / n_bins
func_x = np.arange(min_x, max_x, step)
func_y = f(func_x)

fig, axs = plt.subplots(2, 2, sharex=True, tight_layout=True)

# We can set the number of bins with the *bins* keyword argument.
axs[1][1].hist(dist1, bins=n_bins)
axs[0][0].hist(dist2, bins=n_bins)
axs[0][1].plot(func_x, func_y)

plt.show()