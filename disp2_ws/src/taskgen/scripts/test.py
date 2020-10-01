import numpy as np
import matplotlib.pyplot as plt

a = np.load('/home/martin/disp2_ws/src/taskgen/scripts/costmap.npy', allow_pickle=True)
plt.figure()
plt.imshow(a)
plt.show()