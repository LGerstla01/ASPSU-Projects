import numpy as np

v = np.array([10, 20, 30, 40, 50, 100, 150, 200])
f_Tx = 76.5e9
c = 2.998e8

f_D = 2/c * f_Tx * v/3.6

print(np.round(f_D, 2))