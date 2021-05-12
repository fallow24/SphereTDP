import numpy as np
import matplotlib.pyplot as plt

elev_max = np.pi / 180.0 * 38.4
azi_max = np.pi / 180.0 * 38.4

n = np.sqrt(20)
s = 100
t = np.arange(0, 1, 1e-5)

x = np.tan(elev_max / 2) * np.sin(s *
                                  (n * t + 0.25 * np.pi)) * np.sin(s * t)  #
y = np.tan(azi_max / 2) * np.sin(s * (n * t + 0.25 * np.pi)) * np.cos(s * t)  #


elev = np.arctan2(y, 1)
azi = np.arctan2(x, 1)

fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.plot(x, y)
ax2.plot(180.0 / np.pi * elev, 180.0 / np.pi * azi)
plt.show()
