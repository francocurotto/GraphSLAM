import numpy as np
import matplotlib.pyplot as plt
from math import *

x = np.linspace(0, 1/(2*pi), 10000)
y = np.sqrt(-2*x*np.log(np.sqrt(2*pi*x)))

lw = 3
lfs = 28
plt.plot(x, y, linewidth=lw)
plt.axes().set_xlim([-0.001, 1/(2*pi)])
plt.grid(True)
plt.xlabel('$\sigma_{|{\Delta}_{i,j}|}^2$', fontsize =lfs)
plt.ylabel('max ${\mu}_{|{\Delta}_{i,j}|}$', fontsize =lfs)
#plt.show()
plt.savefig("maxVariance.pdf", bbox_inches='tight')
