import numpy as np
import matplotlib.pyplot as plt
from math import *

def maxDis (s):
    return sqrt(-2*s*log(sqrt(2*pi*s)))

def disFunct (x, mu, s):
    return 1/sqrt(2*pi*s) * np.exp(-(((x-mu)**2) / (2*s)))

# parameters
s1 = 0.01
s2 = 0.0585
s3 = 0.15
m1 = maxDis(s1)
m2 = maxDis(s2)
m3 = maxDis(s3)

# variables

# function
scale = 5
x = np.linspace(m3-scale*s3, m1+scale*s3, 1000)
d1 = disFunct(x, m1, s1)
d2 = disFunct(x, m2, s2)
d3 = disFunct(x, m3, s3)

# plot
lw = 3
lfs = 28
plt.plot([m3-scale*s3, m3+scale*s3+1], [1, 1],'k', linewidth=lw, label='$\chi$')
plt.plot(x, d1, 'b', linewidth=lw, label='$\sigma_{|{\Delta}_{i,j}|}^2 = 0.01$')
plt.plot(x, d2, 'g', linewidth=lw, label='$\sigma_{|{\Delta}_{i,j}|}^2 = 0.0585$')
plt.plot(x, d3, 'r', linewidth=lw, label='$\sigma_{|{\Delta}_{i,j}|}^2 = 0.15$')
plt.plot([m1, m1], [0, 4],'b--')
plt.plot([m2, m2], [0, 4],'g--')
plt.plot([m3, m3], [0, 4],'r--')
plt.grid(True)
ax = plt.axis()
plt.axes().set_xlim([x[0], x[-1]])
plt.xlabel('${\mu}_{|{\Delta}_{i,j}|}$', fontsize=lfs)
plt.ylabel('$d_{i=j}$', fontsize =lfs)

ax = plt.subplot(111)
box = ax.get_position()
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0, prop={'size':lfs})
plt.savefig("maxDistance.pdf", bbox_inches='tight')
#plt.show()
