import numpy as np
import matplotlib.pyplot as plt

# parameters
w = 2.0
xl = 7.5
c = 2.0
x = np.linspace(-xl, xl, 100)
wv = np.array([y&z for (y,z) in zip(x>-w, x<w)])
nwv = np.array([not y for y in wv])

# kernels
quad = x**2
huber = ((x**2)/2)*wv + (w*(np.abs(x)-w/2))*nwv
cauchy = c**2/2 * np.log(1+(x/c)**2)
fair = c**2 * (np.abs(x)/c - np.log(1 + np.abs(x)/c))
geman = (x**2/2) / (1 + x**2)
turkey = (c**2/6)*((1-(1-(x/c)**2))**3)*wv + (c**2/6)*nwv
welsch = (c**2/2)*(1-np.exp(-(x/c)**2))
pseudo = 2*w**2*(np.sqrt(1+(x/w)**2)-1)

# plot
lw = 3
lfs = 20
plt.plot(x, quad, linewidth=lw, label='Quadratic')
plt.plot(x, huber, linewidth=lw, label='Huber')
plt.plot(x, fair, linewidth=lw, label='Fair')
plt.plot(x, turkey, linewidth=lw, label='Turkey')
plt.plot(x, geman, linewidth=lw, label='Geman-McClure')
plt.plot(x, cauchy, linewidth=lw, label='Cauchy')
plt.plot(x, welsch, linewidth=lw, label='Welsch')
plt.plot(x, pseudo, linewidth=lw, label='Pseudo Huber', color = '#800000')

plt.xlim([-xl, xl])
plt.ylim([0, 10])
plt.grid(True)
plt.xlabel('$x$', fontsize=lfs)
plt.ylabel('$f(x)$', fontsize=lfs)

ax = plt.subplot(111)
box = ax.get_position()
#ax.set_position([box.x0, box.y0, box.width * 0.7, box.height])
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0)
#plt.legend(bbox_to_anchor=(1.1, 0.5))
#plt.show()
plt.savefig("kernels.pdf", bbox_inches='tight')
