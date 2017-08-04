import matplotlib as mp
import matplotlib.pyplot as plot
import numpy as np
from math import pi

style = {
  "axes.linewidth": 3,
  "grid.linewidth": 3,
  "grid.linestyle": "solid",
  "grid.color": "#dadad0",
  "lines.linewidth": 3,
  "lines.markersize": 10,
  "xtick.labelsize": 24,
  "ytick.labelsize": 24,
}

for key in style:
  mp.rcParams[key]=style[key]

fig = plot.figure()
ax = fig.add_subplot(1, 1, 1)
ax.set_aspect('equal')
color1 = "#505050"
ax.spines['bottom'].set_color(color1)
ax.spines['top'].set_color(color1) 
ax.spines['right'].set_color(color1)
ax.spines['left'].set_color(color1)

ax.axhline(y=0, color=color1,zorder=4)
ax.axvline(x=0, color=color1,zorder=4)

ax.xaxis.grid()
ax.yaxis.grid()
ax.set_axisbelow(True)
ax.xaxis.set_tick_params(width=3,length=6,color=color1)
ax.yaxis.set_tick_params(width=3,length=6,color=color1)

t = np.arange(0,2*pi,0.02)
plot.axis([-1.5,1.5,-1,1])
plot.plot(np.cos(t),np.sin(2*t)/2,color='#0040b0',alpha=0.6,zorder=6,linewidth=6)

ax.text(1.3,-0.94,"x", fontsize=32, style="italic", color="#202020")
ax.text(-1.44,0.8,"y", fontsize=32, style="italic", color="#202020")

# plot.show()
plot.savefig("Gerono.svg",bbox_inches='tight')
