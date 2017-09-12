import numpy as np
import matplotlib.pyplot as plt
import random

data1 = []
data2 = []
data3 = []
data4 = []
for val in range(100):
    data1.append(random.uniform(100-val,100-val+1))
    data2.append(random.uniform(90-val,90-val+1))
    data3.append(random.uniform(80-val,80-val+1))
    data4.append(random.uniform(70-val,70-val+1))
x1 = len(data1)
t = x1
t1 = np.linspace(0,x1,x1)
 
#plt.subplot(211)
#ax.get_legend_handles_labels()
line1,= plt.plot(t1,data1,'-r',label='no fusion')
line2,= plt.plot(t1,data2,'-g',label='with sensor fusion')
line3,= plt.plot(t1,data3, 'b',label='with fitness landscape')
line4,= plt.plot(t1,data4, 'y',label='with actuator feedback')

#red_patch = mpatches(color='red', label='The red data')
plt.legend([line1,line2,line3,line4])

plt.xlim(0,t)
plt.xlabel("time")
plt.ylim(0,100)
plt.ylabel("Speed")
plt.grid(True)
plt.show()

