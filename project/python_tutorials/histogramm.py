import matplotlib.pyplot as plt
import numpy as np

with open("hist") as test:
    hist = []
    for line in test:
        hist.append(line.rstrip())

yBuf = []
y = []

for line in hist:
    Type = line.split(" ")
    yBuf.append(Type[2])

for l in yBuf:
    y.append(float(l[:2]))

print(y)

plt.xlabel("Counter")
plt.ylabel("Displacement", )

plt.hist(y,align="left",bins = 30)
plt.show()

