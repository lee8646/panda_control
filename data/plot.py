import matplotlib.pyplot as plt
import numpy as np

time = []
tx = []
ty = []
tz = []
fx = []
fy = []
fz = []

with open('ftsensor.txt') as f:
    lines = f.readlines()
    
    for line in lines:
        l = line.strip().split('\t')
        time.append(l[0])
        tx.append(l[1])
        ty.append(l[2])
        tz.append(l[3])
        fx.append(l[4])
        fy.append(l[5])
        fz.append(l[6])

plt.figure(1)
plt.plot(np.array(time, dtype=np.float32), np.array(tx, dtype=np.float32))
plt.plot(np.array(time, dtype=np.float32), np.array(ty, dtype=np.float32))
plt.plot(np.array(time, dtype=np.float32), np.array(tz, dtype=np.float32))
plt.xlabel("time")
plt.ylabel("torque")
plt.legend(['tx','ty','tz'])

plt.figure(2)
plt.plot(np.array(time, dtype=np.float32), np.array(fx, dtype=np.float32))
plt.plot(np.array(time, dtype=np.float32), np.array(fy, dtype=np.float32))
plt.plot(np.array(time, dtype=np.float32), np.array(fz, dtype=np.float32))
plt.xlabel("time")
plt.ylabel("force")
plt.legend(['fx','fy','fz'])
plt.show()