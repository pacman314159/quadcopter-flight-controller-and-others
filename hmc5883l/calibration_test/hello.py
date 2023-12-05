import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import numpy as np

plt.figure(figsize=(10, 10))
ax = plt.axes(projection ="3d")  

numData = 0
X, Y, Z = [], [], []

x_max, y_max, z_max = -1e8, -1e8, -1e8
x_min, y_min, z_min = 1e8, 1e8, 1e8

def get_data_dynamic():
    ser = serial.Serial("COM3", 19200)
    while(True):
        rawData = str(ser.readline())
        rawData = rawData[:-5]
        rawData = rawData[2:]
        if(rawData == "quit"): break
        print(rawData) 

        numData += 1
        rawData = rawData.split(" ")[:-1];
        x, y, z, = [int(i) for i in rawData]
        X.append(x)
        Y.append(y)
        Z.append(z)
        omegaArray.append([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*z*y, 2*x, 2*y, 2*z])
        if(numData > 1000):
            numData = 1000
            X.pop(0)
            Y.pop(0)
            Z.pop(0)

def get_data_file():
    file = open("data.txt", "r")
    global numData, X, Y, Z
    global x_max, y_max, z_max, x_min, y_min, z_min

    for row in file:
        numData += 1
        x, y, z = [int(i) for i in row.split(" ") if i != "\n"]
        x_max , x_min = max(x_max, x), min(x_min, x)
        y_max , y_min = max(y_max, y), min(y_min, y)
        z_max , z_min = max(z_max, z), min(z_min, z)
        X.append(x)
        Y.append(y)
        Z.append(z)
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)

get_data_file()

hard_iron = np.array([(x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2]).T
mag_mes = np.array([X, Y, Z]).T
mag_mes = mag_mes - hard_iron

D = np.array([
    mag_mes[:,0]**2,
    mag_mes[:,1]**2,
    mag_mes[:,2]**2,
    2*mag_mes[:, 0]*mag_mes[:,1],
    2*mag_mes[:, 0]*mag_mes[:,2],
    2*mag_mes[:,1]*mag_mes[:,2],
    2*mag_mes[:, 0],
    2*mag_mes[:,1],
    2*mag_mes[:,2]
]).T
ones = np.ones((numData, 1))
V = (np.linalg.inv(D.T @ D) @ D.T @ ones).reshape(9)

M = np.array([
    [V[0], V[3], V[4]],
    [V[3], V[1], V[5]],
    [V[4], V[5], V[2]]
])
# B = np.array([V[6], V[7], V[8]]).T
# hard_iron = -1 * B @ np.linalg.inv(M)
soft_iron = np.linalg.cholesky(M)
mag_cal = mag_mes @ soft_iron

temp = -1 * mag_cal

ax.set_zlim(-1.2,1.2)
ax.set_aspect('equal')
ax.scatter(mag_cal[:, 0], mag_cal[:, 1], mag_cal[:, 2], color = "orange")
ax.scatter(temp[:, 0], temp[:, 1], temp[:, 2], color = "orange")
plt.show()