import numpy as np
n = [2,3,4]
m = np.array([[2], [3], [4]])
# print(np.asarray(n), np.asarray(n).shape)
x, y, z = np.asarray(n)
# print(x)
omega_x = 3
omega_y = 4
omega_z = 5
omega_skew = np.array([[0, -omega_z, omega_y], [omega_z, 0, -omega_x], [-omega_y, omega_x, 0]])
omega_skew = np.exp(np.dot(0.2, omega_skew))
print(np.array(n),np.array(n).reshape((3,1)))
print('======')
print(np.dot(omega_skew, np.array(n).reshape((3,1))).reshape((3,1)))
# print(np.dot(3, [0,1,1,1]))
# print(-np.array([1,2,3,4]))
k = np.array([1,2,3,4,5,6,7,8]).reshape((8,1))
k1 = k[0:4]
k1 = np.vstack((k1, k[4:8]))
print(k1)
ooo = np.zeros(3)
ooo[0:2] = True
print(ooo)
print(np.eye(3))
