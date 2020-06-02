import numpy as np
import math
from scipy.spatial.transform import Rotation as R
q=[0,0,0,1]
rotation = R.from_quat(q)
R_matrix = rotation.as_matrix()


u1 = np.transpose(np.dot(R_matrix,[[0], [0], [1]]))
b = 2*u1 / np.linalg.norm(u1)
c = np.sqrt(2*R_matrix)
ccp_range = np.arange(0, 1, 0.01)   # the end value is not counted
for iter in range(0, 100):
    ccp_alpha_value = iter / 100  # ccp_alpha = 0 - 1 with 0.1 step length ; it is the current ccp value
    print(ccp_alpha_value)
    print('============')
    print(type(ccp_alpha_value))
#print(c)
x        = np.zeros((3,))
x_dot    = np.zeros((3,))
x_ddot   = np.zeros((3,))           # set this to be zero instead of using fitting
x_dddot  = np.zeros((3,))
x_ddddot = np.zeros((3,))
yaw = 0
yaw_dot = 0
points = np.array([
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [1, 1, 1]])

former_point = points[2,:]
next_point = points[3,:]
in_section_time = 0.8
displacement = next_point - former_point
if in_section_time < 0.5:  # acceleration part
    x_ddot = np.dot(4, displacement)  # constant acceleration to the mid point then decelerate
    x_dot = np.dot(in_section_time, x_ddot)
    x = np.dot(in_section_time, x_dot) / 2 + former_point  # former coordinate plus the distance covered
elif in_section_time == 0.5:  # turning point/ mid point
    x = (former_point + next_point) / 2
    x_dot = np.dot(2, displacement)  # reaching the highest speed
elif in_section_time > 0.5:
    x_ddot = np.dot(-4, displacement)  # constant acceleration away from mid point
    x_dot = np.dot(2, displacement) + np.dot(in_section_time - 0.5, x_ddot)  # plus minus value
    x = np.dot(np.square(in_section_time - 0.5), x_ddot) / 2 + (former_point + next_point) / 2 + np.dot(in_section_time - 0.5, np.dot(2, displacement))  # former coordinate plus the distance covered
print(x, x_dot, x_ddot)
# if time >= self.point_num - 1:
#     print(points[-1, :])
# next_point = points[3,:]
# dis = next_point - former_point
# ddot = np.dot(4, dis)
# dot = np.dot(-0.4, ddot)
# x = np.dot(0.4, dot) / 2
t=np.zeros((6, 3))
time_table = np.zeros((10, 1))

#print(np.cross([4,4,4], [0,0,1]))
# print([0,3,4]/np.linalg.norm([0,3,4]))
a=np.array([0,0,2])
b=np.array([0,3,2])
print((a+b)/2)