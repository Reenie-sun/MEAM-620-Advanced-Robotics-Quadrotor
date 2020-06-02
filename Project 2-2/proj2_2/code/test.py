import numpy as np
a = np.array([1,2,3,4])
b,n,m,l = a
print(np.zeros((3, 3)))
from scipy.spatial.transform import Rotation
q = Rotation.from_quat([1, 0, 0,1])
vec = q.as_rotvec()
print(vec, vec.shape)
i1 = np.eye(3)
i2=i1
i3=i2
print(np.dot(i1,i2,i3))