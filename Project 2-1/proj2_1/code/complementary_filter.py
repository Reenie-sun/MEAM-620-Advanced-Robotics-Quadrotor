# %% Imports

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation
from scipy.linalg import expm, sinm, cosm


# %%

def complementary_filter_update(initial_rotation, angular_velocity, linear_acceleration, dt):
    """
    Implements a complementary filter update

    :param initial_rotation: rotation_estimate at start of update
    :param angular_velocity: angular velocity vector at start of interval in radians per second
    :param linear_acceleration: linear acceleration vector at end of interval in meters per second squared
    :param dt: duration of interval in seconds
    :return: final_rotation - rotation estimate after update
    """

    #TODO Your code here - replace the return value with one you compute\

    # 1. update rotation estimate using angular velocity
    ini_rotation_mat = initial_rotation.as_matrix()     # initial_rotation is an object
    [omega_x, omega_y, omega_z] = np.asarray(angular_velocity)       # automatically assigned
    omega_skew = np.array([[0, -omega_z, omega_y], [omega_z, 0, -omega_x], [-omega_y, omega_x, 0]])     # skew rotation matrix
    R_k = expm(np.dot(dt, omega_skew))
    R_1k = np.dot(ini_rotation_mat, R_k)

    # 2. error measure, the acceleration is measured in g's, so the 1 -> 9.8
    a_k_norm = norm(linear_acceleration)
    e_m = np.abs((9.8 - a_k_norm) / 9.8)
    # print(e_m)
    if e_m >= 0 and e_m <= 0.1:
        alpha = 1
    elif e_m > 0.1 and e_m < 0.2:
        alpha = 1 - 10 * (e_m - 0.1)
    elif e_m >= 0.2:
        alpha = 0

    # 3.
    g_mod = np.dot(R_1k, linear_acceleration).reshape((3,1))      # g' reshape into column vector
    g_prem = g_mod / norm(g_mod)            # normalize into unit length
    # print(norm(g_prem))
    g_x, g_y, g_z = g_prem

    q_acc = [0, g_z/np.sqrt(2*(g_x + 1)), -g_y/np.sqrt(2*(g_x + 1)), np.sqrt((g_x+1)/2)]
    q_acc_prem = np.dot((1 - alpha), [0,0,0,1]) + np.dot(alpha, q_acc)     # the quaternion [x,y,z, w]
    rot_correction = Rotation.from_quat(q_acc_prem)
    R_acc = rot_correction.as_matrix()

    R_correct = np.dot(R_acc, R_1k)
    final_rotation = Rotation.from_matrix(R_correct)
    return final_rotation
